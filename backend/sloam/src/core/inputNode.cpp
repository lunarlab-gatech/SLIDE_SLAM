/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <cube.h>
#include <definitions.h>
#include <databaseManager.h>
#include <geometry_msgs/PoseStamped.h>
#include <graphWrapper.h>
#include <gtsam/geometry/Point3.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <robot.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sloamNode.h>
#include <sloam_msgs/EvaluateLoopClosure.h>
#include <sloam_msgs/ROSRangeBearing.h>
#include <sloam_msgs/SemanticLoopClosure.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt64.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vizTools.h>

#include <boost/array.hpp>
#include <deque>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "sloam_msgs/ROSRangeBearingSyncOdom.h"
#include "sloam_msgs/ROSSyncOdom.h"

class InputManager {
 public:
  explicit InputManager(ros::NodeHandle nh);
  void RunInputNode(const ros::TimerEvent &e);
  void saveRuntimeCommUsage();
  Robot robot;

 private:
  void resetAllFlags();
  float runInputNodeRate_;
  ros::Timer timer_;

  void updateLastPose(const StampedSE3 &odom, const int &robotID);

  double max_timestamp_offset_ = 0.01;

  bool callSLOAM(SE3 relativeRawOdomMotion, ros::Time stamp,
                 std::deque<StampedSE3> &odomQueue, const int &robotID);
  void PublishAccumOdom_(const SE3 &relativeRawOdomMotion);
  void Odom2SlamTf();
  void PublishOdomAsTf(const nav_msgs::Odometry &odom_msg,
                       const std::string &parent_frame_id,
                       const std::string &child_frame_id);

  SE3 computeSloamToVioOdomTransform(const SE3 &sloam_odom,
                                     const SE3 &vio_odom);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster broadcaster_;

  // params
  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string robot_ns_prefix_;
  int number_of_robots_;
  std::string odom_topic_;
  std::string robot_frame_id_;
  float minOdomDistance_;
  float minSLOAMAltitude_;

  // vars
  boost::shared_ptr<sloam::SLOAMNode> sloam_ = nullptr;

  bool publishTf_;
  ros::NodeHandle nh_;

  // robotID
  int hostRobotID_;

  bool turn_off_intra_loop_closure_;
  bool turn_off_rel_inter_robot_factor;
};

InputManager::InputManager(ros::NodeHandle nh)
    : nh_(nh), tf_listener_{tf_buffer_}, robot(nh) {
  nh_.param<float>("main_node_rate", runInputNodeRate_, 5.0);
  timer_ = nh.createTimer(ros::Duration(1.0 / runInputNodeRate_),
                          &InputManager::RunInputNode, this);
  nh_.param<float>("min_odom_distance", minOdomDistance_, 0.5);
  nh_.param<float>("min_robot_altitude", minSLOAMAltitude_, 0.0);
  // maxQueueSize_ = nh_.param("max_queue_size", 100);
  publishTf_ = nh_.param("publish_tf", false);
  nh_.param<int>("number_of_robots", number_of_robots_, 1);
  nh_.param<std::string>("robot_ns_prefix", robot_ns_prefix_, "robot");
  nh_.param<std::string>("odom_topic", odom_topic_, "odom");
  nh_.param<std::string>("robot_frame_id", robot_frame_id_, "robot");
  nh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
  nh_.param<std::string>("map_frame_id", map_frame_id_, "map");
  std::string node_name = ros::this_node::getName();
  std::string idName = node_name + "/hostRobotID";
  nh_.param<int>(idName, hostRobotID_, 0);
  nh_.param<bool>(node_name + "/turn_off_intra_loop_closure",
                  turn_off_intra_loop_closure_, false);
  nh_.param(node_name+"/turn_off_rel_inter_robot_factor", 
                  turn_off_rel_inter_robot_factor, true);

  auto sloam_ptr = boost::make_shared<sloam::SLOAMNode>(nh_);
  sloam_ = std::move(sloam_ptr);
}

void InputManager::RunInputNode(const ros::TimerEvent &e) {
  if (robot.robotOdomQueue_.size() == 0) {
    ROS_INFO_STREAM_THROTTLE(1, "Odom queue is not filled yet for robot "
                                   << robot.robotId_
                                   << ", waiting for odometry...");
    return;
  }

  // robot.robotOdomQueue_ is filled, execute the rest of the code
  SE3 highFreqSLOAMPose;
  ros::Time odom_stamp;

  // Publishing high freq pose, should use the latest odom pose
  auto cur_vio_odom = robot.robotOdomQueue_.back();
  if (robot.robotOdomReceived_) {
    // if odom has been updated in sloam, use the relative odom pose and add
    // it to the previous sloam pose
    SE3 latestRelativeMotionFactorGraph =
        robot.robotLatestOdom_.pose.inverse() * cur_vio_odom.pose;
    highFreqSLOAMPose =
        robot.robotLastSLOAMKeyPose_ * latestRelativeMotionFactorGraph;
    odom_stamp = cur_vio_odom.stamp;
  } else {
    // directly take current odometry and publish
    highFreqSLOAMPose = cur_vio_odom.pose;
    odom_stamp = cur_vio_odom.stamp;
  }

  // syncOdom is used for publishing the relative transform from semantic slam
  // reference frame to odometry reference frame for drift compensation in the
  // navigation stack
  sloam_msgs::ROSSyncOdom syncOdom;
  syncOdom.header.stamp = odom_stamp;
  auto odom_msg =
      sloam::toRosOdom_(highFreqSLOAMPose, map_frame_id_, odom_stamp);
  syncOdom.vio_odom =
      sloam::toRosOdom_(cur_vio_odom.pose, map_frame_id_, odom_stamp);
  syncOdom.sloam_odom = odom_msg;
  // calculate this for drift compensation
  SE3 sloam_to_vio_tf =
      computeSloamToVioOdomTransform(highFreqSLOAMPose, cur_vio_odom.pose);
  // publish sloam_to_vio_tf as a odometry message
  auto sloam_to_vio_msg =
      sloam::toRosOdom_(sloam_to_vio_tf, map_frame_id_, odom_stamp);
  robot.pubSloamToVioOdom_.publish(sloam_to_vio_msg);

  robot.pubRobotHighFreqSLOAMPose_.publish(
      sloam::makeROSPose(highFreqSLOAMPose, map_frame_id_, odom_stamp));

  robot.pubRobotHighFreqSLOAMOdom_.publish(odom_msg);
  robot.pubRobotHighFreqSyncOdom_.publish(syncOdom);

  // Adding Factors for Relative Inter-Robot Measurements
  if(!turn_off_rel_inter_robot_factor) {
    // For each received measurement
    for (int i = 0; i < robot.robotRelativeMeasQueue_.size(); i++) {
      // If measurement is at least robot.semantic_meas_delay_tolerance_ seconds old
      if ((robot.robotRelativeMeasQueue_.back().stamp - robot.robotRelativeMeasQueue_[i].stamp).toSec() 
            > robot.semantic_meas_delay_tolerance_) {

        // Create the stamped pose
        RelativeMeas relativeMeas = robot.robotRelativeMeasQueue_[i];
        StampedSE3 validStampedPose;
        validStampedPose.pose = relativeMeas.odomPose;
        validStampedPose.stamp = relativeMeas.stamp;

        // Create the observation (with only odometry, no objects)
        Observation latestObservation = Observation();
        latestObservation.stampedPose = validStampedPose;

        // Use odom to estimate motion since last key frame
        StampedSE3 raw_vio_odom_used_for_sloam = latestObservation.stampedPose;
        SE3 relativeRawOdomMotion = robot.robotLatestOdom_.pose.inverse() * raw_vio_odom_used_for_sloam.pose;

        // Get the Previous Key Pose
        SE3 prevKeyPose;
        if (robot.robotKeyPoses_.size() > 0) {
          prevKeyPose = robot.robotKeyPoses_[robot.robotKeyPoses_.size() - 1];
        } else {
          ROS_WARN("No previous key pose. Use identity as the previous key pose.");
          prevKeyPose = SE3();
        }

        // Send the relative measurement synched odometry to SLOAM
        SE3 keyPose;
        bool success = sloam_->runSLOAMNode(
            relativeRawOdomMotion, prevKeyPose, latestObservation.cylinders,
            latestObservation.cubes, latestObservation.ellipsoids,
            latestObservation.stampedPose.stamp, keyPose, hostRobotID_);
        if (success) {
          robot.robotKeyPoses_.push_back(keyPose);
          updateLastPose(raw_vio_odom_used_for_sloam, hostRobotID_);
        }

        // Extract Host Robot Data class
        sloam_->dbMutex.lock();
        robotData data = sloam_->dbManager.getHostRobotData();

        // Pass the relative inter-robot measurement to the database manager
        // and factor generation thread
        data.relativeMeasPacket.push_back(relativeMeas);
        sloam_->addRelativeMeasurement(relativeMeas);

        // Remove this value from the queue, as it's been used
        robot.robotRelativeMeasQueue_.erase(robot.robotRelativeMeasQueue_.begin() + i);
        sloam_->dbMutex.unlock();
        i--;

      } 
      // Remaining measurements are not old enough to add, so finish 
      else { break; } 
    }
  }

  // ADDING OTHER FACTORS
  // add odom factor only if the robot has moved enough
  bool add_odom_factor = false;
  bool valid_pose_found = false;
  StampedSE3 validStampedPose;
  if (robot.robotOdomUpdated_) {
    // search in the odom queue, find the first pose that is atleast
    // robot.semantic_meas_delay_tolerance_ seconds old because we want to avoid
    // adding pose nodes back and forth due to semantic measurements delay
    // iterate from the back of the queue to the front
    for (int i = robot.robotOdomQueue_.size() - 1; i >= 0; i--) {
      if ((robot.robotOdomQueue_.back().stamp - robot.robotOdomQueue_[i].stamp)
              .toSec() > robot.semantic_meas_delay_tolerance_) {
        // found the first pose that is atleast
        // robot.semantic_meas_delay_tolerance_ seconds old add odom factor
        // record this stamped pose
        validStampedPose = robot.robotOdomQueue_[i];
        // ROS_INFO_STREAM(
        //     "Found the first pose that is atleast "
        //     "robot.semantic_meas_delay_tolerance_ seconds old");
        valid_pose_found = true;
        break;
      }
    }

    if (valid_pose_found) {
      SE3 currRelativeMotion;
      // Use odom to estimate motion since last key frame
      currRelativeMotion =
          robot.robotLatestOdom_.pose.inverse() * validStampedPose.pose;
      double accumMovement = currRelativeMotion.translation().norm();
      // ROS_INFO_STREAM("current odom distance is " << accumMovement);
      bool moved_enough = accumMovement > minOdomDistance_;
      if (moved_enough) {
        add_odom_factor = true;
        // ROS_INFO_STREAM("Moved enough. Accumulated movement is "
        //                 << accumMovement);
      } else {
        // ROS_WARN_STREAM("Not moved enough. Accumulated movement is "
        //                 << accumMovement);
      }
    }
  }

  if (add_odom_factor || robot.robotObservationUpdated_) {
    // add both odometry and object factor
    Observation latestObservation;
    if (robot.robotObservationUpdated_) {
      // max distance to check for loop closure
      // TODO(xu): make this ROS param
      double max_dist_xy = 10;
      double max_dist_z = 2;  // 0.01; // approximately = 0.5 * floor height
      // at least how many poses away should be regarded as "revisiting" instead
      // of "consecutive" poses
      size_t at_least_num_of_poses_old = 30;
      // input pose for checking the candidate loop closure region
      SE3 inputPose = robot.robotObservationQueue_.back().stampedPose.pose;
      if (turn_off_intra_loop_closure_) {
        ROS_INFO_THROTTLE(5.0, 
            "Intra Loop closure is turned off, the default of the variable is "
            "false");
      }

      // check if we have the potential to establish a loop closure, if yes,
      // skip adding this factor, otherwise, add it
      if (sloam_->semanticMap_.InLoopClosureRegion(max_dist_xy, max_dist_z,
                                                   inputPose, hostRobotID_,
                                                   at_least_num_of_poses_old) &&
          !turn_off_intra_loop_closure_) {
        // still takes the latest observation as the input, because loop closure
        // node needs this however set the flag isInLoopClosureRegion_ to true
        // to avoid adding landmarks to the map!
        latestObservation = robot.robotObservationQueue_.back();
        // reset the flags
        robot.robotObservationUpdated_ = false;
        robot.robotOdomUpdated_ = false;
        sloam_->isInLoopClosureRegion_ = true;
        // ROS_WARN_STREAM(
        //     "Robot inside candidate loop closure region, skipping the semantic "
        //     "observation addition...");
      } else {
        // ROS_INFO_STREAM_THROTTLE(3.0, 
        //     "Robot outside candidate loop closure region, adding the semantic "
        //     "observation...");
        sloam_->isInLoopClosureRegion_ = false;
        // add factors
        latestObservation = robot.robotObservationQueue_.back();
        // reset the flags
        robot.robotObservationUpdated_ = false;
        robot.robotOdomUpdated_ = false;
      }
    } else {
      // reset the flag
      robot.robotOdomUpdated_ = false;
      latestObservation = Observation();
      // assemble observation with the valid pose
      latestObservation.stampedPose = validStampedPose;
    }

    SE3 keyPose;
    SE3 relativeRawOdomMotion;
    // Use odom to estimate motion since last key frame
    StampedSE3 raw_vio_odom_used_for_sloam = latestObservation.stampedPose;
    // VERY IMPORTANT: use latestObservation pose instead of odomQueue.back()
    // pose since we want the odom that is synced with the observation
    relativeRawOdomMotion = robot.robotLatestOdom_.pose.inverse() *
                            raw_vio_odom_used_for_sloam.pose;

    SE3 prevKeyPose;
    if (robot.robotKeyPoses_.size() > 0) {
      prevKeyPose = robot.robotKeyPoses_[robot.robotKeyPoses_.size() - 1];
    } else {
      ROS_WARN("No previous key pose. Use identity as the previous key pose.");
      prevKeyPose = SE3();
    }

    // timestamp is used for visualization
    bool success = sloam_->runSLOAMNode(
        relativeRawOdomMotion, prevKeyPose, latestObservation.cylinders,
        latestObservation.cubes, latestObservation.ellipsoids,
        latestObservation.stampedPose.stamp, keyPose, hostRobotID_);
    if (success) {
      robot.robotKeyPoses_.push_back(keyPose);
      updateLastPose(raw_vio_odom_used_for_sloam, hostRobotID_);
    }
  } else {
    ROS_INFO_STREAM_THROTTLE(3.0, "Neither the odometry nor the observation is updated for robot " << hostRobotID_);
  }

  if (sloam_->save_runtime_analysis) {
    saveRuntimeCommUsage();
  }
}

void InputManager::updateLastPose(const StampedSE3 &odom, const int &robotID) {
  robot.robotOdomReceived_ = true;
  robot.robotLatestOdom_.pose = odom.pose;
  robot.robotLatestOdom_.stamp = odom.stamp;
  if (robot.robotKeyPoses_.size() == 0) {
    robot.robotLastSLOAMKeyPose_ = robot.robotLatestOdom_.pose;
  } else {
    // used for calculating high freq pose
    robot.robotLastSLOAMKeyPose_ = robot.robotKeyPoses_.back();
  }
}

SE3 InputManager::computeSloamToVioOdomTransform(const SE3 &sloam_odom,
                                                 const SE3 &vio_odom) {
  // calculate the transform from sloam odom to vio odom
  SE3 sloam_to_vio_transform = vio_odom * sloam_odom.inverse();
  return sloam_to_vio_transform;
}

void InputManager::PublishOdomAsTf(const nav_msgs::Odometry &odom_msg,
                                   const std::string &parent_frame_id,
                                   const std::string &child_frame_id) {
  geometry_msgs::TransformStamped tf;
  tf.header = odom_msg.header;
  // note that normally parent_frame_id should be the same as
  // odom_msg.header.frame_id
  tf.header.frame_id = parent_frame_id;
  tf.child_frame_id = child_frame_id;
  tf.transform.translation.x = odom_msg.pose.pose.position.x;
  tf.transform.translation.y = odom_msg.pose.pose.position.y;
  tf.transform.translation.z = odom_msg.pose.pose.position.z;
  tf.transform.rotation = odom_msg.pose.pose.orientation;
  broadcaster_.sendTransform(tf);
}

void InputManager::saveRuntimeCommUsage() {
  // save the runtime communication usage variables to a txt file
  ROS_DEBUG("Saving runtime communication usage to a txt file 1...");
  std::ofstream file(sloam_->runtime_analysis_file,
                     std::ios::out | std::ios::trunc);
  if (!file.is_open()) {
    std::cerr << "Failed to open the file." << std::endl;
    return;
  }

  file << "Total number of landmarks: " << sloam_->total_number_of_landmarks
       << std::endl;
  file << "Trajectory length: " << sloam_->trajectory_length << std::endl;
  file << "Number of attempts for intra loop closure: "
       << sloam_->num_attempts_intra_loop_closure << std::endl;
  file << "Number of successful intra loop closure: "
       << sloam_->num_successful_intra_loop_closure << std::endl;
  file << "Number of attempts for inter loop closure: "
       << sloam_->num_attempts_inter_loop_closure << std::endl;
  file << "Number of successful inter loop closure: "
       << sloam_->num_successful_inter_loop_closure << std::endl;

  file << "Average factor adding and graph optimization time [s]: "
       << std::accumulate(sloam_->fg_optimization_time.begin(),
                          sloam_->fg_optimization_time.end(), 0.0) /
              sloam_->fg_optimization_time.size()
       << std::endl;
  file << "Average data association time [s]: "
       << std::accumulate(sloam_->data_association_time.begin(),
                          sloam_->data_association_time.end(), 0.0) /
              sloam_->data_association_time.size()
       << std::endl;
  file << "Average intra loop closure time [s]: "
       << std::accumulate(sloam_->intra_loop_closure_time.begin(),
                          sloam_->intra_loop_closure_time.end(), 0.0) /
              sloam_->intra_loop_closure_time.size()
       << std::endl;
  file << "Average inter loop closure time [s]: "
       << std::accumulate(sloam_->inter_loop_closure_time.begin(),
                          sloam_->inter_loop_closure_time.end(), 0.0) /
              sloam_->inter_loop_closure_time.size()
       << std::endl;

  // total communication usage
  file << "Total Publication Communication Usage [MB]: "
       << std::accumulate(sloam_->dbManager.publishMsgSizeMB.begin(),
                          sloam_->dbManager.publishMsgSizeMB.end(), 0.0)
       << std::endl;
  file << "Total Subscription Communication Usage [MB]: "
       << std::accumulate(sloam_->dbManager.receivedMsgSizeMB.begin(),
                          sloam_->dbManager.receivedMsgSizeMB.end(), 0.0)
       << std::endl;

  file << "Average publish msg size MB: "
       << std::accumulate(sloam_->dbManager.publishMsgSizeMB.begin(),
                          sloam_->dbManager.publishMsgSizeMB.end(), 0.0) /
              sloam_->dbManager.publishMsgSizeMB.size()
       << std::endl;
  file << "Average received msg size MB: "
       << std::accumulate(sloam_->dbManager.receivedMsgSizeMB.begin(),
                          sloam_->dbManager.receivedMsgSizeMB.end(), 0.0) /
              sloam_->dbManager.receivedMsgSizeMB.size()
       << std::endl;

  auto maximal_instant_publish_msg_size_MB =
      std::max_element(sloam_->dbManager.publishMsgSizeMB.begin(),
                       sloam_->dbManager.publishMsgSizeMB.end());
  if (maximal_instant_publish_msg_size_MB !=
      sloam_->dbManager.publishMsgSizeMB.end())
    file << "Maximal Instant publish msg size MB: "
         << *maximal_instant_publish_msg_size_MB << std::endl;

  auto maximal_instant_received_msg_size_MB =
      std::max_element(sloam_->dbManager.receivedMsgSizeMB.begin(),
                       sloam_->dbManager.receivedMsgSizeMB.end());
  if (maximal_instant_received_msg_size_MB !=
      sloam_->dbManager.receivedMsgSizeMB.end())
    file << "Maximal Instant received msg size MB: "
         << *maximal_instant_received_msg_size_MB << std::endl;

  for (auto iter = sloam_->dbManager.loopClosureTf.begin();
       iter != sloam_->dbManager.loopClosureTf.end(); iter++) {
    file << "Loop closure tf between " << std::to_string(hostRobotID_)
         << " and " << std::to_string(iter->first) << ": " << std::endl;
    file << iter->second.matrix() << std::endl;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sloam");
  ros::NodeHandle n("sloam");
  InputManager in(n);

  while (ros::ok()) {
    ros::spin();
  }

  return 0;
}
