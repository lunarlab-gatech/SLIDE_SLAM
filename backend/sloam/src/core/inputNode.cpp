/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <inputNode.h>

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

  // Print information about parameters
  if (turn_off_intra_loop_closure_) {
    ROS_INFO_ONCE("Intra Loop closure is turned off, the default of the variable is false");
  }
}

void InputManager::RunInputNode(const ros::TimerEvent &e) {
  if (robot.robotOdomQueue_.size() == 0) {
    ROS_INFO_STREAM_THROTTLE(1, "Odom queue is not filled yet for robot "
      << robot.robotId_ << ", waiting for odometry...");
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
  auto odom_msg = sloam::toRosOdom_(highFreqSLOAMPose, map_frame_id_, odom_stamp);
  syncOdom.vio_odom = sloam::toRosOdom_(cur_vio_odom.pose, map_frame_id_, odom_stamp);
  syncOdom.sloam_odom = odom_msg;

  // Calculate this for drift compensation
  SE3 sloam_to_vio_tf = computeSloamToVioOdomTransform(highFreqSLOAMPose, cur_vio_odom.pose);
  
  // publish sloam_to_vio_tf as a odometry message
  auto sloam_to_vio_msg = sloam::toRosOdom_(sloam_to_vio_tf, map_frame_id_, odom_stamp);
  robot.pubSloamToVioOdom_.publish(sloam_to_vio_msg);
  robot.pubRobotHighFreqSLOAMPose_.publish(sloam::makeROSPose(highFreqSLOAMPose, map_frame_id_, odom_stamp));
  robot.pubRobotHighFreqSLOAMOdom_.publish(odom_msg);
  robot.pubRobotHighFreqSyncOdom_.publish(syncOdom);

  // Find out if we have any measurements to add
  int meas_to_add;
  PickNextMeasurementToAdd(robot.robotOdomQueue_, robot.robotObservationQueue_, robot.robotRelativeMeasQueue_, 
           robot.robotLatestOdom_, ros::Time::now().toSec(), robot.msg_delay_tolerance, minOdomDistance_, meas_to_add);

  // Keep adding measurements as long as we have valid ones to add
  while(meas_to_add != 0) {
    // Holds measurement information to add to factor graph
    Observation latestObservation; 

    // Extract measurement information (depends on measurement type)
    switch(meas_to_add) {
      case 0: { // No measurements to add
        break;
      }
      case 1: { // Odometry Measurement
        latestObservation = Observation();
        latestObservation.stampedPose = robot.robotOdomQueue_.front();
        robot.robotOdomQueue_.pop_front();
        break;
      }
      case 2: { // Object Observation Measurement

        // Set variables for InLoopClosureRegion
        double max_dist_xy = 10;
        double max_dist_z = 2;  // 0.01; // approximately = 0.5 * floor height
        size_t at_least_num_of_poses_old = 30;
        SE3 inputPose = robot.robotObservationQueue_.front().stampedPose.pose;

        // Check if we have the potential to establish a loop closure
        // If yes, skip adding this factor, otherwise, add it
        if (!turn_off_intra_loop_closure_ && 
            sloam_->semanticMap_.InLoopClosureRegion(max_dist_xy, max_dist_z, 
                          inputPose, hostRobotID_, at_least_num_of_poses_old)) {
          sloam_->isInLoopClosureRegion_ = true;
        } else {
          sloam_->isInLoopClosureRegion_ = false;
        }

        // Pass our observation (with synched odometry) to be added
        latestObservation = robot.robotObservationQueue_.front();
        robot.robotObservationQueue_.pop();
        break;
      }
      case 3: { // Relative Inter-Robot Measurement

        // Extract synced odometry from relative measurement
        RelativeMeas relativeMeas = robot.robotRelativeMeasQueue_.front();
        StampedSE3 validStampedPose;
        validStampedPose.pose = relativeMeas.odomPose;
        validStampedPose.stamp = relativeMeas.stamp;
        validStampedPose.covariance = relativeMeas.covariance;
        latestObservation.stampedPose = validStampedPose;

        // If we observed another robot (rather than being observed)
        if (!relativeMeas.onlyUseOdom) {
            sloam_->dbMutex.lock();

            // Pass the relative meas to database manager & factor generation thread
            robotData data = sloam_->dbManager.getHostRobotData();
            data.relativeMeasPacket.push_back(relativeMeas);
            sloam_->addRelativeMeasurement(relativeMeas);

            sloam_->dbMutex.unlock();
        }

        // Pop the relative measurement from the queue
        robot.robotRelativeMeasQueue_.pop_front();
        break;
      }
      default: {
        ROS_FATAL_STREAM("Invalid measurement # returned from PickNextOb: " << meas_to_add);
        ros::shutdown();
        break;
      }
    }

    // Use odometry (individual measurement or synced) to estimate motion since last key frame
    StampedSE3 raw_vio_odom_used_for_sloam = latestObservation.stampedPose;
    SE3 relativeRawOdomMotion = robot.robotLatestOdom_.pose.inverse() * raw_vio_odom_used_for_sloam.pose;

    // Estimate the covariance of the relative motion
    std::array<double, 6> relativeRawOdomCovariance = computeRelativeMotionCovariance(
        robot.robotLatestOdom_.pose, robot.robotLatestOdom_.covariance,
        raw_vio_odom_used_for_sloam.pose, raw_vio_odom_used_for_sloam.covariance);

    // Get the Previous Key Pose
    SE3 prevKeyPose;
    if (robot.robotKeyPoses_.size() > 0) {
      prevKeyPose = robot.robotKeyPoses_[robot.robotKeyPoses_.size() - 1];
    } else {
      ROS_WARN("No previous key pose. Use identity as the previous key pose.");
      prevKeyPose = SE3();
    }

    // Send the measurement to SLOAM
    SE3 keyPose;
    bool success = sloam_->runSLOAMNode(
        relativeRawOdomMotion, relativeRawOdomCovariance, prevKeyPose, 
        latestObservation.cylinders, latestObservation.cubes, 
        latestObservation.ellipsoids, latestObservation.stampedPose.stamp, 
        keyPose, hostRobotID_);
    if (success) {
      robot.robotKeyPoses_.push_back(keyPose);
      updateLastPose(raw_vio_odom_used_for_sloam, hostRobotID_);
    }

    // Check to see if we have another measurement to add
    PickNextMeasurementToAdd(robot.robotOdomQueue_, robot.robotObservationQueue_, robot.robotRelativeMeasQueue_, 
             robot.robotLatestOdom_, ros::Time::now().toSec(), robot.msg_delay_tolerance, minOdomDistance_, meas_to_add);
  }

  // If desired, save runtime & communication usage info
  if (sloam_->save_runtime_analysis) {
    saveRuntimeCommUsage();
  }
}

void InputManager::updateLastPose(const StampedSE3 &odom, const int &robotID) {
  robot.robotOdomReceived_ = true;
  robot.robotLatestOdom_.pose = odom.pose;
  robot.robotLatestOdom_.stamp = odom.stamp;
  robot.robotLatestOdom_.covariance = odom.covariance;
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

/*
 * @brief Save the runtime & communication usage info to a txt file
 */
void InputManager::saveRuntimeCommUsage() {
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
