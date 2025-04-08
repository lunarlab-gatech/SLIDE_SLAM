/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Jiuzhou Lei, Xu Liu, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <robot.h>

Robot::Robot(const ros::NodeHandle &nh) : nh_(nh) {

  // access ros parameter
  odomFreqFilter_ = nh_.param("odom_freq_filter", 10);
  nh_.param<std::string>("robot_frame_id", robot_frame_id_, "robot");
  nh_.param<float>("min_robot_altitude", minSLOAMAltitude_, 0.0);
  maxQueueSize_ = nh_.param("max_queue_size", 100);
  nh_.param<std::string>("robot_ns_prefix", robot_ns_prefix_, "robot");
  std::string node_name = ros::this_node::getName();
  std::string idName = node_name + "/hostRobotID";
  nh_.param<int>(idName, robotId_, 0);

  // Initialize variables
  robotOdomCounter_ = 0;

  // Initialize publishers
  std::string RobotHighFreqSLOAMPose_topic = robot_ns_prefix_ + std::to_string(robotId_) + "/pose_high_freq";
  pubRobotHighFreqSLOAMPose_ = nh_.advertise<geometry_msgs::PoseStamped>(RobotHighFreqSLOAMPose_topic, 10);

  std::string RobotHighFreqSLOAMOdom_topic = robot_ns_prefix_ + std::to_string(robotId_) + "/sloam_odom_high_freq";
  pubRobotHighFreqSLOAMOdom_ = nh_.advertise<nav_msgs::Odometry>(RobotHighFreqSLOAMOdom_topic, 20);

  std::string SloamToVioOdom_topic = robot_ns_prefix_ + std::to_string(robotId_) + "/sloam_to_vio_odom";
  pubSloamToVioOdom_ = nh_.advertise<nav_msgs::Odometry>(SloamToVioOdom_topic, 20);

  std::string RobotHighFreqSyncOdom_topic = robot_ns_prefix_ + std::to_string(robotId_) + "/sync_odom_high_freq";
  pubRobotHighFreqSyncOdom_ = nh_.advertise<nav_msgs::Odometry>(RobotHighFreqSyncOdom_topic, 20);

  // Initialize Subscribers
  RobotOdomSub_ = nh_.subscribe("odom", 10, &Robot::RobotOdomCb, this);
  
  std::string observationSub_topic = robot_ns_prefix_ + std::to_string(robotId_) + "/semantic_meas_sync_odom";
  RobotObservationSub_ = nh_.subscribe(observationSub_topic, 10, &Robot::RobotObservationCb, this);

  bool turn_off_rel_inter_robot_factor = nh_.param(node_name+"/turn_off_rel_inter_robot_factor", true);
  if (!turn_off_rel_inter_robot_factor) {
    std::string relativeMeasSub_topic = "/relative_inter_robot_meas_sync";
    RobotRelativeMeasSub_ = nh_.subscribe(relativeMeasSub_topic, 10, &Robot::RobotRelativeMeasCb, this);
  }

  ROS_INFO_STREAM("Robot Initialized! Robot # " << robotId_);
}

/*
 * @brief This method is called when a new odometry message is received.
 * It extracts the odometry and adds it to a queue. It filters out
 * messages at the rate of odomFreqFilter_, and skips a message if the
 * robot's z position is below minSLOAMAltitude_.
 * 
 * @param odom_msg: The odometry message received.
 */
void Robot::RobotOdomCb(const nav_msgs::OdometryConstPtr &odom_msg) {
  // Only take 1 out of every odomFreqFilter_ odometry messages
  if (odomFreqFilter_ > 1) {
    robotOdomCounter_++;
    if (robotOdomCounter_ % odomFreqFilter_ != 0)
      return;
    robotOdomCounter_ = 0;
  } 

  // Check to make sure that this message is not too old
  if ((ros::Time::now() - odom_msg->header.stamp).toSec() > msg_delay_tolerance) {
    PrintLateMsgWarning("Odometry");
    return;
  }

  // Extract info from the message
  auto pose = odom_msg->pose.pose;
  ros::Time odomStamp = odom_msg->header.stamp;
  Quat rot(pose.orientation.w, pose.orientation.x, pose.orientation.y,
           pose.orientation.z);
  Vector3 pos(pose.position.x, pose.position.y, pose.position.z);

  // Construct odom as SE3 object
  SE3 odom = SE3();
  odom.setQuaternion(rot);
  odom.translation() = pos;

  // Skip adding odometry if the robot is too low
  if (pose.position.z < minSLOAMAltitude_) {
    ROS_WARN_STREAM_THROTTLE(5, "Robot is too low, will not call sloam, height threshold is " << minSLOAMAltitude_);
    return;
  } 

  // Add odometry to the queue, and pop the oldest one if the queue is too long
  robotOdomQueue_.emplace_back(odom, odomStamp);
  if (robotOdomQueue_.size() > 10 * maxQueueSize_) robotOdomQueue_.pop_front();
}

void Robot::RobotObservationCb(const sloam_msgs::SemanticMeasSyncOdom &observation_msg) {
  // Check to make sure that this message is not too old
  if ((ros::Time::now() - observation_msg.header.stamp).toSec() > msg_delay_tolerance) {
    PrintLateMsgWarning("Observation");
    return;
  }

  // Save the observation
  Observation cur_observation;
  cur_observation.stampedPose.pose =
      databaseManager::toSE3Pose(observation_msg.odometry.pose.pose);
  cur_observation.stampedPose.stamp = observation_msg.header.stamp;

  std::vector<Cube> scan_cubes_body;
  int total_cuboids = observation_msg.cuboid_factors.size();
  for (const sloam_msgs::ROSCube &cur_cuboid_marker :
       observation_msg.cuboid_factors) {
    double x = cur_cuboid_marker.pose.position.x;
    double y = cur_cuboid_marker.pose.position.y;
    double z = cur_cuboid_marker.pose.position.z;
    gtsam::Rot3 rot(cur_cuboid_marker.pose.orientation.w,
                    cur_cuboid_marker.pose.orientation.x,
                    cur_cuboid_marker.pose.orientation.y,
                    cur_cuboid_marker.pose.orientation.z);
    gtsam::Point3 position(x, y, z);
    gtsam::Pose3 pose(rot, position);
    const gtsam::Point3 scale(cur_cuboid_marker.dim[0],
                              cur_cuboid_marker.dim[1],
                              cur_cuboid_marker.dim[2]);
    scan_cubes_body.push_back(
        Cube(pose, scale, cur_cuboid_marker.semantic_label));
  }

  cur_observation.cubes = scan_cubes_body;
  cur_observation.cylinders =
      rosCylinder2CylinderObj(observation_msg.cylinder_factors);
  cur_observation.ellipsoids =
      rosEllipsoid2EllipObj(observation_msg.ellipsoid_factors);
  robotObservationQueue_.push(cur_observation);
}

/**
 * @brief This method converts a sloam_msgs::RelativeInterRobotMeasurement 
 * into a RelativeMeas struct and adds it to the queue. 
 * 
 * @param relativeMeas_msg: The relative measurement message received.
 */
void Robot::RobotRelativeMeasCb(const sloam_msgs::RelativeInterRobotMeasurementOdom &relativeMeas_msg) {
  // Check to make sure that this message is not too old
  if ((ros::Time::now() - relativeMeas_msg.header.stamp).toSec() > msg_delay_tolerance) {
    PrintLateMsgWarning("Relative Measurement");
    return;
  }
  
  // If this relative measurement involves our robot in some way, add it
  if(relativeMeas_msg.robotIdObserver == robotId_ || relativeMeas_msg.robotIdObserved == robotId_) {
    RelativeMeas cur_measurement;
    cur_measurement.stamp = relativeMeas_msg.header.stamp;
    cur_measurement.relativePose = databaseManager::toSE3Pose(relativeMeas_msg.relativePose);

    // If we are observed, we only use the odometry to add a factor,
    // but don't consider it for a relative inter-robot factor
    if(relativeMeas_msg.robotIdObserver == robotId_) {
      cur_measurement.onlyUseOdom = false;
      cur_measurement.robotIndex = relativeMeas_msg.robotIdObserved;
      cur_measurement.odomPose = databaseManager::toSE3Pose(relativeMeas_msg.odometryObserver.pose.pose);
    } else {
      cur_measurement.onlyUseOdom = true;
      cur_measurement.robotIndex = relativeMeas_msg.robotIdObserver;
      cur_measurement.odomPose = databaseManager::toSE3Pose(relativeMeas_msg.odometryObserved.pose.pose);
    }

    robotRelativeMeasQueue_.push_back(cur_measurement);
  }
}

std::vector<Cylinder> Robot::rosCylinder2CylinderObj(
    const std::vector<sloam_msgs::ROSCylinder> &rosCylinders) {

  std::vector<Cylinder> cylinders;
  for (const auto &obj : rosCylinders) {
    double radius = obj.radius;
    gtsam::Point3 rayPoint3(obj.ray[0], obj.ray[1], obj.ray[2]);
    gtsam::Point3 rootPoint3(obj.root[0], obj.root[1], obj.root[2]);
    cylinders.emplace_back(rootPoint3, rayPoint3, radius, obj.semantic_label);
  }
  return cylinders;
}

std::vector<Ellipsoid> Robot::rosEllipsoid2EllipObj(
    const std::vector<sloam_msgs::ROSEllipsoid> &msgs) {
  std::vector<Ellipsoid> ellipsoids;

  for (const auto m : msgs) {
    double x = m.pose.position.x;
    double y = m.pose.position.y;
    double z = m.pose.position.z;
    gtsam::Rot3 rot(m.pose.orientation.w, m.pose.orientation.x,
                    m.pose.orientation.y, m.pose.orientation.z);
    gtsam::Point3 position(x, y, z);
    gtsam::Pose3 pose(rot, position);
    int label = m.semantic_label;
    // scale of the ellipsoid
    const gtsam::Point3 scale(m.scale[0], m.scale[1], m.scale[2]);
    // assemble a pose from the
    ellipsoids.emplace_back(pose, scale, label);
  }

  return ellipsoids;
}

void Robot::PrintLateMsgWarning(const std::string &msg_type) {
  ROS_WARN_STREAM_THROTTLE(10, msg_type << "message arrived after " << msg_delay_tolerance << " seconds, dicarding...");
  ROS_WARN_STREAM_THROTTLE(10, "Please increase `msg_delay_tolerance` to account for lag in your system");
}

/**
 * @brief This method extracts the covariance values from a
 * nav_msgs::Odometry message.
 * 
 * @param odom_msg: The odometry message from which to extract the covariance.
 * @return A std::array containing the covariance diagonal values (in XYZRPY order)
 */
// std::array<double, 6> Robot::ExtractCovarianceFromOdometry(nav_msgs::Odometry odom_msg) {
//   std::array<double, 6> covariance;
//   for (size_t i = 0; i < 6; i++) {
//     // Absolute value used in case some very small covariance values come in negative
//     covariance[i] = std::abs(odom_msg.pose.covariance[i * 6 + i]);
//   }
//   return covariance;
// }