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
  robotFirstOdom_ = true;
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

  bool turn_off_rel_inter_robot_loop_closure = nh_.param(node_name+"/turn_off_rel_inter_robot_loop_closure", true);
  if (!turn_off_rel_inter_robot_loop_closure) {
    std::string relativeMeasSub_topic = robot_ns_prefix_ + std::to_string(robotId_) + "/relative_inter_robot_meas";
    RobotRelativeMeasSub_ = nh_.subscribe(relativeMeasSub_topic, 10, &Robot::RobotRelativeMeasCb, this);
  }


  ROS_INFO_STREAM("Robot Initialized! Robot # " << robotId_);
}

void Robot::RobotOdomCb(const nav_msgs::OdometryConstPtr &odom_msg) {
  if (odomFreqFilter_ > 1) {
    robotOdomCounter_++;
    if (robotOdomCounter_ % odomFreqFilter_ != 0)
      return;
    robotOdomCounter_ = 0;
  } 

  auto pose = odom_msg->pose.pose;
  ros::Time odomStamp = odom_msg->header.stamp;
  Quat rot(pose.orientation.w, pose.orientation.x, pose.orientation.y,
           pose.orientation.z);
  Vector3 pos(pose.position.x, pose.position.y, pose.position.z);

  SE3 odom = SE3();
  odom.setQuaternion(rot);
  odom.translation() = pos;


  if (robotFirstOdom_ && pose.position.z < minSLOAMAltitude_) {
    ROS_INFO_STREAM_THROTTLE(
        5, "Robot is too low, will not call sloam, height threshold is "
               << minSLOAMAltitude_);
    return;
  } 

  // if either robotObservationQueue_.empty() or robotOdomQueue_.empty(), we need to add odom factor
  bool is_first_run = false;
  if (robotObservationQueue_.empty() || robotOdomQueue_.empty()) {
    ROS_INFO_STREAM_THROTTLE(1.0, "Either robotObservationQueue_ or robotOdomQueue_ is empty, will add odom factor");
    is_first_run = true;
  }

  robotOdomQueue_.emplace_back(odom, odomStamp);
  if (robotOdomQueue_.size() > 10 * maxQueueSize_)
    robotOdomQueue_.pop_front();

  if(is_first_run){
    robotOdomUpdated_ = true;
    return;
  } else {
    // we set robotOdomUpdated_ to true only if the latest semantic measurements can be discard (i.e. its timestamp is at least semantic_meas_delay_tolerance_ seconds earlier than the latest odometry stamp)
    auto latest_observation_stamp = robotObservationQueue_.back().stampedPose.stamp;
    if ((odomStamp - latest_observation_stamp).toSec() > semantic_meas_delay_tolerance_) {
      ROS_INFO_STREAM_THROTTLE(5.0, "Odometry delay is enabled, and semantic observation is old enough, setting robotOdomUpdated_ to true");
      robotOdomUpdated_ = true;
    } else {
      robotOdomUpdated_ = false;
    }
  }
}

void Robot::RobotObservationCb(
    const sloam_msgs::SemanticMeasSyncOdom &observation_msg) {
  // ROS_INFO_STREAM("RobotObservationCb!");
  // compared the stamp of the observation with the latest odometry stamp, if too old, discard this observation msg
  if (robotOdomQueue_.empty()) {
    ROS_WARN_STREAM("RobotOdomQueue is empty, cannot compare stamp");
    return;
  }
  auto latest_odom_stamp = robotOdomQueue_.back().stamp;
  if ((latest_odom_stamp - observation_msg.header.stamp).toSec() > semantic_meas_delay_tolerance_) {
    ROS_WARN_STREAM("Semantic observation arrived too late, discard this observation");
    ROS_WARN_STREAM("Semantic observation arrived " << (latest_odom_stamp - observation_msg.header.stamp).toSec() << " seconds late, tolerance is " << semantic_meas_delay_tolerance_);
    return;
  }
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
  robotObservationUpdated_ = true;
}

/**
 * @brief This method converts a 
 * sloam_msgs::RelativeInterRobotMeasurement into
 * a RelativeMeas struct and adds it to the queue.
 * inputNode will pass this onto the databaseManager
 * in sloamNode.
 * 
 * @param sloam_msgs::RelativeInterRobotMeasurement 
 *    &relativeMeas_msg
 */
void Robot::RobotRelativeMeasCb(const sloam_msgs::RelativeInterRobotMeasurement &relativeMeas_msg) {
  RelativeMeas cur_measurement;
  cur_measurement.stamp = relativeMeas_msg.header.stamp;
  cur_measurement.odomPose = databaseManager::toSE3Pose(relativeMeas_msg.odometry.pose.pose);
  cur_measurement.relativePose = databaseManager::toSE3Pose(relativeMeas_msg.relativePose);
  cur_measurement.robotIndex = relativeMeas_msg.robotId;
  robotRelativeMeasQueue_.push_back(cur_measurement);
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