/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Jiuzhou Lei, Xu Liu, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#pragma once

#include <cube.h>
#include <cylinder.h> 
#include <ellipsoid.h> 
#include <gtsam/geometry/Point3.h>
#include <databaseManager.h>
#include <definitions.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sloam_msgs/SemanticMeasSyncOdom.h>
#include <sloam_msgs/RelativeInterRobotMeasurement.h>
#include <sloam_msgs/RelativeInterRobotMeasurementOdom.h>
#include <sloam_msgs/ROSCylinder.h>

#include <deque>
#include <queue>
#include <vector>

struct StampedSE3 {
  StampedSE3(SE3 p, ros::Time s, std::array<double, 6> c) : pose(p), stamp(s), covariance(c) {};
  StampedSE3(SE3 p, ros::Time s) : pose(p), stamp(s){};
  StampedSE3() : pose(SE3()), stamp(ros::Time::now()){};
  SE3 pose;
  ros::Time stamp;
  std::array<double, 6> covariance; // Ordering XYZRPY
};

struct Observation {
  StampedSE3 stampedPose;
  std::vector<Cube> cubes;
  std::vector<Cylinder> cylinders;
  std::vector<Ellipsoid> ellipsoids;
};

class Robot {
 private:
  size_t odomFreqFilter_;
  std::string robot_frame_id_;
  float minSLOAMAltitude_;
  size_t maxQueueSize_;
  ros::NodeHandle nh_;

 public: 
  StampedSE3 robotLatestOdom_;
  SE3 robotLastSLOAMKeyPose_ = SE3();
  size_t robotOdomCounter_;
  bool turn_off_rel_inter_robot_loop_closure;
  
  int robotId_; // ID (index) of this robot
  std::string robot_ns_prefix_; // string before robotId_ in ros topics

  // Initializer
  Robot(const ros::NodeHandle &nh);

  // Data structures for holding recieved data from this robot
  std::deque<StampedSE3> robotOdomQueue_;
  std::queue<Observation> robotObservationQueue_;
  std::deque<RelativeMeas> robotRelativeMeasQueue_;

  // Used by inputNode.cpp to track if sloam has already updated a previous odometry
  bool robotOdomReceived_ = false;

  // Time to wait for all incoming messages to be recieved before processing
  double msg_delay_tolerance = 3.0; // seconds

  // Publishers
  ros::Publisher pubRobotHighFreqSLOAMPose_;
  ros::Publisher pubRobotHighFreqSLOAMOdom_;
  ros::Publisher pubSloamToVioOdom_;
  ros::Publisher pubRobotHighFreqSyncOdom_;

  // Subscribers
  ros::Subscriber RobotOdomSub_;
  ros::Subscriber RobotObservationSub_;
  ros::Subscriber RobotRelativeMeasSub_;

  // Optimized key poses
  std::vector<SE3> robotKeyPoses_;

  // Callback functions for Subscribers
  void RobotOdomCb(const nav_msgs::OdometryConstPtr &odom_msg);
  void RobotObservationCb(const sloam_msgs::SemanticMeasSyncOdom &observation_msg);
  void RobotRelativeMeasCb(const sloam_msgs::RelativeInterRobotMeasurementOdom &relativeMeas_msg);

  // Helper methods for converting messages to objects
  std::vector<Ellipsoid> rosEllipsoid2EllipObj(const std::vector<sloam_msgs::ROSEllipsoid>& msgs);
  std::vector<Cylinder> rosCylinder2CylinderObj(const std::vector<sloam_msgs::ROSCylinder>& rosCylinders);
  std::array<double, 6> ExtractCovarianceFromOdometry(nav_msgs::Odometry odom_msg);

  // Logging methods
  void PrintLateMsgWarning(const std::string &msg_type);
};
