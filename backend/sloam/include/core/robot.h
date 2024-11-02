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
#include <sloam_msgs/ROSCylinder.h>
#include <databaseManager.h>

#include <deque>
#include <queue>
#include <vector>

struct StampedSE3 {
  StampedSE3(SE3 p, ros::Time s) : pose(p), stamp(s){};
  StampedSE3() : pose(SE3()), stamp(ros::Time::now()){};
  SE3 pose;
  ros::Time stamp;
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
  // how many seconds max to wait for semantic measurements to arrive
  double semantic_meas_delay_tolerance_ = 3.0;
  std::deque<StampedSE3> robotOdomQueue_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> robotTreePcQueue_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> robotGroundPcQueue_;
  std::queue<std::pair<std::vector<Cube>, ros::Time>> robotCubesQueue_;
  std::queue<Observation> robotObservationQueue_;

  // Publishers
  ros::Publisher pubRobotHighFreqSLOAMPose_;
  ros::Publisher pubRobotHighFreqSLOAMOdom_;
  ros::Publisher pubSloamToVioOdom_;
  ros::Publisher pubRobotHighFreqSyncOdom_;
  //Subscribers
  ros::Subscriber RobotOdomSub_;
  ros::Subscriber RobotGroundPCSub_;
  ros::Subscriber RobotTreePCSub_;
  ros::Subscriber RobotCubeSub_;
  ros::Subscriber RobotObservationSub_;

  std::string robot_odom_topic_;
  bool robotTreeCloudUpdated_ = false;
  bool robotGroundCloudUpdated_ = false;
  bool robotCubesUpdated_ = false;
  bool robotOdomReceived_ = false;
  bool robotOdomUpdated_ = false;
  bool robotObservationUpdated_ = false;
  // optimized key poses
  std::vector<SE3> robotKeyPoses_;
  StampedSE3 robotLatestOdom_;
  SE3 robotLastSLOAMKeyPose_ = SE3();
  bool robotFirstOdom_;
  size_t robotOdomCounter_;
  int robotId_;
  std::string robot_ns_prefix_;
  std::string odom_topic_;

  void RobotOdomCb(const nav_msgs::OdometryConstPtr &odom_msg);
  void RobotTreePCCb(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
  void RobotGroundPCCb(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
  void RobotCubeCb(const visualization_msgs::MarkerArray &cuboid_msg);
  void RobotObservationCb(const sloam_msgs::SemanticMeasSyncOdom &observation_msg);
  std::vector<Ellipsoid> rosEllipsoid2EllipObj(const std::vector<sloam_msgs::ROSEllipsoid>& msgs);
  std::vector<Cylinder> rosCylinder2CylinderObj(const std::vector<sloam_msgs::ROSCylinder>& rosCylinders);
  Robot(const ros::NodeHandle &nh);
};
