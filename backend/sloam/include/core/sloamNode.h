/**
 * This file is part of SlideSLAM
 *
 * Copyright (C) 2024 Guilherme Nardari, Xu Liu, Jiuzhou Lei, Ankit Prabhu,
 * Yuezhan Tao
 *
 * TODO: License information
 *
 */

#pragma once

// ROS
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// pcl
#include <cube.h>
#include <cubeMapManager.h>
#include <cylinderMapManager.h>
#include <databaseManager.h>
#include <definitions.h>
#include <ellipsoid.h>
#include <ellipsoidMapManager.h>
#include <graphWrapper.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <place_recognition.h>
#include <sloam.h>
#include <sloam_msgs/ROSObservation.h>
#include <tf2/buffer_core.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <utils.h>
#include <vizTools.h>

#include <mutex>
#include <queue>
#include <random>
#include <thread>

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

namespace sloam {
class SLOAMNode : public sloam {
 public:
  explicit SLOAMNode(const ros::NodeHandle &nh);
  ~SLOAMNode();
  void destaggerCloud(const Cloud::Ptr cloud, Cloud::Ptr &outCloud);
  SLOAMNode(const SLOAMNode &) = delete;
  SLOAMNode operator=(const SLOAMNode &) = delete;
  using Ptr = boost::shared_ptr<SLOAMNode>;
  using ConstPtr = boost::shared_ptr<const SLOAMNode>;

  // timestamp is used for visualization
  bool runSLOAMNode(const SE3 &relativeRawOdomMotion, 
                    std::array<double, 6> relativeRawOdomMotionCov,
                    const SE3 &prevKeyPose,
                    const std::vector<Cylinder> &cylindersBody,
                    const std::vector<Cube> &cubesBody,
                    const std::vector<Ellipsoid> &ellipsoidBody,
                    ros::Time stamp, SE3 &outPose, const int &robotID);
  void addRelativeMeasurement(RelativeMeas relativeMeas);
  bool isInLoopClosureRegion_ = false;
  SemanticFactorGraphWrapper factorGraph_;
  databaseManager dbManager;
  std::mutex dbMutex;
  int hostRobotID;
  CylinderMapManager semanticMap_;

  // results analysis related variables
  std::vector<double> fg_optimization_time;
  int total_number_of_landmarks = 0;
  double trajectory_length = 0;
  std::vector<double> data_association_time;
  std::vector<double> intra_loop_closure_time;
  int num_attempts_intra_loop_closure = 0;
  int num_successful_intra_loop_closure = 0;
  std::vector<double> inter_loop_closure_time;
  int num_attempts_inter_loop_closure = 0;
  int num_successful_inter_loop_closure = 0;
  int num_successful_rel_inter_robot_factor = 0;
  bool save_runtime_analysis = false;
  std::string runtime_analysis_file;

 private:
  // TODO(xu): load the following four params from rosparam
  bool save_inter_robot_closure_results_ = true;
  string save_results_dir_;
  bool save_robot_trajectory_as_csv_ = true;
  string save_runtime_analysis_dir_;


  double inter_robot_place_recognition_frequency_;
  double intra_robot_place_recognition_frequency_;
  double rel_inter_robot_factor_frequency_;

  std::vector<ros::Time> KeyPoseTimeStamps;
  ros::Publisher groundPub_;

  void initParams_();
  Cloud::Ptr trellisCloud(
      const std::vector<std::vector<TreeVertex>> &landmarks);
  void publishMap_(const ros::Time stamp);
  void publishCubeMaps_(const ros::Time stamp);
  void publishInterRobotFactors(const int &robotID);

  bool prepareInputs_(const SE3 relativeMotion, const SE3 prevKeyPose,
                      CloudT::Ptr tree_cloud, CloudT::Ptr ground_cloud,
                      SloamInput &sloamIn);
  void publishResults_(const SloamInput &sloamIn, const SloamOutput &sloamOut,
                       ros::Time stamp, const int &robotID);
  void intraLoopClosureThread_();
  void interLoopClosureThread_();
  void relInterRobotFactorThread_();
  void GetIndexClosestPoseMstPair(std::deque<PoseMstPair> &poseMstPacket, ros::Time stamp, int &indexClosest, double &timeDiffClosest);

  std::vector<Eigen::Vector3d> extractPosition(
      const std::vector<Cylinder> &candidateCylinderObs,
      const std::vector<Cube> &candidateCubeObs,
      const std::vector<SE3> &candidateCentroidObs);

  std::vector<Eigen::Vector3d> extractPosition(
      const std::vector<gtsam_cylinder::CylinderMeasurement>
          &candidateCylinderObs,
      const std::vector<gtsam_cube::CubeMeasurement> &candidateCubeObs,
      const std::vector<gtsam::Point3> &candidateCentroidObs);

  std::vector<Eigen::Vector7d> prepareLCInput(
      const std::vector<Cylinder> &candidateCylinderObs,
      const std::vector<Cube> &candidateCubeObs,
      const std::vector<Ellipsoid> &candidateCentroidObs);

  ros::NodeHandle nh_;
  
  bool use_slidematch_; // whether to use the slidematch algorithm, if false, use
                        // the slidegraph place recognition algorithm

  tf::TransformBroadcaster worldTfBr_;
  ros::Publisher pubMapPose_;
  ros::Publisher pubObs_;
  ros::Publisher pubAllPointLandmarks_;

  std::vector<ros::Publisher> pubRobotTrajectory_;
  ros::Publisher pubMapGroundFeatures_;
  ros::Publisher pubObsTreeFeatures_;
  ros::Publisher pubObsGroundFeatures_;
  ros::Publisher pubMapTreeModel_;
  ros::Publisher pubSubmapTreeModel_;
  ros::Publisher pubObsTreeModel_;
  ros::Publisher pubMapGroundModel_;
  ros::Publisher pubObsGroundModel_;
  ros::Publisher pubMapCubeModel_;
  ros::Publisher pubSubmapCubeModel_;

  ros::Publisher pubRelInterRobotFactors_;

  // Transform
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_id_;

  // Instance graphDetector_;
  std::vector<Cylinder> submap_cylinders_;
  std::vector<int> cylinder_matches_;
  FeatureModelParams fmParams_;

  // loop closure related
  PlaceRecognition intra_loopCloser_;
  ros::Time last_intra_loop_closure_stamp_;
  ros::Time last_inter_loop_closure_stamp_;
  PlaceRecognition inter_loopCloser_;
  std::thread intraLoopthread_;
  std::thread interLoopthread_;
  int lastLoopAttemptPose_;
  std::mutex semanticMapMtx_;
  std::mutex cubeSemanticMapMtx_;
  std::mutex ellipsoidSemanticMapMtx_;
  std::mutex factorGraphMtx_;

  // Relative Measurement Factor Generation
  ros::Time last_rel_inter_robot_factor_stamp_;
  std::thread relInterRobotFactorthread_;
  std::vector<RelativeMeas> feasible_relative_meas_for_factors; // All measurements that could still be used to generate a factor
  std::vector<RelativeInterRobotFactor> relative_inter_robot_factors; // All measurements that have been used to generate a factor
  std::mutex feasRelMeasVectorMtx_;
  std::mutex relInterRobotFactorsMtx_;

  // For cuboid semantic landmarks
  CubeMapManager cube_semantic_map_;
  std::vector<int> cube_matches_;
  std::vector<Cube> submap_cubes_;
  std::vector<Cube> scan_cubes_world_;

  // For ellipsoid semantic landmarks
  EllipsoidMapManager ellipsoid_semantic_map_;
  std::vector<int> ellipsoid_matches_;
  std::vector<Ellipsoid> submap_ellipsoids_;
  std::vector<Ellipsoid> scan_ellipsoids_world_;

  // flags
  bool cube_map_initialized_ = false;
  int counter_for_noise = 0;
  bool firstScan_;
  bool debugMode_;

  int numRobots;

  void getCentroidSubmap(const std::vector<SE3> &allCentroids,
                         std::vector<SE3> &submapCentroids, const SE3 &pose,
                         const double &radius);
};
}  // namespace sloam
