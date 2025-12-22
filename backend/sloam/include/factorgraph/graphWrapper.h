/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#pragma once

#include <cubeMapManager.h>
#include <definitions.h>
#include <graph.h>
#include <cylinderMapManager.h>
#include <cube.h>
#include <cubeFactor.h>
#include <cylinder.h>
#include <cylinderFactor.h>
#include <ellipsoid.h>
#include <ellipsoidMapManager.h>

class SemanticFactorGraphWrapper : public SemanticFactorGraph {
 public:
  // constructor
  SemanticFactorGraphWrapper(const ros::NodeHandle &nh, int numRobots = 1);

  /** brief: addLoopClosureObservation (only for active SLAM, currently not used
   * in slideSLAM)
   * @param relativeMotion: relative motion between the two poses
   * @param poseEstimate: the pose estimate of the robot
   * @param cov: covariance of the relative motion
   * @param loop_closure_pose: the pose of the loop closure
   * @param closure_matched_pose_idx: the index of the matched pose
   * @return true: this is supposed to return true unless sanity check has
   * failed
   */
  bool addLoopClosureObservation(const SE3 &relativeMotion,
                                 const SE3 &poseEstimate,
                                 const boost::array<double, 36> &cov,
                                 const SE3 &loop_closure_pose,
                                 const size_t &closure_matched_pose_idx);

  bool getPoseByID(SE3 &curr_pose, const int &poseID);

  /** brief: addOdomBearingObservation (only for active SLAM, currently not used
   * in slideSLAM)
   * @param relativeMotion: relative motion between the two poses
   * @param poseEstimate: the pose estimate of the robot
   * @param cov: covariance of the relative motion
   * @param bearing_factors: bearing factors of the landmarks
   * @param range_factors: range factors of the landmarks
   * @param ids: ids of the landmarks
   * @param landmark_body_positions: body positions of the landmarks
   * @param data_association_distance: data association distance
   * @return true: this is supposed to return true unless sanity check has
   * failed
   */
  bool addOdomBearingObservation(
      const SE3 &relativeMotion, const SE3 &poseEstimate,
      const boost::array<double, 36> &cov,
      const std::vector<Point3> &bearing_factors,
      const std::vector<double> &range_factors, const std::vector<size_t> &ids,
      const std::vector<Point3> &landmark_body_positions,
      const double &data_association_distance);

  /**
   * @brief add betweenPose factors and Object Pose factors into the graph
   *
   * @param semanticMap semanticMap object instance
   * @param cube_semantic_map sube_semantic_map ibject instance
   * @param cyl_matches matches between detected objects and existing objects
   * @param cylinders detected objects
   * @param cube_matches matches between detected objects and existing objects
   * @param scan_cubes_world detected objects
   * @param pose the current KeyPose
   * @param robotID
   * @return true: this is supposed to return true unless sanity check has
   * failed
   * @return false
   */
  bool addSLOAMObservation(const CylinderMapManager &semanticMap,
                           const CubeMapManager &cubeSemanticMap,
                           const EllipsoidMapManager &ellipsoidSemanticMap,
                           const std::vector<int> &cyl_matches,
                           const std::vector<Cylinder> &cylinders,
                           const std::vector<int> &cube_matches,
                           const std::vector<Cube> &cubes,
                           const std::vector<int> &ellipsoid_matches,
                           const std::vector<Ellipsoid> &ellipsoids,
                           const SE3 &relativeMotion, 
                           const SE3 &poseEstimates,
                           const int &robotID, bool opt = true);

  /**
   * @brief updateFactorGraphMap
   *
   * @param semanticMap
   * @param cubeSemanticMap
   * @param ellipsoidSemanticMap
   */
  void updateFactorGraphMap(CylinderMapManager &semanticMap,
                            CubeMapManager &cubeSemanticMap,
                            EllipsoidMapManager &ellipsoidSemanticMap);
  void updateCylinder(const CylinderMeasurement &measurement, Cylinder &cyl);
  void updateCube(const CubeMeasurement &measurement, Cube &cube);
  void updateEllipsoid(const Point3 &measurement, Ellipsoid &ellipsoid);
  void getCurrPose(SE3 &curr_pose, const int &robotID,
                   boost::optional<Eigen::MatrixXd &> cov = boost::none);
  int numRobots;

  std::unordered_map<size_t, size_t> robotNumPoseInGraph;
  std::unordered_map<size_t, gtsam::Pose3> robotPrevPose;

  void getAllPoses(std::vector<SE3> &optimized_poses,
                   std::vector<size_t> &pose_inds, const int &robotID);
  void getAllCentroidLandmarks(std::vector<SE3> &optimized_landmark_pos,
                               std::vector<size_t> &landmark_inds);
  void getAllCentroidLandmarks(std::vector<SE3> &optimized_landmark_pos);
  void getAllCentroidLandmarksAndLabels(
      std::vector<SE3> &optimized_landmark_pos,
      std::vector<int> &landmark_labels);

  size_t pose_counter_robot1_;
  size_t pose_counter_robot2_;
  
  size_t getPoseCounterById(const int &robotID) const;
  std::vector<size_t> pose_counter_robot_;

 private:
  size_t cyl_counter_;
  size_t cube_counter_;
  size_t point_landmark_counter_;
  std::vector<gtsam::Pose3> robot_prev_pose_;

  // ROS NodeHandle for getting rosparams
  ros::NodeHandle nh_;
};
