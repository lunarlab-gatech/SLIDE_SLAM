/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/


#pragma once

#include <cube.h>
#include <cylinder.h>
#include <databaseManager.h>
#include <ellipsoid.h>
#include <definitions.h>
#include <plane.h>
#include <ros/ros.h>
#include <utils.h>

//  --------------- Feature and Model Structures ------------------

template <typename T>
struct ObjectMatch {
  ObjectMatch(PointT ft, T obj, Scalar dist) : object(obj) {
    Vector3d tfeat;
    tfeat << ft.x, ft.y, ft.z;

    feature = tfeat;
    dist = dist;
  }
  Vector3 feature;
  T object;
  double dist;
};

// -------------- odom/map inputs and outputs ------------------
struct SloamInput {
  SloamInput() {
  };
  SE3 poseEstimate;
  SE3 relativeRawOdomMotion;
  Scalar distance;
  std::vector<Cylinder> scanCylindersBody;
  std::vector<Cylinder> submapCylinders;
  std::vector<Cube> scanCubesBody;
  std::vector<Cube> submapCubes;
  std::vector<Ellipsoid> scanEllipsoidsBody;
  std::vector<Ellipsoid> submapEllipsoids;
};

struct SloamOutput {
  SloamOutput(){};
  std::vector<int> cubeMatches;
  std::vector<int> cylinderMatches;
  std::vector<int> ellipsoidMatches;
  std::vector<Cube> scanCubesWorld;
  std::vector<Cylinder> scanCylindersWorld;
  std::vector<Ellipsoid> scanEllipsoidsWorld;
  SE3 T_Map_Curr;
  SE3 T_Delta;
};

// Holds results from FindRelativeMeasurementMatch()
struct RelativeMeasMatch {
  RelativeMeasMatch(RelativeMeas m, int i, int h, int o) : 
        meas(m), index(i), indexClosestHostRobot(h), 
        indexClosestOtherRobot(o) {}
  RelativeMeas meas;
  int index;
  int indexClosestHostRobot;
  int indexClosestOtherRobot;
};

namespace sloam {
class sloam {
 public:
  explicit sloam();

  const FeatureModelParams &fmParams() const { return fmParams_; }
  void setFmParams(const FeatureModelParams &fmParams) { fmParams_ = fmParams; }
  bool RunSloam(SloamInput &in, const std::vector<Cube> &scan_cubes_body,
                std::vector<Cube> &scan_cubes_world,
                const std::vector<Cube> &submap_cubes,
                std::vector<int> &cube_match_indices, SloamOutput &out);
  
  bool RunSloam(SloamInput &in, SloamOutput &out);
  // Model estimation
  void projectModels(const SE3 &tf, std::vector<Cylinder> &cylinders, std::vector<Cube> &cubes,
                     std::vector<Ellipsoid> &ellipsoids);
  
  void computeModels(SloamInput &in, std::vector<Cylinder> &landmarks,
                     std::vector<Plane> &planes);
   // Data Association
  template <typename T>
  void matchModels(const std::vector<T> &currObjects,
                   const std::vector<T> &mapObjects,
                   std::vector<int> &matchIndices);
  template <typename T>
  void matchCubeModels(const std::vector<T> &currObjects,
                       const std::vector<T> &mapObjects,
                       std::vector<int> &matchIndices);

  template <typename T>
  void matchEllipsoidModels(const std::vector<T> &currObjects,
                       const std::vector<T> &mapObjects,
                       std::vector<int> &matchIndices);

  template <typename T>
  std::vector<ObjectMatch<T>> matchFeatures(const SE3 tf,
                                            const std::vector<T> &currObjects,
                                            const std::vector<T> &mapObjects,
                                            const Scalar distThresh);
  template <typename T>
  void addFeatureMatches(const VectorType &features, const T &object,
                         const double dist,
                         std::vector<ObjectMatch<T>> &matches);

  std::vector<Plane> getPrevGroundModel();
  CloudT getPrevGroundFeatures();

  // Relative Inter-Robot Factor Generation
  void FindRelativeMeasurementMatch(std::vector<size_t>& pose_counter_robot_,
                 const std::unordered_map<size_t, robotData>& robotDataDict_,
                                                             int hostRobotID,
                                    std::vector<RelativeMeasMatch>& matches);
  void GetIndexClosestPoseMstPair(std::deque<PoseMstPair> &poseMstPacket, 
            ros::Time stamp, int &indexClosest, double &timeDiffClosest);
  std::vector<RelativeMeas> feasible_relative_meas_for_factors; // All measurements that could still be used to generate a factor
  std::mutex feasRelMeasVectorMtx_;

 private:
  // Pose of ANCHOR frame in the Map frame
  SE3 T_Map_Anchor_;  
  // boost::shared_ptr<Plane> prevGPlane_;
  std::vector<Plane> prevGPlanes_;
  int numMapTrees_;
  double totalDistance_;
  FeatureModelParams fmParams_;
  bool firstScan_;
  double minPlanes_;
};

}  // namespace sloam
