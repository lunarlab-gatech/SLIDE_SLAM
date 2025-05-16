/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <sloam.h>

namespace sloam {

sloam::sloam() {
  // Initialize the map parameters
  T_Map_Anchor_ = SE3();
  firstScan_ = true;
  minPlanes_ = (fmParams_.groundRadiiBins * fmParams_.groundThetaBins) * 0.1;
}

std::vector<Plane> sloam::getPrevGroundModel() { return prevGPlanes_; }

CloudT sloam::getPrevGroundFeatures() {
  CloudT pc;
  for (const auto &ground : prevGPlanes_) {
    for (const auto &pt : ground.features)
      pc.points.push_back(pt);
  }

  pc.height = 1;
  pc.width = pc.points.size();
  return pc;
}

template <typename T>
std::vector<ObjectMatch<T>>
sloam::matchFeatures(const SE3 tf, const std::vector<T> &currObjects,
                     const std::vector<T> &mapObjects,
                     const Scalar distThresh) {
  std::vector<ObjectMatch<T>> matches;
  for (const auto &co : currObjects) {
    T proj_obj = co;
    proj_obj.project(tf);
    // find closest model in map
    Scalar bestDist = distThresh + 100;
    T bestObject = mapObjects[0];
    for (const auto &mo : mapObjects) {
      Scalar d = mo.distance(proj_obj.getModel());
      if (d < bestDist) {
        bestDist = d;
        bestObject = mo;
      }
    }
    // create feature matches according to model association
    if (bestDist < distThresh) {
      addFeatureMatches(co.features, bestObject, bestDist, matches);
    }
  }

  return matches;
}

template <typename T>
void sloam::addFeatureMatches(const VectorType &features, const T &object,
                              const double dist,
                              std::vector<ObjectMatch<T>> &matches) {
  for (const auto &feature : features) {
    ObjectMatch<T> match(feature, object, dist);
    matches.push_back(match);
  }
}

template <typename T>
void sloam::matchModels(const std::vector<T> &currObjects,
                        const std::vector<T> &mapObjects,
                        std::vector<int> &matchIndices) {
  size_t obj_counter = 0;
  if (currObjects.size() == 0) {
    ROS_WARN_THROTTLE(30, "No cylinder detected in current scan!");
    if (mapObjects.size() == 0) {
      ROS_WARN_THROTTLE(30, "No cylinder in submap!");
    }
    return;
  } else if (mapObjects.size() == 0) {
    ROS_WARN_THROTTLE(30, "No cylinder in submap!");
    return;
  }
  for (const auto &co : currObjects) {
    // find closest model in map
    Scalar bestDist = fmParams_.cylinderMatchThresh + 100;
    T bestObject = mapObjects[0];
    size_t bestKey = 0;
    size_t key = 0;
    for (const auto &mo : mapObjects) {
      Scalar d = mo.distance(co.getModel());
      if (d < bestDist) {
        bestDist = d;
        bestObject = mo;
        bestKey = key;
      }
      ++key;
    }
    // create feature matches according to model association
    if (bestDist < fmParams_.cylinderMatchThresh) {
      matchIndices[obj_counter] = bestKey;;
    }
    obj_counter++;
  }
  // std::cout << "total scan objects in Match Cylinder Models:" << obj_counter
  //           << '\n';
}

template <typename T>
void sloam::matchCubeModels(const std::vector<T> &currObjects,
                            const std::vector<T> &mapObjects,
                            std::vector<int> &matchIndices) {
  size_t obj_counter = 0;

  if (currObjects.size() == 0) {
    ROS_WARN_THROTTLE(20, "No cube detected in current scan!");
    return;
  } else if (mapObjects.size() == 0) {
    ROS_WARN_THROTTLE(20, "No cube in submap!");
    return;
  }

  // TODO(ankit): Make cube_match_search_threshold and valid_match_treshold a parameter
  Scalar cube_match_search_threshold = 30;
  // distance within which cuboid pair is regarded as a valid match
  // (cube match threshold) cube threshold for matching
  // cuboid matching threshold for data association, cube match threshold, data association threshold
  Scalar valid_match_treshold = fmParams_.cuboidMatchThresh;  // 2;
  // cuboid data association distance threshold
  for (const auto &co : currObjects) {
    // find closest model in map
    Scalar bestDist = cube_match_search_threshold;
    T bestObject = mapObjects[0];
    size_t bestKey = 0;
    size_t key = 0;
    for (const auto &mo : mapObjects) {
      Scalar d = mo.distance(co.getModel());
      if (d < bestDist) {
        bestDist = d;
        bestObject = mo;
        bestKey = key;
      }
      ++key;
    }
    // create feature matches according to model association
    if (bestDist < valid_match_treshold) {
      matchIndices[obj_counter] = bestKey;
    }
    obj_counter++;
  }
  //std::cout << "total scan objects in matchCubeModels:" << obj_counter << '\n';
}

template <typename T>
void sloam::matchEllipsoidModels(const std::vector<T> &currObjects,
                                 const std::vector<T> &mapObjects,
                                 std::vector<int> &matchIndices) {

  if (currObjects.size() == 0) {
    ROS_WARN_STREAM_THROTTLE(20.0, "No Ellipsoid detected in current scan!");
    return;
  } else if (mapObjects.size() == 0) {
    ROS_WARN_THROTTLE(20.0, "No Ellipsoid in submap!");
    return;
  }
  // TODO(ankit): Make ellip_match_search_threshold and valid_match_treshold a parameter
  
  // distance within which ellipsoid pair is regarded as a valid match
  // ellipsoid matching threshold for data association, ellipsoid match threshold, data association threshold
  Scalar valid_match_treshold = fmParams_.ellipsoidMatchThresh;  // 0.75;
  // ellipsoid data association distance threshold
  Scalar ellip_match_search_threshold = 1000;
  size_t obj_counter = 0;
  for (const auto &co : currObjects) {
    // find closest model in map
    Scalar bestDist = ellip_match_search_threshold;
    T bestObject = mapObjects[0];
    size_t bestKey = 0;
    size_t key = 0;
    for (const auto &mo : mapObjects) {
      // multi-class data association
      if (mo.model.semantic_label == co.model.semantic_label) {
        Scalar d = mo.distance(co.getModel());
        if (d < bestDist) {
          bestDist = d;
          bestObject = mo;
          bestKey = key;
        }
      }
      ++key;
    }

    // create feature matches according to model association
    if (bestDist < valid_match_treshold) {
      matchIndices[obj_counter] = bestKey;
    } 
       obj_counter++;
  }
}

void sloam::projectModels(const SE3 &tf, std::vector<Cylinder> &cylinders,
                          std::vector<Cube> &cubes,
                          std::vector<Ellipsoid> &ellipsoids) {
  for (auto &cyl : cylinders) {
    cyl.project(tf);
  }
  for (auto &cube : cubes) {
    cube.project(tf);
  }
  for (auto &ellipsoid : ellipsoids) {
    ellipsoid.project(tf);
  }
}

// called in sloamNode.cpp
bool sloam::RunSloam(SloamInput &in, SloamOutput &out) {
  // input landmarks are in local frame, output landmarks are in the world frame

  // Initialize all matches to be -1 (-1 means no match is found for the landmark)
  std::vector<int> cylMatchIndices(in.scanCylindersBody.size(), -1);
  std::vector<int> cubeMatchIndices(in.scanCubesBody.size(), -1);
  std::vector<int> ellipsoidMatchIndices(in.scanEllipsoidsBody.size(), -1);
  // the following two landmarks will be used to store projected landmarks
  // this vector will be empty if there is no cylinder/cube detected in current
  // scan
  std::vector<Cylinder> cyl_landmarks_world = in.scanCylindersBody;
  std::vector<Cube> cube_landmarks_world = in.scanCubesBody;
  std::vector<Ellipsoid> ellipsoid_landmarks_world = in.scanEllipsoidsBody;

  bool success = true;
  if (firstScan_) {
    projectModels(in.poseEstimate, cyl_landmarks_world, cube_landmarks_world,
                  ellipsoid_landmarks_world);
    // First run, pose will be same as odom
    out.T_Map_Curr = in.poseEstimate;
    // Matches will be all -1
    out.cylinderMatches = cylMatchIndices;
    out.scanCylindersWorld = cyl_landmarks_world;
    out.cubeMatches = cubeMatchIndices;
    out.scanCubesWorld = cube_landmarks_world;
    out.ellipsoidMatches = ellipsoidMatchIndices;
    out.scanEllipsoidsWorld = ellipsoid_landmarks_world;
    firstScan_ = false;
    return success;
  } else {
    bool no_cylinder = false;
    bool no_cube = false;
    bool no_ellipsoid = false;
    if (cyl_landmarks_world.size() == 0) {
      // ROS_WARN("No cylinder models found");
      no_cylinder = true;
    }
    if (cube_landmarks_world.size() == 0) {
      // ROS_WARN("No cube models found");
      no_cube = true;
    }
    if (ellipsoid_landmarks_world.size() == 0) {
      // ROS_WARN("No ellipsoid models found");
      no_ellipsoid = true;
    }

    SE3 T_Delta = SE3();
    SE3 currPose = in.poseEstimate;

    projectModels(currPose, cyl_landmarks_world, cube_landmarks_world,
                  ellipsoid_landmarks_world);

    if (in.submapCylinders.size() == 0 || no_cylinder) {
      // ROS_WARN(
      //     "Cylinder Submap is empty! OR no cylinder detected in current scan!");
    } else {
      matchModels(cyl_landmarks_world, in.submapCylinders, cylMatchIndices);
    }

    if (in.submapCubes.size() == 0 || no_cube) {
      // ROS_WARN("Cube Submap is empty! OR no cube detected in current scan!");
    } else {
      matchCubeModels(cube_landmarks_world, in.submapCubes, cubeMatchIndices);
      // std::cout << "total submap cubes:" << in.submapCubes.size() << '\n';
    }

    if (in.submapEllipsoids.size() == 0 || no_ellipsoid) {
      // ROS_WARN("Ellipsoid Submap is empty! OR no ellipsoid detected in current "
      //          "scan!");
    } else {
      matchEllipsoidModels(ellipsoid_landmarks_world, in.submapEllipsoids,
                           ellipsoidMatchIndices);
      // std::cout << "total submap ellipsoids:" << in.submapEllipsoids.size()
      //           << '\n';
    }

    out.cylinderMatches = cylMatchIndices;
    out.cubeMatches = cubeMatchIndices;
    out.ellipsoidMatches = ellipsoidMatchIndices;
    out.T_Map_Curr = currPose;
    out.T_Delta = T_Delta;
    out.scanCylindersWorld = cyl_landmarks_world;
    out.scanCubesWorld = cube_landmarks_world;
    out.scanEllipsoidsWorld = ellipsoid_landmarks_world;
    return success;
  }
} // end of RunSloam

/**
 * @brief Iterates through each relative measurement and checks
 * if a corresponding pair of robot poses (within reasonable
 * time) are found within the graph. Return all the matches.
 * 
 * @param pose_counter_robot_: The pose counters for each robot,
 *   which tracks the number of pose variables in the factor graph.
 * @param robotDataDict_: The robot data dictionary, which contains
 *   all recieved robot data for each robot.
 * @param hostRobotID: The ID of the current robot.
 * @param[out] matches: The vector of relative measurements that have
 *   matches onto poses for both robots in the factor graph.
 */
void sloam::FindRelativeMeasurementMatch(std::vector<size_t>& pose_counter_robot_,
                      const std::unordered_map<size_t, robotData>& robotDataDict_,
                                                                  int hostRobotID,
                                         std::vector<RelativeMeasMatch>& matches) {
  // Lock mutexes
  feasRelMeasVectorMtx_.lock();

  // Maximum time difference between relative inter robot measurement and stamped pose
  // in order to add measurement to the factor graph
  double maxTimeDiff = 0.001; // 1 ms

  // Get the data of the host robot
  std::deque<PoseMstPair> poseMstPacketHost = robotDataDict_.at(hostRobotID).poseMstPacket;

  // Only run loop closures if we have relative inter-robot measurements waiting
  if (feasible_relative_meas_for_factors.size() > 0) {
    // Prepare return variables
    int indexClosestHostRobot;
    int indexClosestOtherRobot;
    double timeDiffClosest;

    // Try to add each relative measurement
    for(int i = 0; i < feasible_relative_meas_for_factors.size(); i++) {
      RelativeMeas relativeMeas = feasible_relative_meas_for_factors[i];

      // Make sure that the robot ID is not the same as the host robot
      if (relativeMeas.robotIndex == hostRobotID) { 
        feasRelMeasVectorMtx_.unlock();
        throw std::runtime_error("robotIndex should not be the same as hostRobotID");
      }

      // Make sure that onlyUseOdom is not true
      if (relativeMeas.onlyUseOdom) { 
        feasRelMeasVectorMtx_.unlock();
        throw std::runtime_error("onlyUseOdom measurements shouldn't get to this function");
      }

      // Iterate through the other robot's poses, and make sure one exists in the factor graph
      // that is close enough in time to our measurement
      size_t pose_counter_other = pose_counter_robot_[relativeMeas.robotIndex];
      if (robotDataDict_.find(relativeMeas.robotIndex) == robotDataDict_.end()) continue;
      std::deque<PoseMstPair> poseMstPacketOther = robotDataDict_.at(relativeMeas.robotIndex).poseMstPacket;
      GetIndexClosestPoseMstPair(poseMstPacketOther, relativeMeas.stamp, indexClosestOtherRobot, timeDiffClosest);
      if (indexClosestOtherRobot == -1 || timeDiffClosest > maxTimeDiff ||
          indexClosestOtherRobot >= pose_counter_other) { 
            continue; 
      }

      // Iterate through our own poses, and make sure one exists in the factor graph thats close 
      // enough in time to our measurement
      size_t pose_counter_host = pose_counter_robot_[hostRobotID];
      GetIndexClosestPoseMstPair(poseMstPacketHost, relativeMeas.stamp, indexClosestHostRobot, timeDiffClosest);
      if (indexClosestHostRobot == -1 || timeDiffClosest > maxTimeDiff ||
          indexClosestHostRobot >= pose_counter_host) { 
            continue; 
      }

      // We found a match
      matches.push_back(RelativeMeasMatch(relativeMeas, i, indexClosestHostRobot, indexClosestOtherRobot));

      // Remove this measurement from vector, so we don't use it again
      feasible_relative_meas_for_factors.erase(feasible_relative_meas_for_factors.begin() + i);
      i--;
    }

    // Check which measurements are no longer feasible so we can remove them
    for(int i = 0; i < feasible_relative_meas_for_factors.size(); i++) {

      // Get pose_counters (tracks number of poses added to factor graph)
      RelativeMeas relativeMeas = feasible_relative_meas_for_factors[i];
      size_t pose_counter_observed = pose_counter_robot_[relativeMeas.robotIndex];
      size_t pose_counter_host = pose_counter_robot_[hostRobotID];

      // Get stamp of last poses if factor graphs aren't empty
      ros::Time stamp_observed(0.0);
      ros::Time stamp_host(0.0);
      if (pose_counter_observed > 0) { 
        stamp_observed = robotDataDict_.at(relativeMeas.robotIndex).poseMstPacket[pose_counter_observed - 1].stamp;
      } if (pose_counter_host > 0) { 
        stamp_host = poseMstPacketHost[pose_counter_host - 1].stamp;
      }

      // If relative measurement stamp is earlier than both, then it can no longer be added
      if (stamp_observed > relativeMeas.stamp && stamp_host > relativeMeas.stamp) {
        feasible_relative_meas_for_factors.erase(feasible_relative_meas_for_factors.begin() + i);
        i--;
      }
    }
  } 

  // Unlock mutexes
  feasRelMeasVectorMtx_.unlock();
}

/**
 * @brief This function searches through the deque of 
 * PoseMstPair and finds the message with the 
 * closest timestamp. It returns the corresponding
 * index. If two entries have the same time difference, 
 * then the first index in deque is returned.
 * 
 * @param std::deque<PoseMstPair> &poseMstPacket : The queue to search.
 * @param ros::Time stamp : The stamp we compare to.
 * @param[out] int &indexClosets : Index in postMstPacket that is closest in time.
 *            Returns -1 if empty.
 * @param[out] double &timeDiffClosest : The difference in time between stamp and
 *            the closest PoseMstPair (in seconds).
 */
void sloam::GetIndexClosestPoseMstPair(std::deque<PoseMstPair> &poseMstPacket, 
                  ros::Time stamp, int &indexClosest, double &timeDiffClosest) {
  indexClosest = -1;
  timeDiffClosest = std::numeric_limits<double>::max();

  for(int i = 0; i < poseMstPacket.size(); i++) {
    double currTimeDiff = abs((poseMstPacket[i].stamp - stamp).toSec());
    if (currTimeDiff < timeDiffClosest) {
      indexClosest = i;
      timeDiffClosest = currTimeDiff;
    }
  }
}

} // namespace sloam
