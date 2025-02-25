/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Guilherme Nardari, Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao
*
* TODO: License information
*
*/

#include <fstream>
#include <pcl/common/io.h>
#include <ros/console.h>
#include <sloamNode.h>

#include <chrono>

namespace sloam {
SLOAMNode::SLOAMNode(const ros::NodeHandle &nh)
    : nh_(nh), dbManager(nh), inter_loopCloser_(nh), intra_loopCloser_(nh) {

  // initialize intra loop closure and inter loop closure nodes 
  intra_loopCloser_.inter_loop_closure = false;
  inter_loopCloser_.inter_loop_closure = true;

  debugMode_ = nh_.param("debug_mode", false);
  if (debugMode_) {
    ROS_DEBUG_STREAM("Running SLOAM in Debug Mode" << std::endl);
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  } else {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Info)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }

  std::string node_name = ros::this_node::getName();
  std::string idName = node_name + "/hostRobotID";


  nh_.param<bool>("use_slidematch", use_slidematch_, true);

  int num_of_robots = nh_.param("number_of_robots", 1);
  inter_robot_place_recognition_frequency_ = nh_.param("inter_robot_place_recognition_frequency", 0.1);
  intra_robot_place_recognition_frequency_ = nh_.param("intra_robot_place_recognition_frequency", 0.1);
  rel_inter_robot_factor_frequency_ = nh_.param("rel_inter_robot_factor_frequency", 1.0);

  // place_recognition_attempt_time_offset
  double place_recognition_attempt_time_offset = nh_.param("place_recognition_attempt_time_offset", 1.5);
  for (int i = 0; i < num_of_robots; i++) {
    std::string topic_name =
        node_name + "/robot" + std::to_string(i) + "/trajectory";
    pubRobotTrajectory_.push_back(
        nh_.advertise<visualization_msgs::MarkerArray>(topic_name, 1, true));
  }

  nh_.param<int>(idName, hostRobotID, 0);


  // initialize last_intra_loop_closure_stamp_ as current time 
  // add offset the timestamp a bit to avoid all robots calling loop closure at the same time when running on the same machine
  last_intra_loop_closure_stamp_ = ros::Time::now() + ros::Duration(place_recognition_attempt_time_offset) * hostRobotID;
  last_inter_loop_closure_stamp_ = last_intra_loop_closure_stamp_;
  last_rel_inter_robot_factor_stamp_ = last_intra_loop_closure_stamp_;

  pubMapTreeModel_ =
      nh_.advertise<visualization_msgs::MarkerArray>("cylinders_map", 1, true);
  pubSubmapTreeModel_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "submap_cylinder_models", 1, true);
  pubObsTreeModel_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "debug/obs_tree_models", 1, true);
  pubMapGroundModel_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "debug/map_ground_model", 1);
  pubObsGroundModel_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "debug/obs_ground_model", 1);

  // cuboids related publishers
  pubMapCubeModel_ =
      nh_.advertise<visualization_msgs::MarkerArray>("cubes_map", 1, true);
  pubSubmapCubeModel_ =
      nh_.advertise<visualization_msgs::MarkerArray>("cubes_submap", 1, true);

  pubAllPointLandmarks_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "optimized_point_landmarks", 1, true);

  pubObs_ = nh_.advertise<sloam_msgs::ROSObservation>("observation", 10);
  pubMapPose_ = nh_.advertise<geometry_msgs::PoseStamped>("map_pose", 10);

  firstScan_ = true;
  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
  initParams_();

  // Intra-Robot Loop Closure
  bool turn_off_intra_loop_closure = nh_.param(node_name+"/turn_off_intra_loop_closure", true);
  if (turn_off_intra_loop_closure) {
    ROS_WARN("Intra Loop Closure is turned off");
  } else {
    intraLoopthread_ = std::thread(&SLOAMNode::intraLoopClosureThread_, this);
    ROS_WARN("Intra Loop Closure is turned on");
  }

  // Inter-Robot Loop Closure
  bool turn_off_inter_loop_closure = nh_.param(node_name+"/turn_off_inter_loop_closure", true);
  if (turn_off_inter_loop_closure) {
    ROS_WARN("Inter Loop Closure is turned off");
  } else {
    interLoopthread_ = std::thread(&SLOAMNode::interLoopClosureThread_, this);
    ROS_WARN("Inter Loop Closure is turned on");
  }
  lastLoopAttemptPose_ = -1;

  // Relative Measurement Inter-Robot Loop Closure
  bool turn_off_rel_inter_robot_factor = nh_.param(node_name+"/turn_off_rel_inter_robot_factor", true);
  if (turn_off_rel_inter_robot_factor) {
    ROS_WARN("Relative Inter-Robot Loop Closure is turned off");
  } else {
    relInterRobotFactorthread_ = std::thread(&SLOAMNode::relInterRobotFactorThread_, this);
    ROS_WARN("Relative Inter-Robot Loop Closure is turned on");
  }

  // initialize the runtime analysis variables 
  runtime_analysis_file = save_runtime_analysis_dir_ + "/robot"+std::to_string(hostRobotID)+"_runtime_analysis.txt";
  ROS_DEBUG_STREAM("THE RUNTIME ANALYSIS FILE IS: " << runtime_analysis_file);
}

/**
 * @brief Make sure all three child threads terminate. 
 */
SLOAMNode::~SLOAMNode() {
  if (intraLoopthread_.joinable())
    intraLoopthread_.join();
  if (interLoopthread_.joinable())
    interLoopthread_.join();
  if (relInterRobotFactorthread_.joinable())
    relInterRobotFactorthread_.join();
}

void SLOAMNode::initParams_() {
  // PARAMETERS
  numRobots = nh_.param("number_of_robots", 1);
  semanticMap_ = CylinderMapManager(numRobots);
  cube_semantic_map_ = CubeMapManager();
  ellipsoid_semantic_map_ = EllipsoidMapManager();
  factorGraph_ = SemanticFactorGraphWrapper(numRobots);
  fmParams_.cylinderMatchThresh = nh_.param("cylinder_match_thresh", 2.0); // Cylinder data association threshold
  fmParams_.cuboidMatchThresh = nh_.param("cuboid_match_thresh",  2.0); // Cuboid data association threshold
  fmParams_.ellipsoidMatchThresh = nh_.param("ellipsoid_match_thresh",  0.75); // Ellipsoid data association threshold
  fmParams_.defaultCylinderRadius = nh_.param("default_cylinder_radius", 0.2); // deafult cylinder radius

  fmParams_.scansPerSweep = 1;
  fmParams_.minGroundModels = nh_.param("min_ground_models", 50); // NO LONGER USED
  fmParams_.maxLidarDist = nh_.param("max_lidar_dist", 20.0); // NO LONGER USED
  fmParams_.maxGroundLidarDist = nh_.param("max_ground_dist", 30.0); // NO LONGER USED
  float beam_cluster_threshold = nh_.param("beam_cluster_threshold", 0.1); // NO LONGER USED
  fmParams_.minGroundLidarDist = nh_.param("min_ground_dist", 0.0); // NO LONGER USED

  fmParams_.twoStepOptim = nh_.param("two_step_optim", true); // NO LONGER USED
  float min_landmark_height = nh_.param("min_landmark_height", 1); // NO LONGER USED
  int min_landmark_size = nh_.param("min_landmark_size", 3); // NO LONGER USED
  int min_vertex_size = nh_.param("min_vertex_size", 2); // NO LONGER USED

  fmParams_.groundRadiiBins = nh_.param("ground_radii_bins", 2); // NO LONGER USED
  fmParams_.groundThetaBins = nh_.param("ground_theta_bins", 10); // NO LONGER USED
  fmParams_.groundMatchThresh = nh_.param("ground_match_thresh", 1.0); // NO LONGER USED
  fmParams_.groundRetainThresh = nh_.param("ground_retain_thresh", 2.0); // NO LONGER USED

  fmParams_.maxTreeRadius = nh_.param("max_tree_radius", 10.0); // NO LONGER USED, checking directly in frontend
  fmParams_.maxAxisTheta = nh_.param("max_axis_theta", 50.0); // NO LONGER USED, checking directly in frontend
  fmParams_.maxFocusOutlierDistance = 0.5;

  fmParams_.featuresPerTree = nh_.param("features_per_tree", 50); // NO LONGER USED
  fmParams_.numGroundFeatures = nh_.param("num_ground_features", 5); // NO LONGER USED

  setFmParams(fmParams_);

  // Frame Ids
  nh_.param<std::string>("map_frame_id", map_frame_id_, "map");
  ROS_DEBUG_STREAM("MAP FRAME " << map_frame_id_);
}

// TODO: visualize transmitted objects in a different color
void SLOAMNode::publishMap_(const ros::Time stamp) {
  sloam_msgs::ROSObservation obs;
  obs.header.stamp = stamp;
  obs.header.frame_id = map_frame_id_;
  if (pubMapTreeModel_.getNumSubscribers() > 0) {
    auto semantic_map = semanticMap_.getFinalMap();
    visualization_msgs::MarkerArray mapTMarkerArray;
    size_t cid = 500000;
    vizTreeModels(semantic_map, mapTMarkerArray, cid);
    pubMapTreeModel_.publish(mapTMarkerArray);
  }
}

void SLOAMNode::publishCubeMaps_(const ros::Time stamp) {
  sloam_msgs::ROSObservation obs;
  obs.header.stamp = stamp;
  obs.header.frame_id = map_frame_id_;
  // get the cubes that are observed more than once
  auto semantic_map = cube_semantic_map_.getFinalMap();
  visualization_msgs::MarkerArray cubeMapTMarkerArray;
  size_t cube_id = 0;
  // publish all cubes that have been observed more than once
  vizCubeModels(semantic_map, cubeMapTMarkerArray, cube_id, true);
  pubMapCubeModel_.publish(cubeMapTMarkerArray);
  visualization_msgs::MarkerArray cubeSubMapTMarkerArray;
  // publish current scan cube map
  vizCubeModels(scan_cubes_world_, cubeSubMapTMarkerArray, cube_id, false);
  pubSubmapCubeModel_.publish(cubeSubMapTMarkerArray);
}

void SLOAMNode::publishResults_(const SloamInput &sloamIn,
                                const SloamOutput &sloamOut, ros::Time stamp,
                                const int &robotID) {
  publishMap_(stamp);
  publishCubeMaps_(stamp);

  std::vector<SE3> allLandmarks;
  std::vector<int> allLabels;
  factorGraph_.getAllCentroidLandmarksAndLabels(allLandmarks, allLabels);

  visualization_msgs::MarkerArray allLandmarksMarkers =
      vizAllCentroidLandmarks(allLandmarks, map_frame_id_, allLabels);

  pubAllPointLandmarks_.publish(allLandmarksMarkers);

  pubMapPose_.publish(makeROSPose(sloamOut.T_Map_Curr, map_frame_id_, stamp));

    std::vector<SE3> trajectory_viz;
    std::vector<size_t> pose_idx;

    // iterate through all robots till numRobots
    for (int temp_id = 0; temp_id < numRobots; temp_id++) {
      factorGraph_.getAllPoses(trajectory_viz, pose_idx, temp_id);
      visualization_msgs::MarkerArray trajMarkers =
          vizTrajectory(trajectory_viz, map_frame_id_, temp_id);
      pubRobotTrajectory_[temp_id].publish(trajMarkers);
    }

    std::vector<SE3> trajectory;
    factorGraph_.getAllPoses(trajectory, pose_idx, robotID);

    // sanity check: the size of the stamped raw odometry pose should be exactly
    // the same as pose_counter_robot_[robotID]
    if (KeyPoseTimeStamps.size() != trajectory.size()) {
      ROS_ERROR("Key pose time stamps and trajectory size do not match!!!");
    } else if (save_robot_trajectory_as_csv_) {
      // Save the trajectory of the robot and corresponding timestamps for
      // each pose as a csv file save rotation as quaternion
      std::ofstream myfile;
      std::string fname = save_results_dir_ + "/trajectory.csv";
      myfile.open(fname);
      myfile << "x,y,z,qx,qy,qz,qw,timestamp\n";
      for (size_t i = 0; i < trajectory.size(); i++) {
        myfile << trajectory[i].translation()[0] << ","
               << trajectory[i].translation()[1] << ","
               << trajectory[i].translation()[2] << ","
               << trajectory[i].so3().unit_quaternion().x() << ","
               << trajectory[i].so3().unit_quaternion().y() << ","
               << trajectory[i].so3().unit_quaternion().z() << ","
               << trajectory[i].so3().unit_quaternion().w() << ","
               << KeyPoseTimeStamps[i] << "\n";
      }
    }

    visualization_msgs::MarkerArray mapTMarkerArray;
    size_t cid = 200000;
    vizTreeModels(sloamIn.submapCylinders, mapTMarkerArray, cid);
    pubSubmapTreeModel_.publish(mapTMarkerArray);

    // Publish aligned observation
    size_t cylinderId = 100000;
    visualization_msgs::MarkerArray obsTMarkerArray;
    vizTreeModels(sloamOut.scanCylindersWorld, obsTMarkerArray, cylinderId);
    pubObsTreeModel_.publish(obsTMarkerArray); 
}


/**
 * @brief loop closure thread for intra-loop closure
 */
void SLOAMNode::intraLoopClosureThread_() {
  ros::Rate rate(0.2);
  double desired_loop_closure_interval = 1.0 / intra_robot_place_recognition_frequency_;

  while (ros::ok()) {
    // if current stamp has passed more than 15 seconds, we continue to try loop
    // closure, otherwise we wait
    if ((ros::Time::now() - last_intra_loop_closure_stamp_).toSec() < desired_loop_closure_interval) {
      ROS_INFO_STREAM_THROTTLE(3.0, "Just did intra loop closure or it's the beginning of the mission, waiting for "
                      << desired_loop_closure_interval << " seconds before next loop closure");
      ros::Duration(0.5).sleep();
      continue;
    }
    if (!isInLoopClosureRegion_) {
      // ROS_INFO("isInLoopClosureRegion_ is false, NOT attempting loop
      // closure");
      ros::Duration(0.1).sleep();
      continue;
    } else {
      ROS_ERROR_THROTTLE(
          1.0, "isInLoopClosureRegion_ is true, attempting loop closure");
    }
    ROS_DEBUG("****** Starting Intra Loop Closure Thread *****");
    // get latest pose and its observation to do loop closure
    semanticMapMtx_.lock();
    int latestPoseIdx = semanticMap_.getLatestPoseIdx(hostRobotID);
    semanticMapMtx_.unlock();
    if (latestPoseIdx < 20) {
      ROS_INFO_THROTTLE(3.0, "latestPoseIdx < 20, skipping loop closure");
      continue;
    }
    dbMutex.lock();

    PoseMstPair latestPoseMstPair =
        dbManager.getHostRobotData().poseMstPacket[latestPoseIdx];
    dbMutex.unlock();
    // extract location of all observations
    std::vector<Eigen::Vector7d> measurements = prepareLCInput(
        latestPoseMstPair.cylinderMsts, latestPoseMstPair.cubeMsts,
        latestPoseMstPair.ellipsoidMsts);
    // get candidate historical pose and its submap to do loop closure
    if (latestPoseIdx == lastLoopAttemptPose_) {
      ROS_ERROR("latestPoseIdx == lastLoopAttemptPose_, skipping loop closure");
    } else {
      num_attempts_intra_loop_closure++;
      lastLoopAttemptPose_ = latestPoseIdx;
      size_t candidatePoseIdx = 0;
      semanticMapMtx_.lock();
      double max_dist = 15;
      size_t at_least_num_of_poses_old = 50;
      bool candidate_key_pose_found = semanticMap_.getLoopCandidateIdx(
          max_dist, latestPoseIdx, candidatePoseIdx, hostRobotID,
          at_least_num_of_poses_old);
      semanticMapMtx_.unlock();
      if (candidate_key_pose_found) {
        ROS_DEBUG("loop closure candidate history key pose found");
        SE3 query_pose = latestPoseMstPair.keyPose;
        SE3 cp = semanticMap_.getPose(candidatePoseIdx, hostRobotID);
        double submap_radius = 20;

        // get the submap at the candidate pose, the submap contains all the
        // objects observed by all the robots in multi-robot case
        // IMPORTANT: the submap is in the map frame
        std::vector<Cylinder> candidateCylinderObs;
        semanticMapMtx_.lock();
        semanticMap_.getkeyPoseSubmap(cp, candidateCylinderObs, submap_radius,
                                      hostRobotID);
        semanticMapMtx_.unlock();

        std::vector<Cube> candidateCubeObs;
        cubeSemanticMapMtx_.lock();
        cube_semantic_map_.getkeyPoseSubmap(cp, candidateCubeObs, submap_radius,
                                            hostRobotID);
        cubeSemanticMapMtx_.unlock();

        std::vector<Ellipsoid> candidateEllipsoidObs;
        ellipsoidSemanticMapMtx_.lock();
        ellipsoid_semantic_map_.getkeyPoseSubmap(cp, candidateEllipsoidObs,
                                                 submap_radius, hostRobotID);
        ellipsoidSemanticMapMtx_.unlock();

        std::vector<Eigen::Vector7d> submaps = prepareLCInput(
            candidateCylinderObs, candidateCubeObs, candidateEllipsoidObs);

        Eigen::Matrix4d tfFromQuery2Candidate;
        int best_number_inliers = 0;
        std::vector<Eigen::Vector3d> map_objects_matched_out;
        std::vector<Eigen::Vector3d> detection_objects_matched_out;
        ROS_DEBUG("number of measurements is: %lu", measurements.size());
        ROS_DEBUG("number of object in submaps is: %lu", submaps.size());

        // Main function for loop closure
        ros::Time loop_closure_start = ros::Time::now();
        if (intra_loopCloser_.findIntraLoopClosure(
                measurements, submaps, query_pose, cp, tfFromQuery2Candidate)) {
          num_successful_intra_loop_closure++;
          ROS_INFO("Success: Intra Loop closure found");
          ros::Time loop_closure_end = ros::Time::now();
          ROS_INFO_STREAM("Intra Loop Closure took "
                          << (loop_closure_end - loop_closure_start).toSec()
                          << " seconds" << "between two submaps of size " << measurements.size() << " and " << submaps.size());
          intra_loop_closure_time.push_back(
              (loop_closure_end - loop_closure_start).toSec());
          last_intra_loop_closure_stamp_ = ros::Time::now();
          Eigen::Matrix3d rotation_matrix =
              tfFromQuery2Candidate.block<3, 3>(0, 0);
          Eigen::Vector3d translation_vector =
              tfFromQuery2Candidate.block<3, 1>(0, 3);

          // Convert to gtsam Pose3, relativePose is the transformation from
          // query to candidate
          gtsam::Pose3 relativePose = gtsam::Pose3(
              gtsam::Rot3(rotation_matrix), gtsam::Point3(translation_vector));

          // add loop closure factor to the factor graph
          factorGraphMtx_.lock();
          ROS_DEBUG_STREAM("A Loop Closure Factor is added between Pose "
                           << latestPoseIdx << "and Pose " << candidatePoseIdx);
          factorGraph_.addLoopClosureFactor(relativePose, candidatePoseIdx,
                                            hostRobotID, latestPoseIdx,
                                            hostRobotID);
          factorGraphMtx_.unlock();
        } else{
          ROS_DEBUG_STREAM("Tried loop closure but was not succesfull");
        }
      } else {
        ROS_DEBUG("No loop closure candidate history key pose found");
      }
    }
    rate.sleep();
  }
}

void SLOAMNode::getCentroidSubmap(const std::vector<SE3> &allCentroidLandmarks,
                                  std::vector<SE3> &centroidSubmap,
                                  const SE3 &query_pose,
                                  const double &submap_radius) {
  for (auto centroid : allCentroidLandmarks) {
    if ((centroid.translation() - query_pose.translation()).norm() <
        submap_radius) {
      centroidSubmap.push_back(centroid);
    }
  }
}

std::vector<Eigen::Vector3d>
SLOAMNode::extractPosition(const std::vector<Cylinder> &candidateCylinderObs,
                           const std::vector<Cube> &candidateCubeObs,
                           const std::vector<SE3> &candidateCentroidObs) {
  std::vector<Eigen::Vector3d> lanmark_positions;
  for (auto cylinder : candidateCylinderObs) {
    lanmark_positions.emplace_back(cylinder.model.root.x(),
                                   cylinder.model.root.y(),
                                   cylinder.model.root.z());
  }
  for (auto cube : candidateCubeObs) {
    gtsam::Point3 cube_center = cube.model.pose.translation();
    lanmark_positions.emplace_back(cube_center.x(), cube_center.y(),
                                   cube_center.z());
  }
  for (auto centroid : candidateCentroidObs) {
    lanmark_positions.emplace_back(centroid.translation().x(),
                                   centroid.translation().y(),
                                   centroid.translation().z());
  }
  return lanmark_positions;
}

std::vector<Eigen::Vector3d> SLOAMNode::extractPosition(
    const std::vector<gtsam_cylinder::CylinderMeasurement>
        &candidateCylinderObs,
    const std::vector<gtsam_cube::CubeMeasurement> &candidateCubeObs,
    const std::vector<gtsam::Point3> &candidateCentroidObs) {
  std::vector<Eigen::Vector3d> lanmark_positions;
  for (auto cylinder : candidateCylinderObs) {
    lanmark_positions.emplace_back(cylinder.root.x(), cylinder.root.y(),
                                   cylinder.root.z());
  }
  for (auto cube : candidateCubeObs) {
    lanmark_positions.emplace_back(cube.pose.translation().x(),
                                   cube.pose.translation().y(),
                                   cube.pose.translation().z());
  }
  for (auto centroid : candidateCentroidObs) {
    lanmark_positions.emplace_back(centroid.x(), centroid.y(), centroid.z());
  }
  return lanmark_positions;
}

std::vector<Eigen::Vector7d>
SLOAMNode::prepareLCInput(const std::vector<Cylinder> &candidateCylinderObs,
                          const std::vector<Cube> &candidateCubeObs,
                          const std::vector<Ellipsoid> &candidateEllipsoidObs) {
  std::vector<Eigen::Vector7d> objects;
  for (auto cylinder : candidateCylinderObs) {
    // 7 dimension vector: semantic_label, root_x, root_y, root_z, radius, 0, 0
    Eigen::Vector7d temp_vec;
    temp_vec << cylinder.model.semantic_label, cylinder.model.root.x(),
        cylinder.model.root.y(), cylinder.model.root.z(),
        cylinder.model.radius, 0, 0;
    objects.emplace_back(temp_vec);
  }
  for (auto cube : candidateCubeObs) {
    // 7 dimension vector: semantic_label, x, y, z, scale_x, scale_y, scale_z
    Eigen::Vector7d temp_vec;
    temp_vec << cube.model.semantic_label, cube.model.pose.translation().x(),
        cube.model.pose.translation().y(), cube.model.pose.translation().z(),
        cube.model.scale.x(), cube.model.scale.y(), cube.model.scale.z();
    objects.emplace_back(temp_vec);
  }
  for (auto ellipsoid : candidateEllipsoidObs) {
    // 7 dimension vector: semantic_label, x, y, z, scale_x, scale_y, scale_z
    Eigen::Vector7d temp_vec;
    temp_vec << ellipsoid.model.semantic_label,
        ellipsoid.model.pose.translation().x(),
        ellipsoid.model.pose.translation().y(),
        ellipsoid.model.pose.translation().z(), ellipsoid.model.scale.x(),
        ellipsoid.model.scale.y(), ellipsoid.model.scale.z();
    objects.emplace_back(temp_vec);
  }
  return objects;
}

void SLOAMNode::interLoopClosureThread_() {
  ros::Rate rate(1.0);
  double desired_loop_closure_interval = 1.0 / inter_robot_place_recognition_frequency_;
  while (ros::ok()) {
    if ((ros::Time::now() - last_inter_loop_closure_stamp_).toSec() < desired_loop_closure_interval) {
      ROS_INFO_STREAM_THROTTLE(3.0, "just did inter loop closure, or in the beginning of the mission, wait for " << desired_loop_closure_interval << " seconds before trying again...");
      ros::Duration(0.5).sleep();
      continue;
    }
    std::vector<int> robotIDLoopClosureToFind;
    dbMutex.lock();
    for (auto iter = dbManager.getRobotDataDict().begin();
         iter != dbManager.getRobotDataDict().end(); iter++) {
      int curRobotID = iter->first;
      if (dbManager.loopClosureTf.find(curRobotID) ==
              dbManager.loopClosureTf.end() &&
          curRobotID != dbManager.getHostRobotID()) {
        robotIDLoopClosureToFind.push_back(curRobotID);
      }
    }
    dbMutex.unlock();
    num_attempts_inter_loop_closure++;
    for (auto query_robot_id : robotIDLoopClosureToFind) {
      ROS_INFO_STREAM("START TO FIND INTER LOOP CLOSURE BETWEEN ROBOTS: "
                      << query_robot_id << " AND " << dbManager.getHostRobotID());
      dbMutex.lock();
      std::vector<Eigen::Vector7d> reference_map =
          dbManager.getRobotMap(dbManager.getHostRobotID());
      if (reference_map.size() == 0) {
        ROS_ERROR("current robot map is empty, skipping loop closure");
        dbMutex.unlock();
        break;
      }

      // get other Robots' map
      std::vector<Eigen::Vector7d> query_map = dbManager.getRobotMap(query_robot_id);
      dbMutex.unlock();
      // ROS_ERROR_STREAM("[INTER LOOP CLOSURE] reference_map size is: " << reference_map.size());
      // ROS_ERROR_STREAM("[INTER LOOP CLOSURE] query_map size is: " << query_map.size());
      // start find the inter loop closure
      Eigen::Matrix4d tfFromQuery2Ref;
      // initialize it to identity matrix
      tfFromQuery2Ref.setIdentity();
      ROS_INFO("Trying to do inter loop closure");
      ros::Time inter_loop_closure_start = ros::Time::now();
      bool found_inter_loop_closure = false;
      
      #if USE_CLIPPER
        if (use_slidematch_){
          ROS_INFO("Using SlideMatch instead of SlideGraph for inter loop closure");
          found_inter_loop_closure = inter_loopCloser_.findInterLoopClosure(
            reference_map, query_map, tfFromQuery2Ref);
        } else {
          ROS_INFO("Using SlideGraph instead of SlideMatch for inter loop closure");
          found_inter_loop_closure =
              inter_loopCloser_.findInterLoopClosureWithClipper(
                  reference_map, query_map, tfFromQuery2Ref);
        }
      #else
        if (use_slidematch_){
          ROS_INFO("Using SlideMatch instead of SlideGraph for inter loop closure");
          found_inter_loop_closure = inter_loopCloser_.findInterLoopClosure(
            reference_map, query_map, tfFromQuery2Ref);
        } else {
          ROS_ERROR("CRITICAL ERROR: Using SlideGraph is requested but build option USE_CLIPPER is not enabled, please enable USE_CLIPPER in CMakeLists.txt and recompile");
          found_inter_loop_closure = false;
        }
      #endif

      if (!found_inter_loop_closure) {
        ROS_WARN_STREAM("INTER LOOP CLOSURE NOT FOUND BETWEEN ROBOTS: "
                         << query_robot_id << " AND " << dbManager.getHostRobotID());
      } else {
        ROS_WARN("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        ROS_WARN_STREAM("INTER LOOP CLOSURE FOUND BETWEEN ROBOTS: "
                         << query_robot_id << " AND " << dbManager.getHostRobotID());
      }

      if (found_inter_loop_closure) {
        num_successful_inter_loop_closure++;
        ROS_WARN_STREAM("INTER LOOP CLOSURE FOUND BETWEEN ROBOTS: "
                         << std::to_string(query_robot_id) << " AND " 
                         << std::to_string(dbManager.getHostRobotID()));
        ros::Time inter_loop_closure_end = ros::Time::now();
        inter_loop_closure_time.push_back(
            (inter_loop_closure_end - inter_loop_closure_start).toSec());
        ROS_INFO_STREAM(
            "Inter Loop Closure took "
            << (inter_loop_closure_end - inter_loop_closure_start).toSec()
            << " seconds"
            << "to match two maps with size " << reference_map.size() << " and "
            << query_map.size());
        // print tfFromQuery2Ref
        ROS_INFO_STREAM("relatively transformation tfFromQuery2Ref is: " << tfFromQuery2Ref);
        ROS_WARN_STREAM("Saving inter robot TF results");
        if (save_inter_robot_closure_results_){
          std::ofstream myfile;
          std::string fname = save_results_dir_ + "/inter-robot-tf-" +
                              std::to_string(query_robot_id) + "-" +
                              std::to_string(dbManager.getHostRobotID()) + "-" +
                              std::to_string(ros::Time::now().toSec()) + ".txt"; 
          myfile.open(fname); 
          for (int temp_i = 0; temp_i < 4; temp_i++) {
            for (int temp_j = 0; temp_j < 4; temp_j++) {
              myfile << tfFromQuery2Ref(temp_i, temp_j) << " ";
            }
            myfile << "\n";
          }
        }
        
        SE3 tfFromQuery2RefSE3;
        tfFromQuery2RefSE3 = SE3(tfFromQuery2Ref);
        dbMutex.lock();
        dbManager.loopClosureTf[query_robot_id] = tfFromQuery2RefSE3;
        dbMutex.unlock();
      }
    }
    rate.sleep();
  }
}

/**
 * @brief This function searches through the deque of 
 * PoseMstPair and finds the message with the 
 * closest timestamp. It returns the corresponding
 * index. If two entries have the same time difference, 
 * then the first index is returned.
 * 
 * @param std::deque<PoseMstPair> &poseMstPacket : The queue to search.
 * @param ros::Time stamp : The stamp we compare to.
 * @return int &indexClosets : Index in postMstPacket that is closest in time.
 *            Returns -1 if empty.
 * @return double &timeDiffClosest : The difference in time between stamp and
 *            the closest PoseMstPair (in seconds).
 */
void SLOAMNode::GetIndexClosestPoseMstPair(std::deque<PoseMstPair> &poseMstPacket, ros::Time stamp, int &indexClosest, double &timeDiffClosest) {
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

/**
 * This function handles relative inter-robot factor generation
 * by iterating through each relative measurement and seeing
 * if the corresponding pair of robot poses (within reasonable
 * time) are found within the graph. If so, its added to the 
 * factor graph. Does this at a rate of
 * rel_inter_robot_factor_frequency_ Hz.
 * 
 * NOTE: Currently relative inter-robot loop closures detected
 * by other robots are NOT sent to other robots, should fix 
 * in the future.
 */
void SLOAMNode::relInterRobotFactorThread_() {
  // Setup a rate 
  double desired_interval = 1.0 / rel_inter_robot_factor_frequency_;
  ros::Rate rate(desired_interval);

  // Maximum time difference between relative inter robot measurement and stamped pose
  // in order to add measurement to the factor graph
  double maxTimeDiff = 0.001; // 1 ms

  while (ros::ok()) {

    // Lock mutexes
    dbMutex.lock();
    feasRelMeasVectorMtx_.lock();

    // Retrive poses from host robot
    std::deque<PoseMstPair> poseMstPacket = dbManager.getHostRobotData().poseMstPacket;

    // Only run loop closures if we have relative inter-robot measurements waiting
    if (feasible_relative_meas_for_factors.size() > 0) {
      ROS_DEBUG("****** Starting Relative Inter Robot Factor Generation Thread *****");

      // Prepare return variables
      int indexClosestHostRobot;
      int indexClosestOtherRobot;
      double timeDiffClosest;
      int matches_found = 0;

      // Try to add each relative measurement
      for(int i = 0; i < feasible_relative_meas_for_factors.size(); i++) {
        RelativeMeas relativeMeas = feasible_relative_meas_for_factors[i];

        // Iterate through the other robot's poses, and make sure one exists in the factor graph
        // that is close enough in time to our measurement
        size_t pose_counter_other = factorGraph_.getPoseCounterById(relativeMeas.robotIndex);
        GetIndexClosestPoseMstPair(dbManager.getRobotDataByID(relativeMeas.robotIndex).poseMstPacket, 
                                  relativeMeas.stamp, indexClosestOtherRobot, timeDiffClosest);
        if (indexClosestOtherRobot == -1 || timeDiffClosest > maxTimeDiff ||
            indexClosestOtherRobot >= pose_counter_other) { continue; }

        // Iterate through our own poses, and make sure one exists in the factor graph thats close 
        // enough in time to our measurement
        size_t pose_counter_host = factorGraph_.getPoseCounterById(relativeMeas.robotIndex);
        GetIndexClosestPoseMstPair(poseMstPacket, relativeMeas.stamp, indexClosestHostRobot, timeDiffClosest);
        if (indexClosestHostRobot == -1 || timeDiffClosest > maxTimeDiff ||
            indexClosestHostRobot >= pose_counter_host) { continue; }

        // We found a match, so add it to the factor graph
        matches_found += 1;
        factorGraphMtx_.lock();
        gtsam::Pose3 relativeMeasPose = SE3ToGTSAMPose3(relativeMeas.relativePose);
        factorGraph_.addLoopClosureFactor(relativeMeasPose, indexClosestHostRobot,
                      hostRobotID, indexClosestOtherRobot, relativeMeas.robotIndex);
        factorGraphMtx_.unlock();
        ROS_DEBUG_STREAM("A Relative Inter-Robot Measurement Factor is added between Robot #" << hostRobotID << " Pose " << indexClosestHostRobot
                         << "and Robot #" << relativeMeas.robotIndex << " Pose " << indexClosestOtherRobot);

        // Remove this measurement from vector, so we don't use it again
        feasible_relative_meas_for_factors.erase(feasible_relative_meas_for_factors.begin() + i);
        i--;
      }

      // Check which factors are no longer feasible for removal
      for(int i = 0; i < feasible_relative_meas_for_factors.size(); i++) {

        // Get pose_counter (tracks number of poses added to factor graph)
        RelativeMeas relativeMeas = feasible_relative_meas_for_factors[i];
        size_t pose_counter = factorGraph_.getPoseCounterById(relativeMeas.robotIndex);

        // If factor graph for this robot isn't empty
        if (pose_counter > 0) { 

          // Get timestamp of most recent pose added to factor graph from observed robot
          ros::Time stamp = dbManager.getRobotDataByID(relativeMeas.robotIndex).poseMstPacket[pose_counter - 1].stamp;

          // If this timestamp is later than relative measurement timestamp
          if (stamp > relativeMeas.stamp) {
            // This relative measurement is no longer feasible, as other robot
            // simply doesn't have a pose factor to connect this measurement to
            feasible_relative_meas_for_factors.erase(feasible_relative_meas_for_factors.begin() + i);
            i--;
          }
        }
      }

      // Keep track of total added
      if (matches_found > 0) {
        ROS_INFO_STREAM("Success: " << matches_found << " Relative Inter Robot Measurement Factors added");
        num_successful_rel_inter_robot_factor += matches_found;
      } else {
        ROS_DEBUG_STREAM("Added no Relative Inter Robot Measurement Factors");
      }
    } 
    else {
      ROS_DEBUG("No Available Relative Inter Robot Measurements");
    }

    // Unlock the mutexes and sleep
    dbMutex.unlock();
    feasRelMeasVectorMtx_.unlock();
    rate.sleep();
  }
}

bool SLOAMNode::runSLOAMNode(const SE3 &relativeRawOdomMotion,
                             const SE3 &prevKeyPose,
                             const std::vector<Cylinder> &cylindersBodyIn,
                             const std::vector<Cube> &cubesBodyIn,
                             const std::vector<Ellipsoid> &ellipsoidBodyIn,
                             ros::Time stamp, SE3 &outPose,
                             const int &robotID) {
  semanticMapMtx_.lock();
  cubeSemanticMapMtx_.lock();
  ellipsoidSemanticMapMtx_.lock();
  dbMutex.lock();
  SE3 poseEstimate = prevKeyPose * relativeRawOdomMotion;
  // compute translation and add to the trajectory length
  double translation = relativeRawOdomMotion.translation().norm();
  trajectory_length += translation;
  // if isInLoopClosureRegion_ clear all measurements to avoid adding them to
  // pollute the map, however, keep track of them in PoseMstPair so that loop
  // closure can use it
  struct PoseMstPair pmp;
  pmp.keyPose = poseEstimate;
  pmp.cylinderMsts = cylindersBodyIn;
  pmp.stamp = stamp;
  pmp.relativeRawOdomMotion = relativeRawOdomMotion;
  pmp.cubeMsts = cubesBodyIn;
  pmp.ellipsoidMsts = ellipsoidBodyIn;
  dbManager.getHostRobotData().poseMstPacket.push_back(pmp);
  std::vector<Cylinder> cylindersBody;
  std::vector<Cube> cubesBody;
  std::vector<Ellipsoid> ellipsoidBody;
  if (isInLoopClosureRegion_) {
    // TODO(xu): record all the semantic measurements and corresponding pose
    // so as to add them after the robot exits the loop closure region
    ROS_WARN_THROTTLE(
        1.0,
        "isInLoopClosureRegion_ is true, clearing all measurements to avoid "
        "polluting the map");
    cylindersBody = std::vector<Cylinder>();
    cubesBody = std::vector<Cube>();
    ellipsoidBody = std::vector<Ellipsoid>();
  } else {
    cylindersBody = cylindersBodyIn;
    cubesBody = cubesBodyIn;
    ellipsoidBody = ellipsoidBodyIn;
  }

  SloamInput sloamIn = SloamInput();
  sloamIn.poseEstimate = poseEstimate;
  sloamIn.distance = relativeRawOdomMotion.translation().norm();
  sloamIn.relativeRawOdomMotion = relativeRawOdomMotion;
  sloamIn.scanCubesBody = cubesBody;
  sloamIn.scanCylindersBody = cylindersBody;
  sloamIn.scanEllipsoidsBody = ellipsoidBody;

  // get Submap at the current Pose
  // get K closest landmarks as submap, this step will update the index
  // matches between submap and global map, i.e., matchesMap_
  semanticMap_.getSubmap(poseEstimate, submap_cylinders_);
  cube_semantic_map_.getSubmap(poseEstimate, submap_cubes_);
  ellipsoid_semantic_map_.getSubmap(poseEstimate, submap_ellipsoids_);
  sloamIn.submapCylinders = submap_cylinders_;
  sloamIn.submapCubes = submap_cubes_;
  sloamIn.submapEllipsoids = submap_ellipsoids_;

  SloamOutput sloamOut = SloamOutput();

  // RunSloam does the following:
  // associates the landmark detections with sub map landmarks
  // input landmarks are in local frame, output landmarks are in the world
  // frame (see sloam.cpp) sloamIn.map: the submap at current pose
  // sloamOut.matches_index: in detected objects, which are already detected
  // in submap. matches[i] is the key of already detected object.
  double da_start = ros::Time::now().toSec();
  bool success = RunSloam(sloamIn, sloamOut);
  double da_end = ros::Time::now().toSec();
  double da_time = da_end - da_start;
  data_association_time.push_back(da_time);

  // when optimization is turned off, then sloamOut.T_Map_Curr will be the
  // same as sloamIn.poseEstimate, which is raw odometry pose
  if (!success) {
    semanticMapMtx_.unlock();
    cubeSemanticMapMtx_.unlock();
    ellipsoidSemanticMapMtx_.unlock();
    dbMutex.unlock();
    return false;
  }

  // Only update map if RunSloam is successful (see cylinderMapManager.cpp)
  semanticMap_.updateMap(sloamOut.T_Map_Curr, sloamOut.scanCylindersWorld,
                         sloamOut.cylinderMatches, robotID);
  // here cubes should be in the global frame (see cubeMapManager.cpp)
  cube_semantic_map_.updateMap(sloamOut.T_Map_Curr, sloamOut.scanCubesWorld,
                               sloamOut.cubeMatches, robotID);
  ellipsoid_semantic_map_.updateMap(sloamOut.T_Map_Curr,
                                    sloamOut.scanEllipsoidsWorld,
                                    sloamOut.ellipsoidMatches, robotID);

  // ROS_INFO("adding observation");

  // sanity check
  if (sloamOut.cubeMatches.size() != sloamOut.scanCubesWorld.size()) {
    ROS_ERROR_STREAM("# cube matches does not matches # detected cubes!!");
  }
  if (sloamOut.cylinderMatches.size() != sloamOut.scanCylindersWorld.size()) {
    ROS_ERROR_STREAM("# cylinder matches does not matches # detected "
                     "cylinders!!");
  }
  if (sloamOut.ellipsoidMatches.size() != sloamOut.scanEllipsoidsWorld.size()) {
    ROS_ERROR_STREAM("# ellipsoid matches does not matches # detected "
                     "ellipsoids!!");
  }

  // This step will add odometry, cylinder and cuboid factors into factor
  // graph, centroid object perform its own DA here
  double fg_optimization_start = ros::Time::now().toSec();
  bool optimized = factorGraph_.addSLOAMObservation(
      semanticMap_, cube_semantic_map_, ellipsoid_semantic_map_,
      sloamOut.cylinderMatches, sloamOut.scanCylindersWorld,
      sloamOut.cubeMatches, sloamOut.scanCubesWorld, sloamOut.ellipsoidMatches,
      sloamOut.scanEllipsoidsWorld, relativeRawOdomMotion, sloamOut.T_Map_Curr,
      robotID);
  double fg_optimization_end = ros::Time::now().toSec();
  double time_diff = fg_optimization_end - fg_optimization_start;
  fg_optimization_time.push_back(time_diff);
  // sanity check: the size of the stamped raw odometry pose should be exactly
  // the same as pose_counter_robot_[robotID]
  KeyPoseTimeStamps.push_back(stamp);
  if (KeyPoseTimeStamps.size() != factorGraph_.getPoseCounterById(robotID)) {
    ROS_ERROR_STREAM(
        "KeyPoseTimeStamps.size() is: " << KeyPoseTimeStamps.size());
    ROS_ERROR_STREAM("factorGraph_.getPoseCounterById(robotID) is: "
                     << factorGraph_.getPoseCounterById(robotID));
    ROS_ERROR_STREAM("KeyPoseTimeStamps.size() is not equal to "
                     "factorGraph_.getPoseCounterById(robotID)");
  }

  // PERFORM INTER-ROBOT LOOP CLOSURE, UPDATE MAP, ADD OBSERVATION, AND
  // PERFORM OPTIMIZATION
  for (auto iter = dbManager.getRobotDataDict().begin();
       iter != dbManager.getRobotDataDict().end(); iter++) {
    int curRobotID = iter->first;
    size_t curBmFG = iter->second.bookmarkFG;
    size_t curBmLC = iter->second.bookmarkLC;
    size_t curSize = iter->second.poseMstPacket.size();

    // MULTI-ROBOT STUFF
    if (dbManager.loopClosureTf.find(curRobotID) !=
            dbManager.loopClosureTf.end() &&
        curRobotID != dbManager.getHostRobotID()) {
      if (curBmFG < curSize) {
        // ROS_INFO_STREAM("START ADDING DATA FROM ROBOT: "
        //                 << curRobotID << " INTO THE FACTOR GRAPH OF ROBOT"
        //                 << dbManager.getHostRobotID());
        // ROS_ERROR("START ADDING DATA FROM OTHER ROBOTS INTO THE FACTOR GRAPH");
        // ROS_INFO("START ADDING DATA FROM OTHER ROBOTS INTO THE FACTOR GRAPH");
        // ROS_INFO("START ADDING DATA FROM OTHER ROBOTS INTO THE FACTOR GRAPH");
        // ROS_INFO("START ADDING DATA FROM OTHER ROBOTS INTO THE FACTOR GRAPH");
        // ROS_INFO_STREAM(
        //     "current number of cylinders:" << semanticMap_.getRawMap().size());
        // ROS_INFO_STREAM(
        //     "current number of cubes:" << cube_semantic_map_.getRawMap().size());
        // ROS_INFO_STREAM("current number of ellipsoids:"
        //                 << ellipsoid_semantic_map_.getRawMap().size());
      }
      for (int i = curBmFG; i < curSize; i++) {
        SE3 poseEstimateInRefFrame = dbManager.loopClosureTf[curRobotID] *
                                     iter->second.poseMstPacket[i].keyPose;
        SE3 relativeRawOdomMotionInRefFrame =
            iter->second.poseMstPacket[i].relativeRawOdomMotion;
        // transform the pose and landmark into host robot map frame
        std::vector<Cylinder> CylinderMeasurement =
            iter->second.poseMstPacket[i].cylinderMsts;
        std::vector<Cube> CubeMeasurement =
            iter->second.poseMstPacket[i].cubeMsts;
        std::vector<Ellipsoid> EllipsoidMeasurement =
            iter->second.poseMstPacket[i].ellipsoidMsts;
        // ROS_WARN_STREAM("NUMBER OF ELLIPSOID IN MEASUREMENT IS: "
        //                 << EllipsoidMeasurement.size());

        // project measurements from the other robot's body frame to current
        // robot's map frame
        projectModels(poseEstimateInRefFrame, CylinderMeasurement,
                      CubeMeasurement, EllipsoidMeasurement);
        std::vector<Cylinder> cylinderInRefFrame = CylinderMeasurement;
        std::vector<Cube> cubeInRefFrame = CubeMeasurement;
        std::vector<Ellipsoid> ellipsoidInRefFrame = EllipsoidMeasurement;

        // Update Map to get updated semanticMap
        std::vector<int> cubeMatchIndices(cubeInRefFrame.size(), -1);
        std::vector<int> cylinderMatchIndices(cylinderInRefFrame.size(), -1);
        std::vector<int> ellipsoidMatchIndices(ellipsoidInRefFrame.size(), -1);
        std::vector<Cylinder> cylinderSubMap;
        std::vector<Cube> cubeSubMap;
        std::vector<Ellipsoid> ellipsoidSubMap;
        semanticMap_.getSubmap(poseEstimateInRefFrame, cylinderSubMap);
        cube_semantic_map_.getSubmap(poseEstimateInRefFrame, cubeSubMap);
        ellipsoid_semantic_map_.getSubmap(poseEstimateInRefFrame,
                                          ellipsoidSubMap);

        matchModels(cylinderInRefFrame, cylinderSubMap, cylinderMatchIndices);
        matchCubeModels(cubeInRefFrame, cubeSubMap, cubeMatchIndices);
        matchEllipsoidModels(ellipsoidInRefFrame, ellipsoidSubMap,
                             ellipsoidMatchIndices);
        semanticMap_.updateMap(poseEstimateInRefFrame, cylinderInRefFrame,
                               cylinderMatchIndices, curRobotID);
        cube_semantic_map_.updateMap(poseEstimateInRefFrame, cubeInRefFrame,
                                     cubeMatchIndices, curRobotID);
        ellipsoid_semantic_map_.updateMap(poseEstimateInRefFrame,
                                          ellipsoidInRefFrame,
                                          ellipsoidMatchIndices, curRobotID);
        // for (int mid : cylinderMatchIndices) {
        //   ROS_INFO_STREAM(mid);
        // }
        // ROS_INFO_STREAM("Cube DA RESULT:");
        // for (int mid : cubeMatchIndices) {
        //   ROS_INFO_STREAM(mid);
        // }
        // add observations into graph
        factorGraph_.addSLOAMObservation(
            semanticMap_, cube_semantic_map_, ellipsoid_semantic_map_,
            cylinderMatchIndices, cylinderInRefFrame, cubeMatchIndices,
            cubeInRefFrame, ellipsoidMatchIndices, ellipsoidInRefFrame,
            relativeRawOdomMotionInRefFrame, poseEstimateInRefFrame, curRobotID,
            false);
      }
      factorGraph_.solve();
      dbManager.updateFGBookmark(curSize, curRobotID);
    }
    // if LC not found yet: loop through the rest pose measurement pair to
    // find one
    else {
      // ROS_INFO("NO INTER-ROBOT LOOP CLOSURE FOUND YET");
    }
  }

  if (optimized) {
    factorGraph_.updateFactorGraphMap(semanticMap_, cube_semantic_map_,
                                      ellipsoid_semantic_map_);
    factorGraph_.getCurrPose(sloamOut.T_Map_Curr, robotID);
  }

  // update the map in databaseManager
  dbManager.updateRobotMap(semanticMap_.getFinalMap(), cube_semantic_map_.getFinalMap(),
                           ellipsoid_semantic_map_.getFinalMap(), robotID);
  total_number_of_landmarks = semanticMap_.getFinalMap().size() +
                               cube_semantic_map_.getFinalMap().size() +
                               ellipsoid_semantic_map_.getFinalMap().size();

  // ROS_DEBUG_STREAM(
  //     "\n---------- FACTOR GRAPH OPTMIZATION POSE OUTPUT -----------------\n"
  //     << sloamOut.T_Map_Curr.matrix()
  //     << "\n-----------------------------------------------------\n");

  outPose = sloamOut.T_Map_Curr;
  publishResults_(sloamIn, sloamOut, stamp, robotID);
  semanticMapMtx_.unlock();
  cubeSemanticMapMtx_.unlock();
  ellipsoidSemanticMapMtx_.unlock();
  dbMutex.unlock();

  return success;
}

/**
 * @brief This method add a relative measurement to the 
 * sloam node so that it can use it for relative inter-robot
 * factor generation. 
 * 
 * @param relativeMeas - A time-stamped relative measurement 
 */
void SLOAMNode::addRelativeMeasurement(RelativeMeas relativeMeas) {
  feasRelMeasVectorMtx_.lock();
  feasible_relative_meas_for_factors.push_back(relativeMeas);
  feasRelMeasVectorMtx_.unlock();
}

} // namespace sloam