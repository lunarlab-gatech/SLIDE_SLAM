/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Jiuzhou Lei, Xu Liu, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <databaseManager.h>

databaseManager::databaseManager(const ros::NodeHandle &nh) {
  nh_ = nh;
  timer_ =
      nh_.createTimer(ros::Duration(0.2), &databaseManager::runCommunication_, this);
  std::string node_name = ros::this_node::getName();
  std::string idName = node_name + "/hostRobotID";
  nh_.getParam(idName, hostRobotID_);
  std::string priorTFKnownParamName = node_name + "/priorTFKnown";
  nh_.getParam(priorTFKnownParamName, priorTFKnown_);

  if(priorTFKnown_){
    // ROS_INFO("priorTFKnown_ is true");
    std::string priorTF_x_ParamName = node_name + "/priorTF_x";
    double priorTF_x = 0;
    nh_.getParam(priorTF_x_ParamName, priorTF_x);
    std::string priorTF_y_ParamName = node_name + "/priorTF_y";
    double priorTF_y = 0;
    nh_.getParam(priorTF_y_ParamName, priorTF_y);
    std::string priorTF_z_ParamName = node_name + "/priorTF_z";
    double priorTF_z = 0;
    nh_.getParam(priorTF_z_ParamName, priorTF_z);

    int numRobots = 0;
    nh_.param<int>("number_of_robots", numRobots, 0);

    Eigen::Vector3d priorTF_xyz(priorTF_x, priorTF_y, priorTF_z);
    priorTF2World_ = SE3(Sophus::SO3d(), priorTF_xyz);
    SE3 tfWorld2Robot = priorTF2World_.inverse();
    // ROS_INFO_STREAM("WE ARE USING THE REFERENCE FRAME FROM ROBOT 0 AS THE WORLD FRAME!");
    for (int i = 0; i < numRobots; i++)
    {
      loopClosureTf[i] = tfWorld2Robot;
    }
  }

  std::string msgName = "/robot" + std::to_string(hostRobotID_)+"/PoseMstPairFromOthers";
  std::string robot_ns_prefix_;
  nh_.param<std::string>("robot_ns_prefix", robot_ns_prefix_, "robot");
  nh_.param<double>("communication_wait_time", commWaitTime_, 30.0);
  std::string communicationTriggerTopic =
      robot_ns_prefix_ + std::to_string(hostRobotID_) + "/pose_high_freq";
  
  // create different subscribers for different robots
  int numRobots = 0;
  nh_.param<int>("number_of_robots", numRobots, 0);
  for(int i = 0; i < numRobots; i++){
    poseMstVectorSub_.emplace_back(nh_.subscribe("/robot" + std::to_string(i) + "/PoseMstPairFromOthers", 10, &databaseManager::poseMstCb_, this));
  }
  poseMstPub_ = nh_.advertise<sloam_msgs::PoseMstBundle>(msgName, 10);
  robotDataDict_.emplace(std::make_pair(hostRobotID_, robotData()));
}

void databaseManager::updateRobotMap(const std::vector<Cylinder> &cylMap,
                                     const std::vector<Cube> &cubeMap,
                                     const std::vector<Ellipsoid> &ellipMap,
                                     int robotID) {
  std::vector<Eigen::Vector7d> newMap;
  for (const auto &cyl : cylMap) {
    Eigen::Vector7d cylVec;
    // 7 dimension vector: semantic_label, root_x, root_y, root_z, radius, 0, 0
    cylVec << cyl.model.semantic_label, cyl.model.root.x(), cyl.model.root.y(),
        cyl.model.root.z(), cyl.model.radius, 0, 0;
    newMap.push_back(cylVec);
  }
  for (const auto &cube : cubeMap) {
    Eigen::Vector7d cubeVec;
    // 7 dimension vector: semantic_label, x, y, z, scale_x, scale_y, scale_z
    cubeVec << cube.model.semantic_label, cube.model.pose.translation().x(),
        cube.model.pose.translation().y(), cube.model.pose.translation().z(),
        cube.model.scale.x(), cube.model.scale.y(), cube.model.scale.z();
    newMap.push_back(cubeVec);
  }
  for (const auto &ellip : ellipMap) {
    Eigen::Vector7d ellipVec;
    // 7 dimension vector: semantic_label, x, y, z, scale_x, scale_y, scale_z
    ellipVec << ellip.model.semantic_label, ellip.model.pose.translation().x(),
        ellip.model.pose.translation().y(), ellip.model.pose.translation().z(),
        ellip.model.scale.x(), ellip.model.scale.y(), ellip.model.scale.z();
    newMap.push_back(ellipVec);
  }
    //ROS ERROR THE SIZE OF the map and robot id
    // ROS_ERROR_STREAM("The size of the map is: " << newMap.size());
    // ROS_ERROR_STREAM("The robot id is: " << robotID);
  robotMapDict_[robotID] = newMap;
}

void databaseManager::poseMstCb_(const sloam_msgs::PoseMstBundle &msgs) {
  int robotID = msgs.robotID;
  if (robotDataDict_.find(robotID) == robotDataDict_.end()) {
    robotDataDict_.emplace(std::make_pair(robotID, robotData()));
  }
  size_t bundleSize = msgs.poseMstPair.size();
  size_t poolSize = robotDataDict_[robotID].poseMstPacket.size();
  if (bundleSize > poolSize && robotID != this->hostRobotID_) {
    ROS_DEBUG_STREAM("New robot data received from robot:" << robotID << "by robot:" << this->hostRobotID_);
    int startIdx = poolSize;
    for (int i = startIdx; i < bundleSize; i++) {
      struct PoseMstPair poseMst;
      sloam_msgs::PoseMst singleMsg = msgs.poseMstPair[i];
      SE3 pose = toSE3Pose(singleMsg.pose);
      SE3 relativeOdom = toSE3Pose(singleMsg.relativeRawOdom);
      // construct a PoseMstPair struct
      poseMst.keyPose = pose;
      poseMst.relativeRawOdomMotion = relativeOdom;
      for (int i = 0; i < 6; i++) {
        poseMst.relativeRawOdomMotionCov[i] = singleMsg.relativeRawOdomCov[i];
      }
      poseMst.stamp = singleMsg.stamp;
      // convert msg to object
      for (int j = 0; j < singleMsg.cylinders.size(); j++) {
        sloam_msgs::ROSCylinder curObjMsg = singleMsg.cylinders[j];
        gtsam::Point3 root(curObjMsg.root[0], curObjMsg.root[1],
                           curObjMsg.root[2]);
        gtsam::Point3 ray(curObjMsg.ray[0], curObjMsg.ray[1], curObjMsg.ray[2]);
        double radius = curObjMsg.radius;
        Cylinder tempCylinder(root, ray, radius, curObjMsg.semantic_label);
        poseMst.cylinderMsts.push_back(tempCylinder);
      }
      for (int j = 0; j < singleMsg.cubes.size(); j++) {
        sloam_msgs::ROSCube curObjMsg = singleMsg.cubes[j];
        gtsam::Point3 scale(curObjMsg.dim[0], curObjMsg.dim[1],
                            curObjMsg.dim[2]);
        gtsam::Pose3 gtsamPose = ToGtsamPose3(curObjMsg.pose);
        Cube tempCube(gtsamPose, scale, curObjMsg.semantic_label);
        poseMst.cubeMsts.push_back(tempCube);
      }
      for (int j = 0; j < singleMsg.ellipsoids.size(); j++) {
        sloam_msgs::ROSEllipsoid curObjMsg = singleMsg.ellipsoids[j];
        gtsam::Point3 scale(curObjMsg.scale[0], curObjMsg.scale[1],
                            curObjMsg.scale[2]);
        gtsam::Pose3 gtsamPose = ToGtsamPose3(curObjMsg.pose);
        Ellipsoid tempEp(gtsamPose, scale, curObjMsg.semantic_label);
        poseMst.ellipsoidMsts.push_back(tempEp);
      }
      robotDataDict_[robotID].poseMstPacket.push_back(poseMst);
    }
    std::vector<Eigen::Vector7d> newRobotMap;
    for (int i = 0; i < msgs.map_of_labelXYZ.size(); i++) {
      // construct vector7d first, store it in temp_vector
      Eigen::Vector7d temp_vector;
      temp_vector << msgs.map_of_labelXYZ[i].labelXYZ[0],
          msgs.map_of_labelXYZ[i].labelXYZ[1], msgs.map_of_labelXYZ[i].labelXYZ[2],
          msgs.map_of_labelXYZ[i].labelXYZ[3], msgs.map_of_labelXYZ[i].labelXYZ[4],
          msgs.map_of_labelXYZ[i].labelXYZ[5], msgs.map_of_labelXYZ[i].labelXYZ[6];
      newRobotMap.emplace_back(temp_vector);
    }
    //ROS ERROR THE SIZE OF the map and robot id
    // ROS_ERROR_STREAM("The size of the map is (second): " << newRobotMap.size());
    // ROS_ERROR_STREAM("The robot id is (second): " << robotID);
    robotMapDict_[robotID] = newRobotMap;
    // transmit the loop closure tf
    for (int i = 0; i < msgs.interRobotTFs.size(); i++){
      // if the tf received contain the tf relevant to the host robot
      if(msgs.interRobotTFs[i].targetRobotID == hostRobotID_){
        SE3 tfReceivedTarget2ReceivedHost = toSE3Pose(msgs.interRobotTFs[i].TFfromTarget2Host); // the tf in the msg 
        SE3 tfReceivedHost2Host = tfReceivedTarget2ReceivedHost.inverse(); // now the receiver is the target in the msg, host in the msg is the target for the actual host robot who receives the msg
        loopClosureTf[msgs.interRobotTFs[i].hostRobotID] = tfReceivedHost2Host;
      }
      // if the tf received doesn't contain the tf relevant to the host robot
      else{
        // but the tf can be inferred using existing tf
        int robot_a = msgs.interRobotTFs[i].hostRobotID;
        int robot_b = msgs.interRobotTFs[i].targetRobotID;
        bool robot_a_found = loopClosureTf.find(robot_a) != loopClosureTf.end();
        bool robot_b_found = loopClosureTf.find(robot_b) != loopClosureTf.end();
        SE3 tfB2A = toSE3Pose(msgs.interRobotTFs[i].TFfromTarget2Host);
        SE3 tfA2B = tfB2A.inverse();
        if(!robot_a_found && robot_b_found){
          SE3 tfB2host = loopClosureTf[robot_b];
          SE3 tfA2host = tfA2B* tfB2host;
          loopClosureTf[robot_a] = tfA2host;
        }
        else if(robot_a_found && !robot_b_found){
          SE3 tfA2host = loopClosureTf[robot_a];
          SE3 tfB2host = tfB2A* tfA2host;
          loopClosureTf[robot_b] = tfB2host;
        }
      }
    }
    measureReceivedCommMsgSize(msgs);
  }
  // ROS_DEBUG_STREAM("CURRENT number of robots whose data has been received:"
  //                  << robotDataDict_.size());
}

void databaseManager::measureReceivedCommMsgSize(const sloam_msgs::PoseMstBundle &msgs){
  double cur_received_msg_size_bytes = 0;
  cur_received_msg_size_bytes += 1;
  for (int i = 0; i < msgs.poseMstPair.size(); i++) {
    cur_received_msg_size_bytes += 56;
    cur_received_msg_size_bytes += 56;
    cur_received_msg_size_bytes += 48; // From covariance
    cur_received_msg_size_bytes +=  8; // For the time stamp (theorized, not proven)
    cur_received_msg_size_bytes += 69*msgs.poseMstPair[i].ellipsoids.size();
    cur_received_msg_size_bytes += 69*msgs.poseMstPair[i].cubes.size();
    cur_received_msg_size_bytes += 37*msgs.poseMstPair[i].cylinders.size();
    cur_received_msg_size_bytes += 58*msgs.interRobotTFs.size();
  }
  cur_received_msg_size_bytes += msgs.map_of_labelXYZ.size()*32;
  receivedMsgSizeMB.push_back(cur_received_msg_size_bytes/1000000);
}

/**
 * @brief This function, triggered by a timer every 0.2 seconds, sends
 * messages to the other robots if commWaitTime_ has passed since
 * the last communication was sent (at time lastCommTime_). Thus,
 * effectively sends info to other robots every commWaitTime_ 
 * seconds.
 * 
 * @param ros::TimerEvent &e
 */
void databaseManager::runCommunication_(const ros::TimerEvent &e) {
  ros::Duration time_diff = ros::Time::now() - lastCommTime_;
  if (time_diff.toSec() > commWaitTime_) {
    lastCommTime_ = ros::Time::now();
    ROS_INFO_THROTTLE(3.0, "TRIGGERING COMMUNICATION!");
    double cur_publish_msg_size_bytes = 0;

    // Send all current data on each robot
    for (auto iter = getRobotDataDict().begin();
         iter != getRobotDataDict().end(); iter++) {
      int curRobotID = iter->first;
      sloam_msgs::PoseMstBundle bundleMsg;
      bundleMsg.robotID = curRobotID;

      // Send each pose measurement pair
      for (int i = 0; i < iter->second.poseMstPacket.size(); i++) {
        struct PoseMstPair pmp = iter->second.poseMstPacket[i];
        sloam_msgs::PoseMst pmMsg;
        pmMsg.pose = ToRosPoseMsg(pmp.keyPose);
        pmMsg.stamp = pmp.stamp;
        pmMsg.relativeRawOdom = ToRosPoseMsg(pmp.relativeRawOdomMotion);
        for (int i = 0; i < 6; i++) {
          pmMsg.relativeRawOdomCov[i] = pmp.relativeRawOdomMotionCov[i];
        }
        pmMsg.cylinders = obj2RosObjMsg(pmp.cylinderMsts);
        pmMsg.cubes = obj2RosObjMsg(pmp.cubeMsts);
        pmMsg.ellipsoids = obj2RosObjMsg(pmp.ellipsoidMsts);
        bundleMsg.poseMstPair.push_back(pmMsg);
        cur_publish_msg_size_bytes += 69*pmp.ellipsoidMsts.size();
        cur_publish_msg_size_bytes += 69*pmp.cubeMsts.size();
        cur_publish_msg_size_bytes += 37*pmp.cylinderMsts.size();
        cur_publish_msg_size_bytes += 56;
        cur_publish_msg_size_bytes += 56;
        cur_publish_msg_size_bytes += 8; // From time stamp (not proven)
        cur_publish_msg_size_bytes += 48; // From covariance
      }

      for (int i = 0; i < robotMapDict_[curRobotID].size(); i++) {
        Eigen::Vector7d curPoint = robotMapDict_[curRobotID][i];
        sloam_msgs::vector7d labelXYZ;
        labelXYZ.labelXYZ[0] = curPoint[0];
        labelXYZ.labelXYZ[1] = curPoint[1];
        labelXYZ.labelXYZ[2] = curPoint[2];
        labelXYZ.labelXYZ[3] = curPoint[3];
        labelXYZ.labelXYZ[4] = curPoint[4];
        labelXYZ.labelXYZ[5] = curPoint[5];
        labelXYZ.labelXYZ[6] = curPoint[6];
        bundleMsg.map_of_labelXYZ.push_back(labelXYZ);
        cur_publish_msg_size_bytes += 56;
      }

      // Send each known transformation from other robots to this robot
      for (auto iter=loopClosureTf.begin(); iter!=loopClosureTf.end(); iter++){
        sloam_msgs::interRobotTF interRobotTFMsg;
        interRobotTFMsg.hostRobotID = hostRobotID_;
        interRobotTFMsg.targetRobotID = iter->first;
        interRobotTFMsg.TFfromTarget2Host = ToRosPoseMsg(iter->second);
        bundleMsg.interRobotTFs.push_back(interRobotTFMsg);
        cur_publish_msg_size_bytes += 58;
      }
      poseMstPub_.publish(bundleMsg);
    }
    publishMsgSizeMB.push_back(cur_publish_msg_size_bytes/1000000);
  }
}

// void databaseManager::publishPoseMsts(int robotID) {
//   if (robotDataDict_.find(robotID) == robotDataDict_.end()) {
//     printf("the robot data doesn't exist in the database! No available data to "
//            "publish!\n");
//   } else {
//     std::deque<PoseMstPair> poseMsts = robotDataDict_[robotID].poseMstPacket;
//     sloam_msgs::PoseMstBundle bundleMsg;
//     bundleMsg.robotID = robotID;
//     for (int i = 0; i < poseMsts.size(); i++) {
//       sloam_msgs::PoseMst singleMsg;
//       geometry_msgs::Pose poseMsg = ToRosPoseMsg(poseMsts[i].keyPose);
//       geometry_msgs::Pose relativeOdom =
//           ToRosPoseMsg(poseMsts[i].relativeRawOdomMotion);
//       singleMsg.pose = poseMsg;
//       singleMsg.relativeRawOdom = relativeOdom;
//       for (int mstIdx = 0; mstIdx < poseMsts[i].cubeMsts.size(); mstIdx++) {
//         sloam_msgs::ROSCube rosCubeMsg;
//         Cube curCube = poseMsts[i].cubeMsts[mstIdx];
//         for (int j = 0; j < 3; j++) {
//           rosCubeMsg.dim[j] = curCube.model.scale[j];
//         }
//         rosCubeMsg.pose = gtsamPoseToRosPose(curCube.model.pose);
//         rosCubeMsg.semantic_label = curCube.model.semantic_label;
//         singleMsg.cubes.push_back(rosCubeMsg);
//       }
//       for (int mstIdx = 0; mstIdx < poseMsts[i].cylinderMsts.size(); mstIdx++) {
//         sloam_msgs::ROSCylinder rosCylinderMsg;
//         Cylinder curCylinder = poseMsts[i].cylinderMsts[mstIdx];
//         for (int j = 0; j < 3; j++) {
//           rosCylinderMsg.ray[j] = curCylinder.model.ray[j];
//           rosCylinderMsg.root[j] = curCylinder.model.root[j];
//         }
//         rosCylinderMsg.radius = curCylinder.model.radius;
//         rosCylinderMsg.semantic_label = curCylinder.model.semantic_label;
//         singleMsg.cylinders.push_back(rosCylinderMsg);
//       }
//       for (int mstIdx = 0; mstIdx < poseMsts[i].ellipsoidMsts.size();
//            mstIdx++) {
//         sloam_msgs::ROSEllipsoid rosEllipsoidMsg;
//         Ellipsoid curEllipsoid = poseMsts[i].ellipsoidMsts[mstIdx];
//         for (int j = 0; j < 3; j++) {
//           rosEllipsoidMsg.scale[j] = curEllipsoid.model.scale[j];
//         }
//         rosEllipsoidMsg.pose = gtsamPoseToRosPose(curEllipsoid.model.pose);
//         rosEllipsoidMsg.semantic_label = curEllipsoid.model.semantic_label;
//         singleMsg.ellipsoids.push_back(rosEllipsoidMsg);
//       }
//       bundleMsg.poseMstPair.push_back(singleMsg);
//     }
//     for (int i = 0; i < robotMapDict_[robotID].size(); i++) {
//       Eigen::Vector7d curPoint = robotMapDict_[robotID][i];
//       sloam_msgs::vector7d labelXYZ;
//       labelXYZ.labelXYZ[0] = curPoint[0];
//       labelXYZ.labelXYZ[1] = curPoint[1];
//       labelXYZ.labelXYZ[2] = curPoint[2];
//       labelXYZ.labelXYZ[3] = curPoint[3];
//       labelXYZ.labelXYZ[4] = curPoint[4];
//       labelXYZ.labelXYZ[5] = curPoint[5];
//       labelXYZ.labelXYZ[6] = curPoint[6];
//       bundleMsg.map_of_labelXYZ.push_back(labelXYZ);
//     }
//     poseMstPub_.publish(bundleMsg);
//   }
// }