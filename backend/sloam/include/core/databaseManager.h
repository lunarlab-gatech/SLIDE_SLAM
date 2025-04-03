/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Jiuzhou Lei, Xu Liu, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#pragma once

#include <definitions.h>
#include <ros/ros.h>
#include <utils.h>

// include messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sloam_msgs/ROSCube.h>
#include <sloam_msgs/ROSCylinder.h>
#include <sloam_msgs/ROSEllipsoid.h>
#include <sloam_msgs/PoseMst.h>
#include <sloam_msgs/PoseMstBundle.h>
#include <sloam_msgs/vector4d.h>
#include <sloam_msgs/vector7d.h>
// include std
#include <deque>
#include <math.h>
#include <unordered_map>
#include <vector>
// include object factors
#include <cube.h>
#include <cubeFactor.h>
#include <cylinder.h>
#include <cylinderFactor.h>
#include <ellipsoid.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Eigen { 
  typedef Matrix<double, 7, 1> Vector7d; 
}

struct PoseMstPair {
  SE3 keyPose;               // the current pose of the robot
  SE3 relativeRawOdomMotion; // relative motion from last keyPose to current as
                             // measured using odometry
  std::array<double, 6> relativeRawOdomMotionCov; // Covariance for relativeRawOdomMotion
  ros::Time stamp;           // the timestamp associated with the keyPose

  // measurement in robot frame
  std::vector<Cylinder> cylinderMsts;
  std::vector<Cube> cubeMsts;
  std::vector<Ellipsoid> ellipsoidMsts;
};

class robotData {
public:
  size_t bookmarkFG; // the index of the poseMst pair which hasn't been added
                     // into factor graph
  size_t bookmarkLC; // the index of the poseMst pair which hasn't been used to
                     // find a loop closure
  std::deque<PoseMstPair>
      poseMstPacket; // all received data about robot i so far
  std::deque<RelativeMeas> 
      relativeMeasPacket; // all received relative inter-robot measurements

  robotData() {
    bookmarkFG = 0;
    bookmarkLC = 0;
    poseMstPacket.clear();
  }
};

class databaseManager {
public:
  /**
   * @brief Construct a new database Manager object
   *
   * @param nh
   */
  explicit databaseManager(const ros::NodeHandle &nh);

  // void publishPoseMsts(int robotID);

  std::unordered_map<int, SE3> loopClosureTf; // the transformation from other robots to the host robot
  std::unordered_map<int, Cylinder> cylinderMap;
  std::unordered_map<int, Cube> cubeMap;
  std::unordered_map<int, Ellipsoid> ellipsoidMap;

  // communication statistics related
  std::vector<double> publishMsgSizeMB;
  std::vector<double> receivedMsgSizeMB;

  const std::unordered_map<size_t, robotData> &getRobotDataDict() {
    return robotDataDict_;
  }

  std::vector<Eigen::Vector7d> getRobotMap(const int robotID) {
    std::vector<Eigen::Vector7d> map = robotMapDict_[robotID];
    return map;
  }

  int getHostRobotID() { return hostRobotID_; }

  robotData &getHostRobotData() { return robotDataDict_[hostRobotID_]; }

  robotData &getRobotDataByID(const int robotID) { return robotDataDict_[robotID]; }

  void updateFGBookmark(int newBookmark, int robotID) {
    robotDataDict_[robotID].bookmarkFG = newBookmark;
  }

  void updateRobotMap(const std::vector<Cylinder> &cylMap,
                      const std::vector<Cube> &cubeMap,
                      const std::vector<Ellipsoid> &ellipMap, int robotID);

  static std::vector<gtsam_cylinder::CylinderMeasurement>
  ToHostRobotFrame(std::vector<gtsam_cylinder::CylinderMeasurement> cms,
                   const SE3 &tf) {
    std::vector<gtsam_cylinder::CylinderMeasurement> transformedCms;
    gtsam::Pose3 gtsamTf(tf.matrix());
    for (const auto &cm : cms) {
      transformedCms.push_back(cm.project(gtsamTf));
    }
    return transformedCms;
  }

  static std::vector<Cylinder> ToHostRobotFrame(std::vector<Cylinder> &cms,
                                                const SE3 &tf) {
    std::vector<Cylinder> transformedCms;
    for (auto &cm : cms) {
      transformedCms.push_back(cm.projectAndReturn(tf));
    }
    return transformedCms;
  }

  static std::vector<gtsam_cube::CubeMeasurement>
  ToHostRobotFrame(std::vector<gtsam_cube::CubeMeasurement> cms,
                   const SE3 &tf) {
    std::vector<gtsam_cube::CubeMeasurement> transformedCms;
    gtsam::Pose3 gtsamTf(tf.matrix());
    for (const auto &cm : cms) {
      transformedCms.push_back(cm.project(gtsamTf));
    }
    return transformedCms;
  }

  static std::vector<Cube> ToHostRobotFrame(std::vector<Cube> &cms,
                                            const SE3 &tf) {
    std::vector<Cube> transformedCms;
    for (auto &cm : cms) {
      transformedCms.push_back(cm.projectAndReturn(tf));
    }
    return transformedCms;
  }

  static std::vector<Ellipsoid> ToHostRobotFrame(std::vector<Ellipsoid> &cms,
                                                 const SE3 &tf) {
    std::vector<Ellipsoid> transformedCms;
    for (auto &cm : cms) {
      transformedCms.push_back(cm.projectAndReturn(tf));
    }
    return transformedCms;
  }

  static std::vector<gtsam::Point3>
  ToHostRobotFrame(std::vector<gtsam::Point3> centroids, const SE3 &tf) {
    std::vector<gtsam::Point3> transformedCentroids;
    gtsam::Pose3 gtsamTf(tf.matrix());
    for (const auto &centroid : centroids) {
      transformedCentroids.push_back(gtsamTf.transformFrom(centroid));
    }
    return transformedCentroids;
  }

  static void Measurement2Cylinder(
      const std::vector<gtsam_cylinder::CylinderMeasurement> &cms,
      std::vector<Cylinder> &objs) {
    for (const auto &cm : cms) {
      objs.emplace_back(cm.root, cm.ray, cm.radius);
    }
  }

  static void Measurement2Cube(const std::vector<gtsam_cube::CubeMeasurement> &cms,
                   std::vector<Cube> &objs) {
    for (const auto &cm : cms) {
      objs.emplace_back(cm.pose, cm.scale);
    }
  }

  static void object2Measurement(const std::vector<Cube> &objs,
                     std::vector<gtsam_cube::CubeMeasurement> &msts) {
    for (const auto &obj : objs) {
      msts.emplace_back(obj);
    }
  }

  static void object2Measurement(const std::vector<Cylinder> &objs,
                     std::vector<gtsam_cylinder::CylinderMeasurement> &msts) {
    for (const auto &obj : objs) {
      msts.emplace_back(obj);
    }
  }

  void DADebugger(const PoseMstPair &pmp, int robotID) {
    SE3 curPose = pmp.keyPose;
    ROS_DEBUG_STREAM("Current position of robot "
                     << robotID << ":" << curPose.translation()[0] << ","
                     << curPose.translation()[1] << ","
                     << curPose.translation()[2]);
    ROS_DEBUG_STREAM("It observed the following cylinder positions:");
    for (const auto &cm : pmp.cylinderMsts) {
      printObjInfo(cm);
    }
    for (const auto &cm : pmp.cubeMsts) {
      printObjInfo(cm);
    }
  }

  void printObjInfo(const gtsam_cylinder::CylinderMeasurement &obj) {
    gtsam::Point3 root = obj.root;
    ROS_DEBUG_STREAM("(" << root.x() << "," << root.y() << "," << root.z()
                         << ")");
  }

  void printObjInfo(const gtsam_cube::CubeMeasurement &obj) {
    gtsam::Point3 position = obj.pose.translation();
    ROS_DEBUG_STREAM("(" << position.x() << "," << position.y() << ","
                         << position.z() << ")");
  }

  static SE3 toSE3Pose(const geometry_msgs::Pose &pose_msg) {
    // Extract translation and rotation components
    Eigen::Vector3d translation(pose_msg.position.x, pose_msg.position.y,
                                pose_msg.position.z);
    Eigen::Quaterniond rotation(pose_msg.orientation.w, pose_msg.orientation.x,
                                pose_msg.orientation.y, pose_msg.orientation.z);

    Eigen::Matrix3d rotation_matrix = rotation.normalized().toRotationMatrix();
    SE3 se3_transform(rotation_matrix, translation);
    return se3_transform;
  }

private:
  // data structure to store transmitted data by robotID <robotID, robotData>
  ros::NodeHandle nh_;
  int hostRobotID_;
  ros::Publisher poseMstPub_;
  ros::Subscriber poseMstSub_;
  ros::Publisher fakeCommunicationPub_;
  std::unordered_map<size_t, robotData> robotDataDict_;
  std::unordered_map<size_t, std::vector<Eigen::Vector7d>> robotMapDict_;
  bool priorTF2WorldKnown_;
  SE3 priorTF2World_;
  bool priorTFKnown_;
  ros::Timer timer_;

  // communication must happen at least every comm_waittime_ seconds, in other words, how long does the robot wait between two communication attempts
  double commWaitTime_;
  ros::Time lastCommTime_;
  
  std::vector<ros::Subscriber> poseMstVectorSub_;

  void poseMstCb_(const sloam_msgs::PoseMstBundle &msgs);
  void runCommunication_(const ros::TimerEvent &e);
  void measureReceivedCommMsgSize(const sloam_msgs::PoseMstBundle &msgs);
  static gtsam::Pose3 ToGtsamPose3(const geometry_msgs::Pose &pose_msg) {
    gtsam::Point3 translation(pose_msg.position.x, pose_msg.position.y,
                              pose_msg.position.z);

    gtsam::Quaternion quat(pose_msg.orientation.w, pose_msg.orientation.x,
                           pose_msg.orientation.y, pose_msg.orientation.z);
    gtsam::Rot3 rotation(quat);

    gtsam::Pose3 pose(rotation, translation);

    return pose;
  }

  static geometry_msgs::Pose gtsamPoseToRosPose(const gtsam::Pose3 &pose3) {
    geometry_msgs::Pose pose_msg;
    const gtsam::Point3 &translation = pose3.translation();
    pose_msg.position.x = translation.x();
    pose_msg.position.y = translation.y();
    pose_msg.position.z = translation.z();
    const gtsam::Rot3 &rotation = pose3.rotation();
    const gtsam::Quaternion quat = rotation.toQuaternion();
    pose_msg.orientation.x = quat.x();
    pose_msg.orientation.y = quat.y();
    pose_msg.orientation.z = quat.z();
    pose_msg.orientation.w = quat.w();

    return pose_msg;
  }

  static std::vector<sloam_msgs::ROSCube>
  obj2RosObjMsg(const std::vector<Cube> &msts) {
    std::vector<sloam_msgs::ROSCube> rosCubes;
    for (const auto &mst : msts) {
      sloam_msgs::ROSCube curRosCube;
      for (int j = 0; j < 3; j++) {
        curRosCube.dim[j] = mst.model.scale[j];
      }
      curRosCube.pose = gtsamPoseToRosPose(mst.model.pose);
      curRosCube.semantic_label = mst.model.semantic_label;
      rosCubes.push_back(curRosCube);
    }
    return rosCubes;
  }

  static std::vector<sloam_msgs::ROSCylinder>
  obj2RosObjMsg(const std::vector<Cylinder> &msts) {
    std::vector<sloam_msgs::ROSCylinder> rosCylinders;
    for (const auto &mst : msts) {
      sloam_msgs::ROSCylinder rosCylinderMsg;
      for (int j = 0; j < 3; j++) {
        rosCylinderMsg.ray[j] = mst.model.ray[j];
        rosCylinderMsg.root[j] = mst.model.root[j];
      }
      rosCylinderMsg.radius = mst.model.radius;
      rosCylinderMsg.semantic_label = mst.model.semantic_label;
      rosCylinders.push_back(rosCylinderMsg);
    }
    return rosCylinders;
  }

  static std::vector<sloam_msgs::ROSEllipsoid>
  obj2RosObjMsg(const std::vector<Ellipsoid> &msts) {
    std::vector<sloam_msgs::ROSEllipsoid> rosEllipsoids;
    for (const auto &mst : msts) {
      sloam_msgs::ROSEllipsoid rosEllipsoidMsg;
      for (int j = 0; j < 3; j++) {
        rosEllipsoidMsg.scale[j] = mst.model.scale[j];
      }
      rosEllipsoidMsg.pose = gtsamPoseToRosPose(mst.model.pose);
      rosEllipsoidMsg.semantic_label = mst.model.semantic_label;
      rosEllipsoids.push_back(rosEllipsoidMsg);
    }
    return rosEllipsoids;
  }
};