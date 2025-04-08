#pragma once

// #include <glog/logging.h>
#include <math.h> /* isnan, sqrt */
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/StdVector>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/multi_array.hpp>
#include <map>
#include <sophus/geometry.hpp>
#include <sophus/se3.hpp>

#define PIDEF 3.14159265

using namespace Eigen;
using namespace boost;

using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;
using Matrix3 = Matrix3d;
using Matrix4 = Matrix4d;
using MatrixX = MatrixXd;
using VectorX = VectorXd;
using Vector4 = Vector4d;
using Vector3 = Vector3d;
using Affine3 = Affine3d;
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using KDTree = pcl::KdTreeFLANN<PointT>;
using Quat = Quaterniond;
using Tran = Translation3d;

typedef std::vector<PointT, Eigen::aligned_allocator<PointT>> VectorType;
typedef std::vector<SE3, Eigen::aligned_allocator<SE3>> SE3VectorType;
using Slash = VectorType;
using Scalar = double;

/*
 * --------------- Trellis Structures ------------------
 */
struct TreeVertex {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int treeId;
  int beam;
  int prevVertexSize;
  Scalar radius;
  bool isValid;
  PointT coords;
  Slash points;
};

using graph_t = adjacency_list<vecS, vecS, directedS, TreeVertex,
                               property<edge_weight_t, float>>;
using vertex_t = graph_traits<graph_t>::vertex_descriptor;
using edge_t = graph_traits<graph_t>::edge_descriptor;

/*
 * --------------- Param Structures ------------------
 */
struct FeatureModelParams {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int scansPerSweep;

  Scalar minGroundModels;
  Scalar maxLidarDist;
  Scalar maxGroundLidarDist;
  Scalar minGroundLidarDist;
  bool twoStepOptim;

  int groundRadiiBins;
  int groundThetaBins;
  Scalar groundRetainThresh;
  Scalar groundMatchThresh;
  Scalar cylinderMatchThresh;       
  Scalar cuboidMatchThresh;
  Scalar ellipsoidMatchThresh;
  Scalar maxTreeRadius;
  Scalar maxAxisTheta;
  Scalar maxFocusOutlierDistance;

  int featuresPerTree;
  int numGroundFeatures;

  Scalar defaultCylinderRadius;
};

/*
 * Relative Inter-Robot Measurements
 */

/**
 * @brief This struct holds relative inter-robot
 * measurements. 
 * 
 * @param stamp - the timestamp.
 * @param odomPose - the synced odometry.
 * @param relativePose - the relative measurement
 *    between the current robot and robot assigned to
 *    "robotIndex".
 * @param robotIndex - the index assigned to the robot
 *    that isn't the host. If onlyUseOdom is true, then
 *    this is the robot that observed us. If onlyUseOdom
 *    is false, this is the robot we observed.
 * @param onlyUseOdom - if true, only use the odometry. 
 *    This is for when we are observed by another robot,
 *    so we add a factor to the graph, but don't add the
 *    relative measurement to the database.
 */
struct RelativeMeas {
  ros::Time stamp;
  SE3 odomPose;
  SE3 relativePose;
  int robotIndex; 
  bool onlyUseOdom;
};

/**
 * @brief This struct holds information on all
 * created relative inter-robot factors so that
 * they can be easily published and visualized
 * with RViz.
 */
struct RelativeInterRobotFactor {
  int hostRobotID;
  int observedRobotID;
  int hostPoseIndex;
  int observedPoseIndex;
  SE3 relativePose;
  ros::Time stamp;
};