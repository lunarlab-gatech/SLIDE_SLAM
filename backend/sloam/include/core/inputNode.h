#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <cube.h>
#include <definitions.h>
#include <databaseManager.h>
#include <geometry_msgs/PoseStamped.h>
#include <graphWrapper.h>
#include <gtsam/geometry/Point3.h>
#include <input.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sloamNode.h>
#include <sloam_msgs/EvaluateLoopClosure.h>
#include <sloam_msgs/ROSRangeBearing.h>
#include <sloam_msgs/SemanticLoopClosure.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt64.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vizTools.h>

#include <boost/array.hpp>
#include <deque>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "sloam_msgs/ROSRangeBearingSyncOdom.h"
#include "sloam_msgs/ROSSyncOdom.h"

class InputManager : public Input {
 public:
  explicit InputManager(ros::NodeHandle nh);
  void RunInputNode(const ros::TimerEvent &e);
  void saveRuntimeCommUsage();
  Robot robot;

 private:
  void resetAllFlags();
  float runInputNodeRate_;
  ros::Timer timer_;

  void updateLastPose(const StampedSE3 &odom, const int &robotID);

  double max_timestamp_offset_ = 0.01;

  bool callSLOAM(SE3 relativeRawOdomMotion, ros::Time stamp,
                 std::deque<StampedSE3> &odomQueue, const int &robotID);
  void PublishAccumOdom_(const SE3 &relativeRawOdomMotion);
  void Odom2SlamTf();
  void PublishOdomAsTf(const nav_msgs::Odometry &odom_msg,
                       const std::string &parent_frame_id,
                       const std::string &child_frame_id);

  SE3 computeSloamToVioOdomTransform(const SE3 &sloam_odom,
                                     const SE3 &vio_odom);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster broadcaster_;

  // params
  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string robot_ns_prefix_;
  int number_of_robots_;
  std::string odom_topic_;
  std::string robot_frame_id_;
  float minOdomDistance_;
  float minSLOAMAltitude_;

  // vars
  boost::shared_ptr<sloam::SLOAMNode> sloam_ = nullptr;

  bool publishTf_;
  ros::NodeHandle nh_;

  // robotID
  int hostRobotID_;

  bool turn_off_intra_loop_closure_;
  bool turn_off_rel_inter_robot_factor;
};