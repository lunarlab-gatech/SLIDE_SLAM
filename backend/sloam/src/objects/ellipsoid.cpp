/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <ellipsoid.h>

Ellipsoid::Ellipsoid(const gtsam::Pose3 &pose, const gtsam::Point3 &scale,
                     const int &label) {
  // simplified assumption: pose will be centroid + upright orientation (aligned
  // with world frame z-axis)
  model.pose = pose;
  // scale will contain x (length), y (width), z(height)
  // simplified assumption: scale x = scale y since we do not use the
  // orientation in the factor graph 
  model.scale = scale;
  model.semantic_label = label;
}

Scalar Ellipsoid::distance(const EllipsoidParameters &input) const {
  return (input.pose.translation() - model.pose.translation()).norm();
}

Scalar Ellipsoid::distance(const PointT &point) const {
  gtsam::Point3 point_vec = gtsam::Point3(point.x, point.y, point.z);
  return (model.pose.translation() - point_vec).norm();
}

void Ellipsoid::project(const SE3 &tf) {
  // tf means tf_sensor_to_map, not tf_map_to_sensor
  gtsam::Pose3 pose_new = gtsam::Pose3(tf.matrix());
  model.pose = pose_new * model.pose;
  model.scale = model.scale;
}

Ellipsoid Ellipsoid::projectAndReturn(const SE3 &tf) {
  gtsam::Pose3 pose_new = gtsam::Pose3(tf.matrix());
  gtsam::Pose3 newPose = pose_new * model.pose;
  gtsam::Point3 newScale = model.scale;
  int newLabel = model.semantic_label;
  return Ellipsoid(newPose, newScale, newLabel);
}

Scalar Ellipsoid::weightedDistance(const EllipsoidParameters &input,
                                   double dim_weight) const {
  if (input.semantic_label != model.semantic_label) {
    return 1000;
  }

  Scalar position_similarity =
      (input.pose.translation() - model.pose.translation()).norm();
  Scalar scale_similarity = (input.scale - model.scale).norm() / 3.0;

  Scalar weighted_distance =
      (1 - dim_weight) * position_similarity + dim_weight * scale_similarity;
  return weighted_distance;
}