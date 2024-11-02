/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <cube.h>

#include <algorithm>

Cube::Cube(const gtsam::Pose3 &pose, const gtsam::Point3 &scale,
           const int &label) {
  model.pose = pose;
  // scale will contain x (length), y (width), z(height)
  model.scale = scale;
  model.semantic_label = label;
}

Scalar Cube::distance(const CubeParameters &input) const {
  return (input.pose.translation() - model.pose.translation()).norm();
};

Scalar Cube::distance(const PointT &point) const {
  gtsam::Point3 point_vec = gtsam::Point3(point.x, point.y, point.z);
  return (model.pose.translation() - point_vec).norm();
}

void Cube::project(const SE3 &tf) {
  // tf means tf_sensor_to_map, not tf_map_to_sensor
  gtsam::Pose3 pose_new = gtsam::Pose3(tf.matrix());
  model.pose = pose_new * model.pose;
  model.scale = model.scale;
}

Cube Cube::projectAndReturn(const SE3 &tf) {
  gtsam::Pose3 pose_new = gtsam::Pose3(tf.matrix());
  gtsam::Pose3 newPose = pose_new * model.pose;
  gtsam::Point3 newScale = model.scale;
  int newLabel = model.semantic_label;
  return Cube(newPose, newScale, newLabel);
}

Scalar Cube::weightedDistance(const CubeParameters &input,
                              double dim_weight) const {
  if (input.semantic_label != model.semantic_label) {
    // return a large cost for different semantic labels to avoid matching
    // TODO(ankit): replace this with something like std::limits<Scalar>::max()
    // while checking for possible overflow 
    return 1000.0;
  }
  Scalar position_similarity =
      (input.pose.translation() - model.pose.translation()).norm();
  Scalar scale_similarity = (input.scale - model.scale).norm() / 3.0;
  return (1 - dim_weight) * position_similarity + dim_weight * scale_similarity;
}

Scalar Cube::IoU(const CubeParameters &input) const {
  // lower coordinates of the two cubes
  gtsam::Point3 lower1 = model.pose.translation() - model.scale / 2;
  gtsam::Point3 lower2 = input.pose.translation() - input.scale / 2;
  // upper coordinates of the two cubes
  gtsam::Point3 upper1 = model.pose.translation() + model.scale / 2;
  gtsam::Point3 upper2 = input.pose.translation() + input.scale / 2;
  // volume of the two cubes
  Scalar volume1 = model.scale.x() * model.scale.y() * model.scale.z();
  Scalar volume2 = input.scale.x() * input.scale.y() * input.scale.z();
  // intersection volume
  double x_overlap = std::max(
      0.0, std::min(upper1.x(), upper2.x()) - std::max(lower1.x(), lower2.x()));
  double y_overlap = std::max(
      0.0, std::min(upper1.y(), upper2.y()) - std::max(lower1.y(), lower2.y()));
  double z_overlap = std::max(
      0.0, std::min(upper1.z(), upper2.z()) - std::max(lower1.z(), lower2.z()));
  double intersection = x_overlap * y_overlap * z_overlap;
  // union volume
  double union_volume = volume1 + volume2 - intersection;
  return intersection / union_volume;
}