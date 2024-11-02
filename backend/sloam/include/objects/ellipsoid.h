/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/



#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <semanticObject.h>
#include <utils.h>

#include <algorithm>

struct EllipsoidParameters {
  gtsam::Pose3 pose;
  gtsam::Point3 scale;  // x y z
  int semantic_label;
};

class Ellipsoid : public SemanticObject<EllipsoidParameters> {
 public:
  explicit Ellipsoid(const gtsam::Pose3 &pose, const gtsam::Point3 &scale,
                     const int &label = 1);

  Scalar distance(const EllipsoidParameters &model) const;
  Scalar distance(const PointT &point) const;
  // tf means tf_sensor_to_map, not tf_map_to_sensor
  void project(const SE3 &tf);
  Ellipsoid projectAndReturn(const SE3 &tf);
  Scalar weightedDistance(const EllipsoidParameters &model,
                          double dim_weight = 0.0) const;
};
