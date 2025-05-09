/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/



#pragma once

#include <definitions.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <plane.h>
#include <semanticObject.h>
#include <utils.h>

#include <algorithm>

// Cylinder object stores information about cylindrical landmarks
struct CylinderParameters {
  Vector3 root;  // root (x,y,z) that lies on line
  Vector3 ray;   // ray (direction)
  std::vector<double> radii;
  std::vector<TreeVertex> vertices;

  double radius;
  double lambda;
  int semantic_label;
};

class Cylinder : public SemanticObject<CylinderParameters> {
 public:
  explicit Cylinder(const std::vector<TreeVertex> vertices, const Plane &gplane,
                    const FeatureModelParams &fmParams);
  explicit Cylinder(const Vector3 &root, const Vector3 &ray,
                    const double &radius, const int &label = -1);
  Scalar distance(const CylinderParameters &model) const;
  Scalar distance(const PointT &point) const;
  void project(const SE3 &tf);
  Cylinder projectAndReturn(const SE3 &tf);
  PointT asPoint(const Scalar height);

 private:
  bool filter(const Scalar &maxTreeRadius, const Scalar &maxAxisTheta,
              const Plane &gplane);
  void computeModel(const std::vector<TreeVertex> &landmarkVtxs,
                    const Scalar defaultCylinderRadius,
                    const Scalar featuresPerTree);
  bool groundBasedRoot(const Plane &gplane);
};
class CylinderCost {
 public:
  CylinderCost(Eigen::Vector3d curr_point, Eigen::Vector3d root,
               Eigen::Vector3d ray, double radius, double weight)
      : curr_point_(curr_point),
        root_(root),
        ray_(ray),
        radius_(radius),
        weight_(weight) {}

  template <typename T>
  bool operator()(const T *const para_t, const T *const para_q,
                  T *residual) const {
    // n: normal of cylinder (x,y,z)
    Eigen::Matrix<T, 3, 1> cp{T(curr_point_[0]), T(curr_point_[1]),
                              T(curr_point_[2])};

    Eigen::Matrix<T, 3, 1> root_p{T(root_[0]), T(root_[1]), T(root_[2])};
    Eigen::Matrix<T, 3, 1> ray_v{T(ray_[0]), T(ray_[1]), T(ray_[2])};

    // last point
    // transform point using current solution
    Eigen::Matrix<T, 3, 1> t_last_curr{para_t[0], para_t[1], para_t[2]};
    Eigen::Quaternion<T> q_last_curr{para_q[3], para_q[0], para_q[1],
                                     para_q[2]};
    Eigen::Matrix<T, 3, 1> lp = q_last_curr * cp + t_last_curr;

    // projected point onto cylinder
    Eigen::Matrix<T, 3, 1> pp =
        root_p + ((lp - root_p).dot(ray_v) / (ray_v.dot(ray_v))) * ray_v;

    T Distance = (pp - lp).norm() - radius_;
    residual[0] = Distance * T(weight_);
    return true;
  }

 private:
  // The measured x,y,z coordinate that should be on the cylinder.
  Eigen::Vector3d curr_point_;
  Eigen::Vector3d root_;
  Eigen::Vector3d ray_;
  double radius_;
  double weight_;
};