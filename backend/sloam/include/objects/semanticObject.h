/**
 * This file is part of SlideSLAM
 *
 * Copyright (C) 2024 Guilherme Nardari, Xu Liu, Jiuzhou Lei, Ankit Prabhu,
 * Yuezhan Tao
 * TODO: License information
 *
 */

#pragma once

#include <definitions.h>

template <typename T>
class SemanticObject {
 public:
  // pure virtual function
  virtual Scalar distance(const T& model) const = 0;
  virtual Scalar distance(const PointT& point) const = 0;
  virtual void project(const SE3& tf) = 0;
  T getModel() const { return model; };
  VectorType getFeatures() const { return features; };
  size_t id;
  bool isValid;
  VectorType features;
  T model;  // object model
};

template <typename T>
void projectObjects(const SE3& tf, std::vector<T>& objs) {
  for (T& o : objs) o.project(tf);
}

template <typename T>
void projectObjects(const SE3& tf, const std::vector<T>& objs,
                    std::vector<T>& projected) {
  for (T o : objs) {
    o.project(tf);
    projected.push_back(o);
  }
}