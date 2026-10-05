#ifndef SEMANTIC_CLIPPER_H
#define SEMANTIC_CLIPPER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include "clipper/clipper.h"
#include "clipper/utils.h"
#include "triangulation/observation.hpp"
#include <chrono>

// namespace Eigen { 
//   typedef Matrix<double, 7, 1> Vector7d; 
// }

namespace semantic_clipper{

    std::vector<int> argsort(const std::vector<double>& v);

    void compute_triangle_diff(const DelaunayTriangulation::Polygon& triangle_model, const DelaunayTriangulation::Polygon& triangle_data, std::vector<double>& diffs, std::vector<std::vector<double>>& matched_points_model, std::vector<std::vector<double>>& matched_points_data, double threshold);

    void match_triangles(const std::vector<DelaunayTriangulation::Polygon>& triangles_model, const std::vector<DelaunayTriangulation::Polygon>& triangles_data, std::vector<double>& diffs, std::vector<std::vector<double>>& matched_points_model, std::vector<std::vector<double>>& matched_points_data, double threshold);

    bool run_semantic_clipper(const std::vector<std::vector<double>>& reference_map, const std::vector<std::vector<double>>& query_map, Eigen::Matrix4d& tfFromQuery2Ref, double sigma, double epsilon, int min_num_pairs, double matching_threshold);
}

#endif  // SEMANTIC_CLIPPER_H