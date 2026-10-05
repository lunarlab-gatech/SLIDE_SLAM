/**
 * @file py_slideslam.cpp
 * @brief Python bindings for SlideSLAM's SlideMatch and SlideGraph place recognition
 *
 * Maps are passed as N x 7 arrays (label x y z dim1 dim2 dim3); pairs come back as lists of
 * (reference index, query index) tuples.
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include <array>

#include <place_recognition.h>
#include <semantic_clipper.h>

namespace py = pybind11;
using namespace pybind11::literals;

using Objects = std::vector<Eigen::Vector7d>;
using Points = std::vector<std::vector<double>>;

PYBIND11_MODULE(slideslampy, m)
{
  m.doc() = "Python bindings for SlideSLAM's SlideMatch and SlideGraph place recognition";

  py::class_<SlideMatchParams>(m, "SlideMatchParams")
    .def(py::init<>())
    .def_readwrite("compute_budget_sec", &SlideMatchParams::compute_budget_sec)
    .def_readwrite("dilation_factor", &SlideMatchParams::dilation_factor)
    .def_readwrite("search_xy_step_size", &SlideMatchParams::search_xy_step_size)
    .def_readwrite("match_yaw_half_range", &SlideMatchParams::match_yaw_half_range)
    .def_readwrite("disable_yaw_search", &SlideMatchParams::disable_yaw_search)
    .def_readwrite("search_yaw_step_size_degrees", &SlideMatchParams::search_yaw_step_size_degrees)
    .def_readwrite("match_threshold_position", &SlideMatchParams::match_threshold_position)
    .def_readwrite("match_threshold_dimension", &SlideMatchParams::match_threshold_dimension)
    .def_readwrite("ignore_dimension", &SlideMatchParams::ignore_dimension)
    .def_readwrite("min_num_inliers", &SlideMatchParams::min_num_inliers)
    .def_readwrite("use_nonlinear_least_squares", &SlideMatchParams::use_nonlinear_least_squares)
    .def_readwrite("min_num_map_objects_to_start", &SlideMatchParams::min_num_map_objects_to_start);

  py::class_<SlideGraphParams>(m, "SlideGraphParams")
    .def(py::init<>())
    .def_readwrite("num_inliners_threshold", &SlideGraphParams::num_inliners_threshold)
    .def_readwrite("descriptor_matching_threshold", &SlideGraphParams::descriptor_matching_threshold)
    .def_readwrite("sigma", &SlideGraphParams::sigma)
    .def_readwrite("epsilon", &SlideGraphParams::epsilon)
    .def_readwrite("min_num_map_objects_to_start", &SlideGraphParams::min_num_map_objects_to_start);

  py::class_<PlaceRecognition>(m, "PlaceRecognition")
    .def(py::init<const SlideMatchParams&, const SlideGraphParams&>(),
         "slidematch_params"_a, "slidegraph_params"_a)
    .def("slidematch",
      [](PlaceRecognition& pr, const Objects& reference, const Objects& query) {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        std::vector<std::pair<int, int>> matched_pairs;
        bool accepted = pr.findInterLoopClosure(reference, query, transform, matched_pairs);
        return py::dict("accepted"_a = accepted, "transform"_a = transform, "matched_pairs"_a = matched_pairs);
      },
      "reference"_a, "query"_a,
      "Run SlideMatch. transform (query to reference) is only set when accepted; matched_pairs are\n"
      "(reference index, query index) of the best hypothesis, set whenever the search runs.")
    .def("slidegraph",
      [](PlaceRecognition& pr, const Objects& reference, const Objects& query,
         const std::function<Eigen::VectorXd(int)>& u0_generator) {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        std::vector<std::pair<int, int>> selected_pairs;
        bool accepted = pr.findInterLoopClosureWithClipper(reference, query, transform, u0_generator, selected_pairs);
        return py::dict("accepted"_a = accepted, "transform"_a = transform, "selected_pairs"_a = selected_pairs);
      },
      "reference"_a, "query"_a, "u0_generator"_a,
      "Run SlideGraph. u0_generator(n) must return CLIPPER's initial vector of length n. transform\n"
      "(query to reference) is only set when accepted; selected_pairs are (reference index, query index)\n"
      "of CLIPPER's selection, duplicates included, set whenever CLIPPER runs.");

  // Deterministic SlideGraph stages, exposed for the equivalence test (equivalence_test/)
  m.def("delaunay_triangles",
    [](Points points) {
      DelaunayTriangulation::Observation observation(points);
      std::vector<std::array<int, 3>> triangles;
      for (const auto& triangle : observation.triangles) {
        triangles.push_back({triangle.edges[0].first, triangle.edges[1].first, triangle.edges[2].first});
      }
      return triangles;
    },
    "points"_a, "Vertex indices into points (N x 2, x y) of each Delaunay triangle, in qhull's order.");

  m.def("match_triangles",
    [](Points model_points, Points data_points, double threshold) {
      DelaunayTriangulation::Observation observation_model(model_points);
      DelaunayTriangulation::Observation observation_data(data_points);
      std::vector<double> diffs;
      Points matched_points_model, matched_points_data;
      std::vector<int> matched_indices_model, matched_indices_data;
      semantic_clipper::match_triangles(observation_model.triangles, observation_data.triangles, diffs,
                                        matched_points_model, matched_points_data,
                                        matched_indices_model, matched_indices_data, threshold);
      return py::dict("candidate_model_indices"_a = matched_indices_model,
                      "candidate_data_indices"_a = matched_indices_data, "triangle_diffs"_a = diffs);
    },
    "model_points"_a, "data_points"_a, "threshold"_a,
    "SlideGraph's candidate list for N x 2 (x y) point sets: model and data index of each matched\n"
    "triangle vertex, and each matched triangle pair's descriptor difference.");
}
