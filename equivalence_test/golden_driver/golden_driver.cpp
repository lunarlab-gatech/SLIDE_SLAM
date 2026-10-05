// Golden-output driver for the SlideSLAM equivalence test (equivalence_test/test_equivalence.py).
// Builds only against branch `original` (inside the xurobotics/slide-slam image); runs one case, writes JSON.
// Usage: golden_driver <case file> <inputs dir> <output json>   (needs a running roscore)
#include <definitions.h>
#include <ros/ros.h>
#include <semantic_clipper.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/bind.hpp>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unsupported/Eigen/NonLinearOptimization>
#include <vector>

// Test-only: expose PlaceRecognition's private helpers and parameters (getCentroid, *_min_num_map_objects_to_start_, ...)
#define private public
#include <place_recognition.h>
#undef private

struct Case {
  std::string method, reference, query;
  std::vector<std::tuple<std::string, std::string, std::string>> params;  // type, key, value
};

Case loadCase(const std::string &path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open case file " + path);
  Case c;
  std::string line;
  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string tag;
    ss >> tag;
    if (tag == "method") ss >> c.method;
    else if (tag == "reference") ss >> c.reference;
    else if (tag == "query") ss >> c.query;
    else if (tag == "param") {
      std::string type, key, value;
      ss >> type >> key >> value;
      c.params.emplace_back(type, key, value);
    } else if (!tag.empty()) throw std::runtime_error("unknown case line: " + line);
  }
  return c;
}

// One object per line: label x y z dim1 dim2 dim3
std::vector<Eigen::Vector7d> loadMap(const std::string &path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open map file " + path);
  std::vector<Eigen::Vector7d> objects;
  Eigen::Vector7d o;
  while (f >> o[0] >> o[1] >> o[2] >> o[3] >> o[4] >> o[5] >> o[6]) objects.push_back(o);
  return objects;
}

void setParams(ros::NodeHandle &nh, const Case &c) {
  for (const auto &[type, key, value] : c.params) {
    if (type == "double") nh.setParam(key, std::stod(value));
    else if (type == "int") nh.setParam(key, std::stoi(value));
    else if (type == "bool") nh.setParam(key, value == "true");
    else throw std::runtime_error("unknown param type " + type);
  }
}

// Index of the single element satisfying eq; values are copied through the code, never recomputed, so == is exact
int uniqueIndex(size_t n, const std::function<bool(size_t)> &eq, const std::string &what) {
  int found = -1;
  for (size_t i = 0; i < n; i++) {
    if (!eq(i)) continue;
    if (found != -1) throw std::runtime_error("ambiguous " + what);
    found = static_cast<int>(i);
  }
  if (found == -1) throw std::runtime_error("no match for " + what);
  return found;
}

std::string num(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}

template <typename M>
std::string matrix(const M &m) {
  std::string s = "[";
  for (int r = 0; r < m.rows(); r++) {
    s += r ? ", [" : "[";
    for (int col = 0; col < m.cols(); col++) s += (col ? ", " : "") + num(m(r, col));
    s += "]";
  }
  return s + "]";
}

std::string intLists(const std::vector<std::vector<int>> &lists) {
  std::string s = "[";
  for (size_t i = 0; i < lists.size(); i++) {
    s += i ? ", [" : "[";
    for (size_t j = 0; j < lists[i].size(); j++) s += (j ? ", " : "") + std::to_string(lists[i][j]);
    s += "]";
  }
  return s + "]";
}

std::string doubles(const std::vector<double> &v) {
  std::string s = "[";
  for (size_t i = 0; i < v.size(); i++) s += (i ? ", " : "") + num(v[i]);
  return s + "]";
}

std::string runSlideMatch(PlaceRecognition &pr, const std::vector<Eigen::Vector7d> &ref,
                          const std::vector<Eigen::Vector7d> &query) {
  pr.inter_loop_closure = true;
  Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
  bool accepted = pr.findInterLoopClosure(ref, query, tf);

  // Same start condition as findInterLoopClosure
  bool searched = !(ref.size() < pr.slidematch_min_num_map_objects_to_start_ ||
                    query.size() < pr.slidematch_min_num_map_objects_to_start_);
  std::string out = "\"accepted\": " + std::string(accepted ? "true" : "false") +
                    ",\n  \"searched\": " + (searched ? "true" : "false") +
                    ",\n  \"transform\": " + (accepted ? matrix(tf) : "null");
  if (!searched) return out;

  // Same centring as findTransformation; MatchMaps reuses the search range findTransformation just set
  Eigen::Vector2d centroid_reference = pr.getCentroid(ref);
  Eigen::Vector2d centroid_query = pr.getCentroid(query);
  std::vector<Eigen::Vector7d> ref_c = ref, query_c = query;
  for (int i = 0; i < ref.size(); i++) {
    ref_c[i][1] -= centroid_reference[0];
    ref_c[i][2] -= centroid_reference[1];
  }
  for (int i = 0; i < query.size(); i++) {
    query_c[i][1] -= centroid_query[0];
    query_c[i][2] -= centroid_query[1];
  }
  Eigen::Matrix3d R_t = Eigen::Matrix3d::Identity();
  int num_inliers = 0;
  std::vector<Eigen::Vector4d> map_matched, detection_matched;
  pr.MatchMaps(ref_c, query_c, R_t, num_inliers, map_matched, detection_matched);

  std::vector<std::vector<int>> pairs;
  for (size_t k = 0; k < map_matched.size(); k++) {
    int r = uniqueIndex(ref_c.size(), [&](size_t i) { return ref_c[i].head<4>() == map_matched[k]; }, "reference match");
    int q = uniqueIndex(query_c.size(), [&](size_t i) { return query_c[i].head<4>() == detection_matched[k]; }, "query match");
    pairs.push_back({r, q});
  }
  return out + ",\n  \"num_inliers\": " + std::to_string(num_inliers) +
         ",\n  \"best_R_t_centered\": " + matrix(R_t) +
         ",\n  \"matched_pairs\": " + intLists(pairs) +
         ",\n  \"centroid_reference\": " + matrix(centroid_reference.transpose()) +
         ",\n  \"centroid_query\": " + matrix(centroid_query.transpose());
}

std::string runSlideGraph(PlaceRecognition &pr, const std::vector<Eigen::Vector7d> &ref,
                          const std::vector<Eigen::Vector7d> &query) {
  // Same zero-coordinate filter as findInterLoopClosureWithClipper, remembering original indices
  auto filter = [](const std::vector<Eigen::Vector7d> &objects, std::vector<int> &kept,
                   std::vector<std::vector<double>> &points) {
    for (int i = 0; i < objects.size(); i++) {
      if (objects[i][1] == 0.0 && objects[i][2] == 0.0) continue;
      kept.push_back(i);
      points.push_back({objects[i][1], objects[i][2]});
    }
  };
  std::vector<int> ref_kept, query_kept;
  std::vector<std::vector<double>> model_points, data_points;
  filter(ref, ref_kept, model_points);
  filter(query, query_kept, data_points);
  bool ran = model_points.size() >= pr.slidegraph_min_num_map_objects_to_start_ &&
             data_points.size() >= pr.slidegraph_min_num_map_objects_to_start_;

  // CLIPPER itself is not run: its u0 is random in the original, and the original's libsloam_core (no AVX) calling
  // libsemantic_clipper (AVX) crashes in free() from mixed Eigen alignment
  std::string out = "\"ran\": " + std::string(ran ? "true" : "false") +
                    ",\n  \"reference_kept\": " + intLists({ref_kept}) +
                    ",\n  \"query_kept\": " + intLists({query_kept});
  if (!ran) return out;

  // Same deterministic stages as run_semantic_clipper, up to the candidate list handed to CLIPPER
  DelaunayTriangulation::Observation observation_model(model_points);
  DelaunayTriangulation::Observation observation_data(data_points);
  std::vector<double> diffs;
  std::vector<std::vector<double>> matched_points_model, matched_points_data;
  semantic_clipper::match_triangles(observation_model.triangles, observation_data.triangles, diffs,
                                    matched_points_model, matched_points_data, pr.slidegraph_matching_threshold_);

  auto original_index = [](const std::vector<std::vector<double>> &points, const std::vector<int> &kept,
                           const std::vector<double> &p, const std::string &what) {
    return kept[uniqueIndex(points.size(), [&](size_t i) { return points[i] == p; }, what)];
  };
  auto triangles = [&](const std::vector<DelaunayTriangulation::Polygon> &polys,
                       const std::vector<std::vector<double>> &points, const std::vector<int> &kept) {
    std::vector<std::vector<int>> out_tris;
    for (const auto &poly : polys) {
      std::vector<int> tri;
      for (const auto &v : poly.points) tri.push_back(original_index(points, kept, v, "triangle vertex"));
      out_tris.push_back(tri);
    }
    return out_tris;
  };
  std::vector<std::vector<int>> candidates;
  for (size_t i = 0; i < matched_points_model.size(); i++)
    candidates.push_back({original_index(model_points, ref_kept, matched_points_model[i], "candidate model point"),
                          original_index(data_points, query_kept, matched_points_data[i], "candidate data point")});
  return out + ",\n  \"triangles_reference\": " + intLists(triangles(observation_model.triangles, model_points, ref_kept)) +
         ",\n  \"triangles_query\": " + intLists(triangles(observation_data.triangles, data_points, query_kept)) +
         ",\n  \"triangle_diffs\": " + doubles(diffs) +
         ",\n  \"candidate_pairs\": " + intLists(candidates);
}

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "usage: golden_driver <case file> <inputs dir> <output json>" << std::endl;
    return 2;
  }
  ros::init(argc, argv, "slideslam_golden_driver", ros::init_options::AnonymousName);
  ros::NodeHandle nh("golden");

  Case c = loadCase(argv[1]);
  std::string inputs_dir = argv[2];
  std::vector<Eigen::Vector7d> ref = loadMap(inputs_dir + "/" + c.reference);
  std::vector<Eigen::Vector7d> query = loadMap(inputs_dir + "/" + c.query);
  setParams(nh, c);
  PlaceRecognition pr(nh);

  std::string body;
  if (c.method == "slidematch") body = runSlideMatch(pr, ref, query);
  else if (c.method == "slidegraph") body = runSlideGraph(pr, ref, query);
  else throw std::runtime_error("unknown method " + c.method);

  std::ofstream out(argv[3]);
  out << "{\n  \"method\": \"" << c.method << "\",\n  \"num_reference\": " << ref.size()
      << ",\n  \"num_query\": " << query.size() << ",\n  " << body << "\n}\n";
  return out ? 0 : 1;
}
