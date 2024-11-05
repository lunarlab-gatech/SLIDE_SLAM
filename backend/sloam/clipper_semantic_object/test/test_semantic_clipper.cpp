#include "semantic_clipper.h"

int main(){
  /*
  Data preparation
  */
  // read from data file 1
  std::string data1_file_name = "/home/jiuzhou/clipper_semantic_object/examples/data/robot1Map_forest_2d.txt";
  Eigen::Matrix2Xd model = semantic_clipper::read_2d_points(data1_file_name);
  // convert to the format of run_semantic_clipper argument which is vector of vector
  std::vector<std::vector<double>> model_points;
    for (int i = 0; i < model.cols(); i++) {
        model_points.push_back({0, model(0, i), model(1, i), 0, 0, 0, 0, 0});
    }

  // read from data file 2
  std::string data2_file_name = "/home/jiuzhou/clipper_semantic_object/examples/data/robot2Map_forest_2d.txt";
  Eigen::Matrix2Xd data = semantic_clipper::read_2d_points(data2_file_name);
    // convert to the format of run_semantic_clipper argument which is vector of vector
    std::vector<std::vector<double>> data_points;

    for (int i = 0; i < data.cols(); i++) {
        data_points.push_back({0, data(0, i), data(1, i), 0, 0, 0, 0, 0});
    }

    // run semantic clipper
    Eigen::Matrix4d tfFromQuery2Ref;
    double sigma = 0.1;
    double epsilon = 0.3;
    int min_num_pairs = 3;
    double matching_threshold = 0.1;
    bool found = semantic_clipper::run_semantic_clipper(model_points, data_points, tfFromQuery2Ref, sigma, epsilon, min_num_pairs, matching_threshold);

    // print the transformation
    std::cout << "Transformation matrix: " << std::endl;
    std::cout << tfFromQuery2Ref << std::endl;
    return 0;
}