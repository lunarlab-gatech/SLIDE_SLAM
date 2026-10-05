/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu
*
* TODO: License information
*
*/

#include <place_recognition.h>

#include <chrono>

// define the constructor of the class GraphMatchNode
PlaceRecognition::PlaceRecognition(const SlideMatchParams &slidematch_params,
                                   const SlideGraphParams &slidegraph_params) {
  ParamInit(slidematch_params, slidegraph_params);
}

// define the ParamInit function
void PlaceRecognition::ParamInit(const SlideMatchParams &slidematch_params,
                                 const SlideGraphParams &slidegraph_params) {
  compute_budget_sec_ = slidematch_params.compute_budget_sec;
  dilation_factor_ = slidematch_params.dilation_factor;
  match_xy_step_size_ = slidematch_params.search_xy_step_size;
  double match_yaw_half_range_degrees = slidematch_params.match_yaw_half_range;
  match_yaw_half_range_ = match_yaw_half_range_degrees * M_PI / 180.;
  disable_yaw_search_ = slidematch_params.disable_yaw_search;
  double match_yaw_step_size_degrees =
      slidematch_params.search_yaw_step_size_degrees;
  match_yaw_angle_step_size_ = match_yaw_step_size_degrees * M_PI / 180.;
  match_threshold_ = slidematch_params.match_threshold_position;
  match_threshold_dimension_ = slidematch_params.match_threshold_dimension;
  ignore_dimension_ = slidematch_params.ignore_dimension;
  // min_num_inliers
  min_num_inliers_ = slidematch_params.min_num_inliers;
  use_lsq = slidematch_params.use_nonlinear_least_squares;
  slidematch_min_num_map_objects_to_start_ =
      slidematch_params.min_num_map_objects_to_start;
  slidegraph_num_inliners_ = slidegraph_params.num_inliners_threshold;
  slidegraph_matching_threshold_ =
      slidegraph_params.descriptor_matching_threshold;
  slidegraph_sigma_ = slidegraph_params.sigma;
  slidegraph_epsilon_ = slidegraph_params.epsilon;
  slidegraph_min_num_map_objects_to_start_ =
      slidegraph_params.min_num_map_objects_to_start;

  printParams();
}

void PlaceRecognition::printParams() {
  std::cout << "[PlaceRecognition]: compute_budget_sec_: " << compute_budget_sec_ << std::endl;
  std::cout << "[PlaceRecognition]: dilation_factor_: " << dilation_factor_ << std::endl;
  std::cout << "[PlaceRecognition]: match_xy_step_size_: " << match_xy_step_size_ << std::endl;
  std::cout << "[PlaceRecognition]: match_yaw_half_range_: " << match_yaw_half_range_ << std::endl;
  std::cout << "[PlaceRecognition]: disable_yaw_search_: " << disable_yaw_search_ << std::endl;
  std::cout << "[PlaceRecognition]: match_yaw_angle_step_size_: " << match_yaw_angle_step_size_ << std::endl;
  std::cout << "[PlaceRecognition]: match_threshold_: " << match_threshold_ << std::endl;
  std::cout << "[PlaceRecognition]: match_threshold_dimension_: " << match_threshold_dimension_ << std::endl;
  std::cout << "[PlaceRecognition]: ignore_dimension_: " << ignore_dimension_ << std::endl;
  // ROS_INFO_STREAM("[PlaceRecognition]: min_loop_closure_overlap_percentage_: "
  //                 << min_loop_closure_overlap_percentage_);
  std::cout << "[PlaceRecognition]: min_num_inliers_: " << min_num_inliers_ << std::endl;
  std::cout << "[PlaceRecognition]: use_lsq: " << use_lsq << std::endl;
}

// define the match_maps function
void PlaceRecognition::MatchMaps(
    const std::vector<Eigen::Vector7d> &reference_objects,
    const std::vector<Eigen::Vector7d> &query_objects, Eigen::Matrix3d &R_t_out,
    int &best_num_inliers_out,
    std::vector<Eigen::Vector4d> &map_objects_matched_out,
    std::vector<Eigen::Vector4d> &detection_objects_matched_out) {
  // Apply the sampled transformations on detection_objects
  Eigen::MatrixXd detection_object_xy_homogeneous =
      Eigen::MatrixXd::Ones(query_objects.size(), 3);
  for (int i = 0; i < query_objects.size(); i++) {
    detection_object_xy_homogeneous(i, 0) = query_objects[i][1];
    detection_object_xy_homogeneous(i, 1) = query_objects[i][2];
  }

  Eigen::MatrixXd map_objects_label_xy =
      Eigen::MatrixXd::Zero(reference_objects.size(), 7);
  for (int i = 0; i < reference_objects.size(); i++) {
    map_objects_label_xy(i, 0) = reference_objects[i][0];
    map_objects_label_xy(i, 1) = reference_objects[i][1];
    map_objects_label_xy(i, 2) = reference_objects[i][2];
    map_objects_label_xy(i, 3) = reference_objects[i][3];
    map_objects_label_xy(i, 4) = reference_objects[i][4];
    map_objects_label_xy(i, 5) = reference_objects[i][5];
    map_objects_label_xy(i, 6) = reference_objects[i][6];
  }

  // initialize the best number of inliers
  int best_num_inliers = -10000;
  // initialize the best transformation matrix
  Eigen::Matrix3d best_R_t = Eigen::Matrix3d::Identity();
  // initialize the best matched map objects
  std::vector<Eigen::Vector4d> best_matched_map_objects =
      std::vector<Eigen::Vector4d>();
  // initialize the best matched detection objects
  std::vector<Eigen::Vector4d> best_matched_detection_objects =
      std::vector<Eigen::Vector4d>();

  // TODO: break this down to small functions
  std::vector<double> yaw_candidates;
  if (disable_yaw_search_) {
    yaw_candidates.push_back(0.0);
  } else {
    for (double yaw_raw = -match_yaw_half_range_;
         yaw_raw < match_yaw_half_range_;
         yaw_raw += match_yaw_angle_step_size_) {
      double yaw = yaw_raw;
      yaw_candidates.push_back(yaw);
    }
  }

  // Anytime implementation
  // start with the center (0, 0), gradually expand the search range
  // until reaching either the max time budget or the max search range
  auto start_cpu = std::chrono::high_resolution_clock::now();
  // outer loop step size around 10 x match_xy_step_size_ to make sure we have
  // significant number of samples each time
  double outer_loop_step_size = 10 * match_xy_step_size_;
  double outer_loop_steps_double =
      std::min(match_x_half_range_, match_y_half_range_) / outer_loop_step_size;
  // get the ceiling of outer_loop_steps_double
  int outer_loop_steps = std::ceil(outer_loop_steps_double);
  // this may cause the outer loop step size to be larger than
  // outer_loop_steps_double if match_x_half_range_ != match_y_half_range_ but
  // it should not matter much since this is just for roughly determining the
  // number of steps each time
  double outer_loop_step_size_x =
      match_x_half_range_ / static_cast<double>(outer_loop_steps);
  double outer_loop_step_size_y =
      match_y_half_range_ / static_cast<double>(outer_loop_steps);
  // add sanity check: make sure outer_loop_step_size_x and
  // outer_loop_step_size_y are larger than match_xy_step_size_
  if (outer_loop_step_size_x < match_xy_step_size_ ||
      outer_loop_step_size_y < match_xy_step_size_) {
    std::cerr <<
        "Outer loop step size is smaller than match_xy_step_size_, this should "
        "not happen" << std::endl;
    return;
  }

  // Outer loop: gradually expand the search range
  for (int cur_step = 0; cur_step < outer_loop_steps; cur_step++) {
    // Anytime algorithm: check if we have exceeded the time budget
    // get the current time
    auto cur_cpu = std::chrono::high_resolution_clock::now();
    // get the duration in seconds
    double duration =
        std::chrono::duration_cast<std::chrono::seconds>(cur_cpu - start_cpu)
            .count();
    // check if we have exceeded the time budget
    if (duration > compute_budget_sec_) {
      std::cerr <<
          "Exceeded the time budget, break the loop, current duration is: "
          << duration << " seconds, and current step is: " << cur_step << std::endl;
      break;
    } else {
      std::cout << "[PlaceRecognition]: Time taken up till now is: "
                      << duration
                      << " seconds, and current step is: " << cur_step << std::endl;
    }

    std::vector<double> x_candidates;
    std::vector<double> y_candidates;
    double cur_step_double = static_cast<double>(cur_step);
    double x_positive_start = cur_step_double * outer_loop_step_size_x;
    // this is also the end (right boundary) of the previous step in the
    // positive direction
    double x_right_boundary_prev = cur_step_double * outer_loop_step_size_x;
    double x_positive_end = (cur_step_double + 1) * outer_loop_step_size_x;
    double x_negative_start = -(cur_step_double + 1) * outer_loop_step_size_x;
    double x_negative_end = -cur_step_double * outer_loop_step_size_x;
    // this is also the start (left boundary) of the previous step in the
    // negative direction
    double x_left_boundary_prev = -cur_step_double * outer_loop_step_size_x;
    double y_positive_start = cur_step_double * outer_loop_step_size_y;
    // this is also the end  (right boundary) of the previous step in the
    // positive direction
    double y_right_boundary_prev = cur_step_double * outer_loop_step_size_y;
    double y_positive_end = (cur_step_double + 1) * outer_loop_step_size_y;
    double y_negative_start = -(cur_step_double + 1) * outer_loop_step_size_y;
    double y_negative_end = -cur_step_double * outer_loop_step_size_y;
    // this is also the start (left boundary) of the previous step in the
    // negative direction
    double y_left_boundary_prev = -cur_step_double * outer_loop_step_size_y;
    std::cout << "[PlaceRecognition]: Current region boundaries: x_positive_start: "
                    << x_positive_start
                    << ", x_positive_end: " << x_positive_end
                    << ", x_negative_start: " << x_negative_start
                    << ", x_negative_end: " << x_negative_end << std::endl;

    // Inner loop: iterate through all x and y within current step
    // Loop at the entire region and remove the center rectangle that has been
    // considered before since we need to get the "ring" region
    for (double x = x_negative_start; x <= x_positive_end;
         x += match_xy_step_size_) {
      for (double y = y_negative_start; y <= y_positive_end;
           y += match_xy_step_size_) {

        // skip the ones that lie in the center rectangle, which we already
        // considered in the previous step note that this need to be conditioned
        // on both X and Y
        if ((x >= x_left_boundary_prev && x <= x_right_boundary_prev) &&
            (y >= y_left_boundary_prev && y <= y_right_boundary_prev)) {
          continue;
        }

        // iterate through all the yaw candidates
        Eigen::Matrix3d cur_R_t = Eigen::Matrix3d::Identity();
        for (double yaw : yaw_candidates) {
          cur_R_t(0, 0) = cos(yaw);
          cur_R_t(0, 1) = -sin(yaw);
          cur_R_t(0, 2) = x;
          cur_R_t(1, 0) = sin(yaw);
          cur_R_t(1, 1) = cos(yaw);
          cur_R_t(1, 2) = y;
          // iterate through the detection objects
          Eigen::MatrixXd cur_detection_objects_transformed =
              Eigen::MatrixXd::Zero(query_objects.size(), 7);
          for (int j = 0; j < query_objects.size(); j++) {
            // apply the transformation on the detection object
            Eigen::Vector3d cur_object_transformed =
                cur_R_t * detection_object_xy_homogeneous.row(j).transpose();
            // normalize the homogeneous coordinates
            cur_object_transformed =
                cur_object_transformed / cur_object_transformed[2];
            // put the label and XY coordinates into the matrix
            cur_detection_objects_transformed(j, 0) = query_objects[j][0];
            cur_detection_objects_transformed(j, 1) = cur_object_transformed[0];
            cur_detection_objects_transformed(j, 2) = cur_object_transformed[1];
            cur_detection_objects_transformed(j, 3) = query_objects[j][3];
            cur_detection_objects_transformed(j, 4) = query_objects[j][4];
            cur_detection_objects_transformed(j, 5) = query_objects[j][5];
            cur_detection_objects_transformed(j, 6) = query_objects[j][6];
          }

          int cur_inliers = 0;
          // record all the matched map objects and detection objects in vector
          std::vector<Eigen::Vector4d> cur_matched_map_objects =
              std::vector<Eigen::Vector4d>();
          std::vector<Eigen::Vector4d> cur_matched_detection_objects =
              std::vector<Eigen::Vector4d>();

          // iterate through all the detection objects, find the best match in
          // the map objects
          for (int cur_idx = 0;
               cur_idx < cur_detection_objects_transformed.rows(); cur_idx++) {
            // take the first two element, which are the XY coordinates
            // take the label from detection_objects_original
            // get all inside map_objects_xyl that (1) has the same label and
            // (2) is within self.match_threshold_ distance to
            // cur_detection_object get the label of cur_detection_object
            double cur_detection_object_label =
                cur_detection_objects_transformed(cur_idx, 0);

            // get the XY coordinates of cur_detection_object
            Eigen::Vector2d cur_detection_object_xy =
                cur_detection_objects_transformed.block<1, 2>(cur_idx, 1);

            // check if there is any object in map_objects_label_xy that has the
            // same label as cur_detection_object, and is within
            // self.match_threshold_ distance to cur_detection_object iterate
            // through all the map objects
            for (int cur_map_idx = 0; cur_map_idx < map_objects_label_xy.rows();
                 cur_map_idx++) {
              // get the current map object label X Y
              Eigen::Vector7d cur_map_object_label_xy =
                  map_objects_label_xy.block<1, 7>(cur_map_idx, 0);
                  
              // first check if the label is the same to be most efficient
              if (cur_map_object_label_xy[0] != cur_detection_object_label) {
                continue;
              } else {
                // check if the distance is within self.match_threshold_
                double x_diff =
                    cur_map_object_label_xy[1] - cur_detection_object_xy[0];
                double y_diff =
                    cur_map_object_label_xy[2] - cur_detection_object_xy[1];
                // avg dim diff
                double avg_dim_diff = 0;
                // for cylinder, we only have one non-zero dimension, so we do
                // not divide by 3 check if both dim 2 and dim 3 are zeros
                if (cur_map_object_label_xy[5] == 0 &&
                    cur_map_object_label_xy[6] == 0) {
                  avg_dim_diff =
                      std::abs(cur_map_object_label_xy[4] -
                               cur_detection_objects_transformed(cur_idx, 4));
                } else {
                  for (int cur_dim_i = 4; cur_dim_i < 7; cur_dim_i++) {
                    avg_dim_diff += std::abs(
                        cur_map_object_label_xy[cur_dim_i] -
                        cur_detection_objects_transformed(cur_idx, cur_dim_i));
                  }
                  avg_dim_diff /= 3;
                }
                // two flags: distance match and dimension match
                bool distance_match =
                    sqrt(x_diff * x_diff + y_diff * y_diff) < match_threshold_;
                bool dimension_match;
                if (ignore_dimension_) {
                  dimension_match = true;
                } else {
                  dimension_match = avg_dim_diff < match_threshold_dimension_;
                }

                if (distance_match && dimension_match) {
                  // if so, add one to the number of inliers
                  cur_inliers++;
                  Eigen::Vector4d cur_map_object(
                      cur_map_object_label_xy[0], cur_map_object_label_xy[1],
                      cur_map_object_label_xy[2], cur_map_object_label_xy[3]);
                  cur_matched_map_objects.push_back(cur_map_object);
                  cur_matched_detection_objects.push_back(Eigen::Vector4d(
                      cur_detection_object_label, query_objects[cur_idx][1],
                      query_objects[cur_idx][2], query_objects[cur_idx][3]));
                  // once inlier is found, break the loop since we only need one
                  // inlier for each detection object
                  break;
                }
              }
            }
          }

          // check if the current number of inliers is larger than the best
          // number of inliers
          if (cur_inliers > best_num_inliers) {
            // update the best number of inliers
            best_num_inliers = cur_inliers;
            // update the best transformation matrix
            best_R_t = cur_R_t;  // sampled_transformations[cur_tf_id];
            // update the best matched map objects
            best_matched_map_objects = cur_matched_map_objects;
            // update the best matched detection objects
            best_matched_detection_objects = cur_matched_detection_objects;
          }
        }
      }
    }
  } 
  // print the best number of inliers ROS INFO
  std::cout << "[PlaceRecognition]: best number of inliers: " << best_num_inliers << std::endl;
  // print the best transformation matrix
  std::cout << "[PlaceRecognition]: best transformation matrix: " << best_R_t << std::endl;
  // assign the best transformation matrix to R_t_out
  R_t_out = best_R_t;
  // assign the best number of inliers to best_num_inliers_out
  best_num_inliers_out = best_num_inliers;
  // assign the best matched map objects to map_objects_matched_out
  map_objects_matched_out = best_matched_map_objects;
  // assign the best matched detection objects to detection_objects_matched_out
  detection_objects_matched_out = best_matched_detection_objects;
}

bool PlaceRecognition::findInterLoopClosure(
    const std::vector<Eigen::Vector7d> &reference_objects,
    const std::vector<Eigen::Vector7d> &query_objects,
    Eigen::Matrix4d &tfFromQueryToRef) {
  // IMPORTANT: each object in reference_objects and query_objects is in the [label, x, y, z, dim1, dim2, dim3] format
  // call the match_maps function
  std::vector<double> xyzYaw_out;
  Eigen::Matrix4d transform_out;
  bool closure_found = false;
  // only call the findTransformation function if the number of objects in both maps exceed the slidematch_min_num_map_objects_to_start_
  if (reference_objects.size() < slidematch_min_num_map_objects_to_start_ ||
      query_objects.size() < slidematch_min_num_map_objects_to_start_) {
    std::cerr << "[PlaceRecognition]: number of objects in reference_objects or query_objects is less than slidematch_min_num_map_objects_to_start_" << std::endl;
  } else {
    closure_found = findTransformation(reference_objects, query_objects,
                                            xyzYaw_out, transform_out);
  }
  if (!closure_found) {
    std::cerr <<
        "[findInterLoopClosure] no loop closure found due to not enough "
        "inliers" << std::endl;
    return false;
  } else {
    std::cout << "[findInterLoopClosure] SUCCESS: loop closure found!" << std::endl;
    // compose a transformation matrix
    double x = xyzYaw_out[0];
    double y = xyzYaw_out[1];
    double z = xyzYaw_out[2];
    double yaw = xyzYaw_out[3];

    tfFromQueryToRef = Eigen::Matrix4d::Identity();
    tfFromQueryToRef(0, 3) = x;
    tfFromQueryToRef(1, 3) = y;
    tfFromQueryToRef(2, 3) = z;
    tfFromQueryToRef(0, 0) = cos(yaw);
    tfFromQueryToRef(0, 1) = -sin(yaw);
    tfFromQueryToRef(1, 0) = sin(yaw);
    tfFromQueryToRef(1, 1) = cos(yaw);
    return true;
  }
}

#if USE_CLIPPER
bool PlaceRecognition::findInterLoopClosureWithClipper(
    const std::vector<Eigen::Vector7d> &reference_objects,
    const std::vector<Eigen::Vector7d> &query_objects,
    Eigen::Matrix4d &tfFromQueryToRef) {

    // // save the input in a file for debugging
    // std::ofstream reference_objects_file;
    // // TODO(ankit): Remove hardcoding of the file path
    // std::string save_reference_objects_file_path = "/home/sam/reference_objects.txt";
    // // if not exist, create the file, if exist, truncate it
    // reference_objects_file.open(save_reference_objects_file_path);
    // for (int i = 0; i < reference_objects.size(); i++) {
    //   reference_objects_file << reference_objects[i][0] << " " << reference_objects[i][1] << " " << reference_objects[i][2] << " " << reference_objects[i][3] << " " << reference_objects[i][4] << " " << reference_objects[i][5] << " " << reference_objects[i][6] << std::endl;
    // }
    // reference_objects_file.close();

    // std::ofstream query_objects_file;
    // std::string save_query_objects_file_path = "/home/sam/query_objects.txt";
    // query_objects_file.open(save_query_objects_file_path);
    // for (int i = 0; i < query_objects.size(); i++) {
    //   query_objects_file << query_objects[i][0] << " " << query_objects[i][1] << " " << query_objects[i][2] << " " << query_objects[i][3] << " " << query_objects[i][4] << " " << query_objects[i][5] << " " << query_objects[i][6] << std::endl;
    // }
    // query_objects_file.close();

    
    double sigma = slidegraph_sigma_;
    double epsilon = slidegraph_epsilon_;
    int min_num_pairs = slidegraph_num_inliners_;
    double matching_threshold = slidegraph_matching_threshold_;

    // print the params in ROS_ERROR_STREAM
    // ROS_ERROR_STREAM("[PlaceRecognition]: sigma is: " << sigma);
    // ROS_ERROR_STREAM("[PlaceRecognition]: epsilon is: " << epsilon);
    // ROS_ERROR_STREAM("[PlaceRecognition]: min_num_pairs is: " << min_num_pairs);
    // ROS_ERROR_STREAM("[PlaceRecognition]: matching_threshold is: " << matching_threshold);

    // convert the reference_objects and query_objects to semantic_clipper
    // argument format
    std::vector<std::vector<double>> reference_objects_vector;
    std::vector<std::vector<double>> query_objects_vector;
    for (int i = 0; i < reference_objects.size(); i++) {
      // check if the object has 0 in its coordinates, if so skip it since it is
      // not valid
      if (reference_objects[i][1] == 0.0 && reference_objects[i][2] == 0.0) {
        std::cerr <<
            "reference object has 0 in its coordinates, skipping it" << std::endl;
        continue;
      }
      std::vector<double> object;
      object.push_back(reference_objects[i][0]);
      object.push_back(reference_objects[i][1]);
      object.push_back(reference_objects[i][2]);
      object.push_back(reference_objects[i][3]);
      object.push_back(reference_objects[i][4]);
      object.push_back(reference_objects[i][5]);
      object.push_back(reference_objects[i][6]);
      reference_objects_vector.push_back(object);
  }
  for (int i = 0; i < query_objects.size(); i++) {
    // check if the object has 0 in its coordinates, if so skip it since it is not valid
    if (query_objects[i][1] == 0.0 && query_objects[i][2] == 0.0) {
      std::cerr << "[PlaceRecognition]: query object has 0 in its coordinates, skipping it" << std::endl;
      continue;
    }
    std::vector<double> object;
    object.push_back(query_objects[i][0]);
    object.push_back(query_objects[i][1]);
    object.push_back(query_objects[i][2]);
    object.push_back(query_objects[i][3]);
    object.push_back(query_objects[i][4]);
    object.push_back(query_objects[i][5]);
    object.push_back(query_objects[i][6]);
    query_objects_vector.push_back(object);
  }
  
  bool found = false;
  // make sure we have at least slidegraph_min_num_map_objects_to_start_ objects to do the matching in both the reference and query
  if (reference_objects_vector.size() >= slidegraph_min_num_map_objects_to_start_ && query_objects_vector.size() >= slidegraph_min_num_map_objects_to_start_) {
    // ROS_WARN_STREAM("[PlaceRecognition]: reference_objects_vector size is: " << reference_objects_vector.size() << " and query_objects_vector size is: " << query_objects_vector.size());
    std::cerr << "Calling CLIPPER for inter loop closure, if anything bad happens, look into that piece of code..." << std::endl;
    found = semantic_clipper::run_semantic_clipper(reference_objects_vector, query_objects_vector, tfFromQueryToRef, sigma, epsilon, min_num_pairs, matching_threshold);
    // get the inverse of the transformation matrix
    tfFromQueryToRef = tfFromQueryToRef.inverse();
  } else {
    std::cerr << "[PlaceRecognition]: Not enough objects to start the place recognition,  slidegraph_min_num_map_objects_to_start_ is set as " << slidegraph_min_num_map_objects_to_start_ << " but the reference_objects_vector size is: " << reference_objects_vector.size() << " and query_objects_vector size is: " << query_objects_vector.size() << std::endl;
  }  
  return found;
}
#endif

void PlaceRecognition::solveLSQ(
    const std::vector<Eigen::Vector3d> &map_objects_matched_out,
    const std::vector<Eigen::Vector3d> &detection_objects_matched_out,
    std::vector<double> &xyzyaw_out, Eigen::Matrix4d &transform_out) {
  Eigen::MatrixXd map_objects_matched_out_matrix =
      Eigen::MatrixXd::Zero(map_objects_matched_out.size(), 3); 
  Eigen::MatrixXd detection_objects_matched_out_matrix =
      Eigen::MatrixXd::Zero(detection_objects_matched_out.size(), 3);  
  for (int i = 0; i < map_objects_matched_out.size(); i++) {
    map_objects_matched_out_matrix(i, 0) = map_objects_matched_out[i][0];
    map_objects_matched_out_matrix(i, 1) = map_objects_matched_out[i][1];
    map_objects_matched_out_matrix(i, 2) = map_objects_matched_out[i][2];
  }
  for (int i = 0; i < detection_objects_matched_out.size(); i++) {
    detection_objects_matched_out_matrix(i, 0) =
        detection_objects_matched_out[i][0];
    detection_objects_matched_out_matrix(i, 1) =
        detection_objects_matched_out[i][1];
    detection_objects_matched_out_matrix(i, 2) =
        detection_objects_matched_out[i][2];
  }

  // Find the centroids of the source and target points
  Eigen::Vector3d centroidSource =
      detection_objects_matched_out_matrix.colwise().mean();
  Eigen::Vector3d centroidTarget =
      map_objects_matched_out_matrix.colwise().mean();

  // transpose the source and target points
  detection_objects_matched_out_matrix.transposeInPlace();
  map_objects_matched_out_matrix.transposeInPlace();

  // Center the source and target points
  Eigen::MatrixXd centeredSource =
      detection_objects_matched_out_matrix.colwise() - centroidSource;
  Eigen::MatrixXd centeredTarget =
      map_objects_matched_out_matrix.colwise() - centroidTarget;

  // Compute the cross-covariance matrix H
  Eigen::Matrix3d H = centeredSource * centeredTarget.transpose();

  // Perform SVD on H
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      H, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Compute the rotation matrix R
  Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

  if (R.determinant() < 0) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d V = svd.matrixV();
    V.col(2) = -V.col(2);
    R = V * svd.matrixU().transpose();
  }

  // Compute the translation vector t
  Eigen::Vector3d t = centroidTarget - R * centroidSource;
  std::cerr << "[PlaceRecognition]: The translation vector in solveLSQ is: " << t.transpose() << std::endl;
  transform_out = Eigen::Matrix4d::Identity();
  transform_out.block<3, 3>(0, 0) = R;
  transform_out.block<3, 1>(0, 3) = t;
  getxyzYawfromTF(transform_out, xyzyaw_out);
}

void PlaceRecognition::getxyzYawfromTF(const Eigen::Matrix4d &tf,
                                       std::vector<double> &xyzYaw) {
  Eigen::Vector3d translation = tf.block<3, 1>(0, 3);
  Eigen::Matrix3d rotation = tf.block<3, 3>(0, 0);
  // get the yaw angle
  double yaw = atan2(rotation(1, 0), rotation(0, 0));
  double roll = atan2(rotation(2, 1), rotation(2, 2));
  double pitch = atan2(-rotation(2, 0), sqrt(rotation(2, 1) * rotation(2, 1) +
                                             rotation(2, 2) * rotation(2, 2)));
  // put the x y z yaw into the vector
  xyzYaw.push_back(translation[0]);
  xyzYaw.push_back(translation[1]);
  xyzYaw.push_back(translation[2]);
  xyzYaw.push_back(yaw);
}

Eigen::Vector2d PlaceRecognition::getCentroid(
    const std::vector<Eigen::Vector7d> &objects) {
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (int i = 0; i < objects.size(); i++) {
    centroid[0] += objects[i][1];
    centroid[1] += objects[i][2];
  }
  centroid /= objects.size();
  return centroid;
}

Eigen::Vector2d PlaceRecognition::getMapBoundaries(
    const std::vector<Eigen::Vector7d> &objects) {
  double max_x = 0;
  double max_y = 0;
  for (int i = 0; i < objects.size(); i++) {
    max_x = std::max(max_x, std::abs(objects[i][1]));
    max_y = std::max(max_y, std::abs(objects[i][2]));
  }
  Eigen::Vector2d boundaries(max_x, max_y);
  return boundaries;
}

bool PlaceRecognition::findTransformation(
    const std::vector<Eigen::Vector7d> &reference_objects_input,
    const std::vector<Eigen::Vector7d> &query_objects_input,
    std::vector<double> &xyz_yaw_out, Eigen::Matrix4d &transform_out) {
  // initialize the two to be zeros
  Eigen::Vector2d centroid_reference = Eigen::Vector2d::Zero();
  Eigen::Vector2d centroid_query = Eigen::Vector2d::Zero();
  std::vector<Eigen::Vector7d> reference_objects;
  std::vector<Eigen::Vector7d> query_objects;
  std::cout <<
      "[findTransformation] running INTER (multi-robot) loop closure, doing "
      "preprocessing to zero center the maps and reduce search region" << std::endl;
  // preprocess step to reduce search region by centering the
  // reference_objects and query_objects to make their centroid aligned on XY
  // plane
  centroid_reference = getCentroid(reference_objects_input);
  centroid_query = getCentroid(query_objects_input);
  // shift the reference_objects to align the centroid with origin
  reference_objects = reference_objects_input;
  for (int i = 0; i < reference_objects_input.size(); i++) {
    reference_objects[i][1] -= centroid_reference[0];
    reference_objects[i][2] -= centroid_reference[1];
  }
  // shift the query_objects to align the centroid with origin
  query_objects = query_objects_input;
  for (int i = 0; i < query_objects_input.size(); i++) {
    query_objects[i][1] -= centroid_query[0];
    query_objects[i][2] -= centroid_query[1];
  }

  // get the max bounding region of the reference_objects and query_objects
  Eigen::Vector2d boundaries_reference = getMapBoundaries(reference_objects);
  Eigen::Vector2d boundaries_query = getMapBoundaries(query_objects);

  double max_x_region =
      std::max(boundaries_reference[0], boundaries_query[0]);
  double max_y_region =
      std::max(boundaries_reference[1], boundaries_query[1]);

  // check if yaw search is needed
  if (!disable_yaw_search_) {
    // since there may be yaw, we need to take the max of the two
    double max_region_size = std::max(max_x_region, max_y_region);
    max_x_region = max_region_size;
    max_y_region = max_region_size;
  }

  // instead of using hard-coded half range values, we use the auto computed
  // max_x_region and max_y_region dilate the map to consider partial overlap
  match_x_half_range_ = max_x_region * dilation_factor_;
  match_y_half_range_ = max_y_region * dilation_factor_;

  // print the centroid difference
  Eigen::Vector2d centroid_diff = centroid_reference - centroid_query;
  std::cerr << "[PlaceRecognition]: centroid diff between the two maps in x and y is: "
                  << centroid_diff[0] << " " << centroid_diff[1] << std::endl;

  std::cerr <<
      "Auto computed the bounding (candidate loop closure) region of the two "
      "maps, the half range in x and y is: "
      << match_x_half_range_ << " " << match_y_half_range_
      << " (with dilation factor: " << dilation_factor_ << ")" << std::endl;

  Eigen::Matrix3d R_t_out;
  int best_num_inliers_out = 0;
  std::vector<Eigen::Vector4d> map_objects_matched_out;
  std::vector<Eigen::Vector4d> detection_objects_matched_out;

  // the input of matchmaps is vector7d label x y z dim1 dim2 dim3
  // the output map_objects_matched_out from matchmaps is vector 3d label x y
  MatchMaps(reference_objects, query_objects, R_t_out, best_num_inliers_out,
            map_objects_matched_out, detection_objects_matched_out);

  // check if the best_num_inliers_out is less than
  // min_num_inliers_for_valid_closure_
  // UPDATE: NO LONGER USING THE OVERLAP THRESHOLD SINCE IT IS NON INTUITIVE TO THE USER
  // double min_num_inliners =
  //     min_loop_closure_overlap_percentage_ *
  //     static_cast<double>(
  //         std::min(reference_objects.size(), query_objects.size()));
  int min_num_inliners = min_num_inliers_;

  // make min_num_inliners at least one to avoid division by zero
  // min_num_inliners = std::max(min_num_inliners, 1.0);
  if (best_num_inliers_out < min_num_inliners) {
    std::cerr << "[PlaceRecognition]: Not enough inliers found, best_num_inliers_out is: "
                    << best_num_inliers_out << std::endl;
    std::cerr <<
        "min_num_inliners is: "
        << min_num_inliners
        << " YOU CAN TUNE THE PARAMETER IN THE YAML FILE (min_num_inliers)" << std::endl;
    return false;
  } else {
    std::cout << "[PlaceRecognition]: Enough inliers found, best_num_inliers_out is: "
                    << best_num_inliers_out << std::endl;
    std::cout <<
        "actual overlap percentage is: "
        << static_cast<double>(best_num_inliers_out) /
               static_cast<double>(
                   std::min(reference_objects.size(), query_objects.size())) << std::endl;
  }

  if (!use_lsq) {
    Eigen::Matrix4d transform_out_raw = Eigen::Matrix4d::Identity();
    // take the upper 2x2 of R_t which is the yaw rotation matrix
    Eigen::Matrix2d Rot_yaw = R_t_out.block<2, 2>(0, 0);
    transform_out_raw.block<2, 2>(0, 0) = Rot_yaw;
    transform_out_raw.block<3, 1>(0, 3) =
        Eigen::Vector3d(R_t_out(0, 2), R_t_out(1, 2), 0);

    // add centroid_diff back (only x and y since we don't estimate Z in this
    // case)
    transform_out = PlaceRecognition::revertCentroidShift(
        transform_out_raw, centroid_reference, centroid_query);

    // get the xyzYaw from the transform_out
    getxyzYawfromTF(transform_out, xyz_yaw_out);

    // print error message, to check if this pass test cases!
    return true;
  } else {
    std::vector<Eigen::Vector3d> map_objects_matched;
    std::vector<Eigen::Vector3d> detection_objects_matched;
    // remove the label from map_objects_matched_out and
    // detection_objects_matched_out and put them into map_objects_matched and
    // detection_objects_matched
    for (int i = 0; i < map_objects_matched_out.size(); i++) {
      Eigen::Vector3d cur_map_object(map_objects_matched_out[i][1],
                                     map_objects_matched_out[i][2],
                                     map_objects_matched_out[i][3]);
      map_objects_matched.push_back(cur_map_object);
    }
    for (int i = 0; i < detection_objects_matched_out.size(); i++) {
      Eigen::Vector3d cur_detection_object(detection_objects_matched_out[i][1],
                                           detection_objects_matched_out[i][2],
                                           detection_objects_matched_out[i][3]);
      detection_objects_matched.push_back(cur_detection_object);
    }

    // for inter loop closure instead of manually call revertCentroidShift, we
    // first shift the map_objects_matched and detection_objects_matched back
    // to their original position before least square fitting
    for (int i = 0; i < map_objects_matched.size(); i++) {
      map_objects_matched[i][0] += centroid_reference[0];
      map_objects_matched[i][1] += centroid_reference[1];
    }
    for (int i = 0; i < detection_objects_matched.size(); i++) {
      detection_objects_matched[i][0] += centroid_query[0];
      detection_objects_matched[i][1] += centroid_query[1];
    }

    // no need to revert the centroid shift since the inputs of solveLSQ
    // function already take care of it 
    solveLSQ(map_objects_matched, detection_objects_matched, xyz_yaw_out,
             transform_out);
    return true;
  }
}

Eigen::Matrix4d PlaceRecognition::revertCentroidShift(
    const Eigen::Matrix4d &tf, const Eigen::Vector2d &centroid_reference,
    const Eigen::Vector2d &centroid_query) {
  // H_1^2 = H_1^1(shifted) * H_1(shifted)^2(shifted) * H_2(shifted)^2
  // we get H_1^1(shifted) based on the centroid_reference
  // we get H_1(shifted)^2(shifted) from tf
  // we get H_2(shifted)^2 from centroid_query
  Eigen::Matrix4d H_1_1_shifted = Eigen::Matrix4d::Identity();
  // translation part is centroid_reference
  H_1_1_shifted(0, 3) = centroid_reference[0];
  H_1_1_shifted(1, 3) = centroid_reference[1];
  // same for H_2shifted^2
  Eigen::Matrix4d H_2_shifted_2 = Eigen::Matrix4d::Identity();
  // need to invert since it is from shifted back to original, since rot is
  // identity, just need to negate the translation
  H_2_shifted_2(0, 3) = -centroid_query[0];
  H_2_shifted_2(1, 3) = -centroid_query[1];
  // H_1^2 = H_1^1(shifted) * H_1(shifted)^2(shifted) * H_2(shifted)^2
  Eigen::Matrix4d tf_out = H_1_1_shifted * tf * H_2_shifted_2;
  return tf_out;
}
