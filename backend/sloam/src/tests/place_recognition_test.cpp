#include <place_recognition.h>

// Either use randomly generated objects or load objects from your own test data
bool use_generated_objects = false;
std::string file_path_robot0 = "/home/sam/slideslam_ws/src/SLIDE_SLAM/backend/sloam/clipper_semantic_object/examples/data/robot0Map_indoor.txt";
std::string file_path_robot1 = "/home/sam/slideslam_ws/src/SLIDE_SLAM/backend/sloam/clipper_semantic_object/examples/data/robot1Map_indoor.txt";
bool visualize_matching_results = true;

// generate a set of 50 objects of various classes and span over a 100 m by 100
// m area
std::vector<Eigen::Vector7d> generateObjects() {
  std::vector<Eigen::Vector7d> objects;
  for (int i = 0; i < 50; i++) {
    Eigen::Vector7d object;
    object[0] = i % 10;                        // label
    object[1] = 100 * (rand() % 100) / 100.0;  // x
    object[2] = 100 * (rand() % 100) / 100.0;  // y
    object[3] = 0;                             // z
    // make dim random from 0.5 to 2.5 m
    object[4] = 0.5 + 2.0 * (rand() % 100) / 100.0;  // dim_x
    object[5] = 0.5 + 2.0 * (rand() % 100) / 100.0;  // dim_y
    object[6] = 0.5 + 2.0 * (rand() % 100) / 100.0;  // dim_z
    objects.push_back(object);
  }
  return objects;
}

// transform the objects given a transformation matrix formed from x, y, yaw
std::vector<Eigen::Vector7d> transformObjects(
    const std::vector<Eigen::Vector7d> &objects,
    const std::vector<double> &xyzYaw) {
  std::vector<Eigen::Vector7d> transformed_objects;
  for (const auto &object : objects) {
    Eigen::Vector7d transformed_object;
    transformed_object[0] = object[0];  // label
    transformed_object[1] = object[1] * cos(xyzYaw[3]) -
                            object[2] * sin(xyzYaw[3]) + xyzYaw[0];  // x
    transformed_object[2] = object[1] * sin(xyzYaw[3]) +
                            object[2] * cos(xyzYaw[3]) + xyzYaw[1];  // y
    transformed_object[3] = object[3] + xyzYaw[2];                   // z
    // dim_x, dim_y, dim_z remain the same
    transformed_object[4] = object[4];
    transformed_object[5] = object[5];
    transformed_object[6] = object[6];
    transformed_objects.push_back(transformed_object);
  }
  return transformed_objects;
}

// main test function:
// 1. generate two sets of objects
// 2. transform the second set of objects
// 3. run the place recognition algorithm
// 4. check if the loop closure is found
int main() {
  // ros init stuff
  int argc = 0;
  char **argv = NULL;

  ros::init(argc, argv, "unit_test_place_recognition");
  // add a namespace "sloam" to the node handle
  std::string ns = "sloam";
  ros::NodeHandle nh(ns);

  while (ros::ok()) {
    // keep this in while loop so as to be able to kill with ctrl+c


    // creat a place recognition object
    PlaceRecognition place_recoger(nh);

    // turn on visualization
    place_recoger.visualize_matching_results = visualize_matching_results;

    if (!use_generated_objects){
      // load objects from file
      std::vector<Eigen::Vector7d> reference_objects;
      std::vector<Eigen::Vector7d> query_objects;
      std::string file_path;
      // only test inter-loop closure
      place_recoger.inter_loop_closure = true;
      // load objects from file
      file_path = file_path_robot0;
      std::ifstream file(file_path);
      if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
          std::istringstream iss(line);
          Eigen::Vector7d object;
          for (int i = 0; i < 7; i++) {
            iss >> object[i];
          }
          reference_objects.push_back(object);
        }
        file.close();
      } else {
        ROS_ERROR("Unable to open file %s", file_path.c_str());
      }
      // load objects from file
      file_path = file_path_robot1;
      file.open(file_path);
      if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
          std::istringstream iss(line);
          Eigen::Vector7d object;
          for (int i = 0; i < 7; i++) {
            iss >> object[i];
          }
          query_objects.push_back(object);
        }
        file.close();
      } else {
        ROS_ERROR("Unable to open file %s", file_path.c_str());
      }
      // run through the place recognition algorithm
      Eigen::Matrix4d tfFromQueryToRef;
      // time the function
      ros::Time start = ros::Time::now();
      // print out the reference objects and query objects
      std::cout << "Reference objects: " << std::endl;
      for (const auto &object : reference_objects) {
        std::cout << object.transpose() << std::endl;
      }
      std::cout << "Query objects: " << std::endl;
      for (const auto &object : query_objects) {
        std::cout << object.transpose() << std::endl;
      }
      bool closure_found = place_recoger.findInterLoopClosure(
          reference_objects, query_objects, tfFromQueryToRef);
      ros::Time end = ros::Time::now();
      ROS_WARN("Time taken for place recognition: %f",
                (end - start).toSec());
      // check if the loop closure is found
      if (closure_found) {
        std::cout << "Loop closure found!" << std::endl;
        // print the transformation matrix
        std::cout << tfFromQueryToRef << std::endl;
        std::vector<double> xyzYawOut;
        place_recoger.getxyzYawfromTF(tfFromQueryToRef, xyzYawOut);
        std::cout << "x out: " << xyzYawOut[0]
                  << ", y out: " << xyzYawOut[1]
                  << ", yaw out: " << xyzYawOut[3] << std::endl;
      } else {
        std::cout << "Loop closure not found!" << std::endl;
      }
    } else {
    // generate two sets of objects
    std::vector<Eigen::Vector7d> reference_objects = generateObjects();
    // iterate through different values of x_transform, y_transform
    for (int lc_type = 0; lc_type < 2; lc_type++) {
      double x_starting_transform;
      double y_starting_transform;
      double yaw_starting_transform_deg;
      if (lc_type == 0) {
        place_recoger.inter_loop_closure = true;
        ROS_INFO("Testing inter-loop closure");
        x_starting_transform = 5.25;
        y_starting_transform = 5.25;
        yaw_starting_transform_deg = 90;
      } else {
        place_recoger.inter_loop_closure = false;
        ROS_INFO("Testing intra-loop closure");
        x_starting_transform = 2.25;
        y_starting_transform = 1.5;
        yaw_starting_transform_deg = 10;
      }
      int iterations_to_test = 1;

      for (int cur_iter = 0; cur_iter < iterations_to_test; cur_iter++) {
        // transform the second set of objects
        double x_transform = x_starting_transform + 2 * cur_iter;
        double y_transform = y_starting_transform + 2 * cur_iter;
        double yaw_transform_deg = yaw_starting_transform_deg + 10 * cur_iter;
        double yaw_transform = yaw_transform_deg * M_PI / 180.0;
        std::vector<double> xyzYawReverse = {x_transform, y_transform, 0,
                                             yaw_transform};
        std::vector<Eigen::Vector7d> query_objects =
            transformObjects(reference_objects, xyzYawReverse);
        // add some noise (up to 0.3 m) to the transformed objects position and
        // dimension, drop random 20 percent of the objects
        double noise_mag = 0.0;  // 0.3;
        for (int i = 0; i < query_objects.size(); i++) {
          // drop 20 percent of the objects
          if (rand() % 100 < 20) {
            query_objects.erase(query_objects.begin() + i);
            i--;
            continue;
          }
          // add noise to the position
          query_objects[i][1] += noise_mag * (rand() % 100) / 100.0;
          query_objects[i][2] += noise_mag * (rand() % 100) / 100.0;
          // add noise to the dimension
          query_objects[i][4] += noise_mag * (rand() % 100) / 100.0;
          query_objects[i][5] += noise_mag * (rand() % 100) / 100.0;
          query_objects[i][6] += noise_mag * (rand() % 100) / 100.0;
        }

        // iterate through place_recoger.use_lsq = {true, false}
        for (int inner_iter = 0; inner_iter < 2; inner_iter++) {
          if (inner_iter == 0) {
            place_recoger.use_lsq = true;
          } else {
            place_recoger.use_lsq = false;
          }
          // run the place recognition algorithm
          Eigen::Matrix4d tfFromQueryToRef;
          // time the function
          ros::Time start = ros::Time::now();

          bool closure_found;
          if (place_recoger.inter_loop_closure) {
            // inter-loop closure
            closure_found = place_recoger.findInterLoopClosure(
                reference_objects, query_objects, tfFromQueryToRef);
          } else {
            // intra-loop closure
            // query pose is SE3()
            SE3 query_pose = SE3();
            SE3 candidate_pose = SE3();
            // reference objects are submap and query objects are current measurements 
            closure_found = place_recoger.findIntraLoopClosure(
                query_objects, reference_objects, query_pose, candidate_pose,
                tfFromQueryToRef);
            // ROS_WARN_STREAM("TODO: intra loop closure test to be completed!");
            // closure_found = false;
          }

          ros::Time end = ros::Time::now();
          ROS_WARN("Time taken for place recognition: %f",
                   (end - start).toSec());

          // check if the loop closure is found
          if (closure_found) {
            std::cout << "Loop closure found!" << std::endl;
            // print the transformation matrix
            std::cout << tfFromQueryToRef << std::endl;
            std::vector<double> xyzYawOut;
            place_recoger.getxyzYawfromTF(tfFromQueryToRef, xyzYawOut);
            std::vector<double> xyzYawExpected = {
                -x_transform * cos(yaw_transform) -
                    y_transform * sin(yaw_transform),
                x_transform * sin(yaw_transform) -
                    y_transform * cos(yaw_transform),
                0, -yaw_transform};
            std::cout << "x expected: " << xyzYawExpected[0]
                      << ", y expected: " << xyzYawExpected[1]
                      << ", yaw expected: " << xyzYawExpected[3] << std::endl;
            std::cout << "x out: " << xyzYawOut[0]
                      << ", y out: " << xyzYawOut[1]
                      << ", yaw out: " << xyzYawOut[3] << std::endl;
            // cast the difference to double, keep only 4 decimal places
            double diff_x =
                round((xyzYawExpected[0] - xyzYawOut[0]) * 10000) / 10000;
            double diff_y =
                round((xyzYawExpected[1] - xyzYawOut[1]) * 10000) / 10000;
            double diff_yaw_rad =
                round((xyzYawExpected[3] - xyzYawOut[3]) * 10000) / 10000;
            // convert the yaw to degree
            double diff_yaw = diff_yaw_rad * 180.0 / M_PI;
            // check if yaw is close to 360 or -360, take the smaller difference
            if (diff_yaw > 180) {
              diff_yaw = diff_yaw - 360;
            } else if (diff_yaw < -180) {
              diff_yaw = diff_yaw + 360;
            }

            // if x or y diff is more than 50 cm or if yaw is more than 3
            // degrees, ROS_ERROR, otherwise ROS_INFO
            if (fabs(diff_x) > 0.5 || fabs(diff_y) > 0.5 ||
                fabs(diff_yaw) > 3) {
              ROS_ERROR("Difference in x: %f, y: %f, yaw (in degrees): %f",
                        diff_x, diff_y, diff_yaw);
            } else {
              ROS_INFO("Difference in x: %f, y: %f, yaw (in degrees): %f",
                       diff_x, diff_y, diff_yaw);
            }
            if (place_recoger.use_lsq) {
              std::cout << "Using LSQ" << std::endl;
            } else {
              std::cout << "Not using LSQ" << std::endl;
            }
          } else {
            std::cout << "Loop closure not found!" << std::endl;
          }

          std::cout
              << "++++++++++++++++++++++++++++ Current test case finished! "
                 "++++++++++++++++++++++++++++ "
              << std::endl;
        }
      }
    }
    }

    ros::spinOnce();
    ros::shutdown();
  }
  return 0;
}