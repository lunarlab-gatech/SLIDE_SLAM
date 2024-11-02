
// Load data from ROS published point cloud and accumulated point cloud
// #include "cnpy.h"

#include <dirent.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <iostream>
#include <vector>

using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::filesystem::recursive_directory_iterator;

// This is going to be replaced by directly subscribing to ROS topic

// http :  // wiki.ros.org/pcl/Overview

int main() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  string path = "/opt/bags/sloam/segmentation_results/point_clouds/world_frame";

  for (const auto &file : recursive_directory_iterator(path)) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file.path(), *cloud) == -1) {
      PCL_ERROR("Couldn't read file \n");
      continue;
      //   return (-1);
    }

    // std::cout << "Loaded " << cloud->width * cloud->height
    //           << " data points from test_pcd.pcd with the following fields: "
    //           << std::endl;

    int point_counter = 0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    for (const auto &point : *cloud) {
      // std::cout << "    " << point.x << " " << point.y << " " << point.z
      //           << std::endl;
      if (point.x != 0) {
        sum_x += point.x;
        sum_y += point.y;
        sum_z += point.z;

        point_counter++;
      }
    }
    double mean_x = sum_x / point_counter;
    double mean_y = sum_y / point_counter;
    double mean_z = sum_z / point_counter;

    std::cout << " MEAN X    " << static_cast<int>(mean_x) << " MEAN Y "
              << static_cast<int>(mean_y) << " MEAN Z "
              << static_cast<int>(mean_z) << "Points " << point_counter
              << std::endl;
  }
}
