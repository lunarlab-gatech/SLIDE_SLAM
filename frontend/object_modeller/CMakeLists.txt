cmake_minimum_required(VERSION 3.16)
project(object_modeller)



find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs sloam_msgs
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(${PROJECT_NAME}_cylinder src/cylinder_modeller.cpp)
target_link_libraries(${PROJECT_NAME}_cylinder ${catkin_LIBRARIES} ${PCL_LIBRARIES} -ldw)
target_include_directories(${PROJECT_NAME}_cylinder PUBLIC include ${PCL_INCLUDE_DIRS})
