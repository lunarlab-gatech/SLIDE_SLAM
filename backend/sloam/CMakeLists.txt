cmake_minimum_required(VERSION 3.16)
project(sloam)

set(CMAKE_CXX_STANDARD 17)
list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(BUILD_TESTING "Build tests" Off)
option(USE_CLIPPER "Use clipper" ON)

set(CC_TARGET_PREFIX sloam)
include(CMakeHelpers)

find_package(catkin REQUIRED COMPONENTS
  sloam_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_geometry_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_eigen
  tf2
  tf
  roscpp
  actionlib
  actionlib_msgs
)

find_package(Boost REQUIRED system serialization)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(GTest REQUIRED)

find_package(yaml-cpp REQUIRED)

if(NOT yaml-cpp_FOUND)
  message(FATAL_ERROR "Yaml-cpp is not found
  Please install yaml-cpp using:
  
  sudo apt-get install libyaml-cpp-dev")
endif()

set(CLIPPER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/clipper_semantic_object")
set(CLIPPER_VALUE 0)
if (USE_CLIPPER)
  add_subdirectory(${CLIPPER_DIR})
  set(CLIPPER_VALUE 1)
endif()

catkin_package()

cc_library(
  NAME log
  DEPS fmt::fmt-header-only
  INTERFACE
)

cc_library(
    NAME base
    DEPS ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES}# ${ONNX_RUNTIME_LIB}
    INCS include/helpers ${Boost_INCLUDE_DIR} ${Sophus_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} # ${ONNX_RUNTIME_SESSION_INCLUDE_DIRS}
    INTERFACE
)

cc_library(
  NAME objects
  DEPS sloam_base gtsam
  INCS include/objects
  SRCS "src/objects/plane.cpp" "src/objects/cylinder.cpp" "src/objects/cube.cpp" "src/objects/ellipsoid.cpp"
)

cc_library(
    NAME viz
    DEPS sloam_log sloam_base sloam_objects yaml-cpp
    INCS include/viz 
    SRCS "src/viz/vizTools.cpp"
)

cc_library(
  NAME map
  INCS include/core include/objects
  SRCS "src/core/cylinderMapManager.cpp" "src/core/cubeMapManager.cpp" "src/core/ellipsoidMapManager.cpp"
  DEPS sloam_log sloam_base sloam_objects
)

cc_library(
  NAME factorgraph
  DEPS Eigen3::Eigen gtsam ${Boost_LIBRARIES} sloam_base sloam_objects sloam_map
  INCS ${GTSAM_LIBRARIES} ${Boost_INCLUDE_DIR} ${Sophus_INCLUDE_DIRS} include/factorgraph include/core
  SRCS "src/factorgraph/cylinderFactor.cpp" "src/factorgraph/cubeFactor.cpp" "src/factorgraph/graph.cpp" "src/factorgraph/graphWrapper.cpp" 
)

cc_library(
  NAME core
  INCS include/core ${CLIPPER_DIR}/include
  SRCS "src/core/sloam.cpp" "src/core/robot.cpp" "src/core/databaseManager.cpp" "src/core/place_recognition.cpp"
  DEPS sloam_log sloam_base sloam_objects sloam_viz sloam_map sloam_factorgraph Eigen3::Eigen gtsam ${catkin_LIBRARIES} -ldw
  DEFINES USE_CLIPPER=${CLIPPER_VALUE}
)

if (USE_CLIPPER)
  target_link_libraries(${CC_TARGET_PREFIX}_core PRIVATE semantic_clipper)
endif()

cc_binary(
  NAME node
  SRCS "src/core/sloamNode.cpp" "src/core/inputNode.cpp"
  DEPS sloam_log sloam_base sloam_objects sloam_core sloam_viz sloam_factorgraph gtsam ${catkin_LIBRARIES} -ldw #sloam_loop 
  # DEPS sloam_log sloam_base sloam_objects sloam_core sloam_viz sloam_segmentation sloam_factorgraph gtsam ${catkin_LIBRARIES} -ldw #sloam_loop 
)

cc_binary(
  NAME place_recognition_test
  SRCS "src/tests/place_recognition_test.cpp"
  DEPS sloam_base sloam_core)



if(BUILD_TESTING)
  enable_testing()

  cc_test(
  NAME inter_robot_factor_test
  SRCS "src/tests/inter_robot_factor_test.cpp"
  DEPS sloam_base sloam_objects sloam_core GTest::GTest)

# These test cases are not up to date, and thus won't work on current code.
#
# Example usage: ./sloam_ws/devel/lib/sloam/sloam_all_test --gtest_filter=CylinderTest* (optional filter)
# cc_test(
  # NAME all_sloam_test
  # SRCS "src/tests/run_all_test.cpp" "src/tests/plane_test.cpp" "src/tests/cylinder_test.cpp" "src/tests/core_test.cpp"
  # DEPS sloam_base sloam_objects sloam_core GTest::GTest)
# cc_test(
  # NAME fg_test
  # SRCS "src/tests/cylinder_factor_test.cpp"
  # DEPS sloam_base sloam_factorgraph GTest::GTest)
# cc_test(
    # NAME cube_fg_test
    # SRCS "src/tests/cube_factor_test.cpp"
    # DEPS sloam_base sloam_factorgraph GTest::GTest)
endif()