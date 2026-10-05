# Builds golden_driver inside the xurobotics/slide-slam image, against a pristine `original` clone and its catkin build.
cmake_minimum_required(VERSION 3.16)
project(slideslam_golden_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(SLOAM_SRC "" CACHE PATH "backend/sloam directory of the pristine original clone")
set(SLOAM_LIB_DIRS "" CACHE STRING "Directories holding the catkin-built sloam_core, semantic_clipper, triangulation and clipper libraries")

find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs pcl_ros)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)

foreach(lib sloam_core semantic_clipper triangulation clipper)
  find_library(${lib}_LIB ${lib} PATHS ${SLOAM_LIB_DIRS} NO_DEFAULT_PATH)
  if(NOT ${lib}_LIB)
    message(FATAL_ERROR "lib${lib}.so not found in SLOAM_LIB_DIRS='${SLOAM_LIB_DIRS}'")
  endif()
  list(APPEND SLOAM_LIBS ${${lib}_LIB})
endforeach()

add_executable(golden_driver golden_driver.cpp)
target_compile_definitions(golden_driver PRIVATE USE_CLIPPER=1)
target_compile_options(golden_driver PRIVATE -march=native -mavx -mfma)
target_include_directories(golden_driver PRIVATE
  ${SLOAM_SRC}/include/core ${SLOAM_SRC}/include/helpers ${SLOAM_SRC}/clipper_semantic_object/include
  /usr/local/include/libqhullcpp /usr/local/include/libqhull_r
  ${catkin_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(golden_driver ${SLOAM_LIBS} -Wl,--no-as-needed /usr/local/lib/libqhull_r.so ${catkin_LIBRARIES} ${OpenCV_LIBS})
string(REPLACE ";" ":" SLOAM_RPATH "${SLOAM_LIB_DIRS}")
set_target_properties(golden_driver PROPERTIES BUILD_RPATH "${SLOAM_RPATH}")
target_link_options(golden_driver PRIVATE "-Wl,-rpath-link,${SLOAM_RPATH}")
