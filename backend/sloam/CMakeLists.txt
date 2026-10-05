cmake_minimum_required(VERSION 3.16)
project(sloam)

set(CMAKE_CXX_STANDARD 17)
list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(BUILD_TESTING "Build tests" Off)
option(USE_CLIPPER "Use clipper" ON)

set(CC_TARGET_PREFIX sloam)
include(CMakeHelpers)

find_package(Eigen3 REQUIRED)

set(CLIPPER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/clipper_semantic_object")
set(CLIPPER_VALUE 0)
if (USE_CLIPPER)
  add_subdirectory(${CLIPPER_DIR})
  set(CLIPPER_VALUE 1)
endif()

cc_library(
  NAME core
  INCS include/core ${CLIPPER_DIR}/include
  SRCS "src/core/place_recognition.cpp"
  DEPS Eigen3::Eigen
  DEFINES USE_CLIPPER=${CLIPPER_VALUE}
)

if (USE_CLIPPER)
  target_link_libraries(${CC_TARGET_PREFIX}_core PRIVATE semantic_clipper)
endif()

# TESTS
# if(BUILD_TESTING)
# enable_testing()
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
