cmake_minimum_required(VERSION 3.10)
project(clipper VERSION 0.2.4)

set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose build type." FORCE)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

if(NOT clipper_FIND_QUIETLY)
  message(STATUS "CLIPPER v${PROJECT_VERSION}")
endif()

###############################################################################
# Options
###############################################################################

option(CLIPPER_BUILD_BINDINGS_PYTHON "Build Python bindings" OFF)
option(CLIPPER_BUILD_BINDINGS_MATLAB "Build MATLAB bindings" OFF)
option(CLIPPER_BUILD_TESTS "Build testsuite" OFF)
option(CLIPPER_BUILD_BENCHMARKS "Build benchmarks" OFF)
option(CLIPPER_ENABLE_MKL "Use MKL with Eigen" OFF)
option(CLIPPER_ENABLE_BLAS "Use BLAS with Eigen" OFF) # apt install libopenblas-dev
option(CLIPPER_ENABLE_MAXCLIQUE "Use PMC lib for solveAsMaximumClique" OFF)
option(CLIPPER_ENABLE_SCS_SDR "Use SCS lib for solveAsMSRCSDR" OFF)

if(CLIPPER_ENABLE_MKL AND CLIPPER_ENABLE_BLAS)
  message(FATAL_ERROR "Cannot enable both MKL and BLAS (prefer MKL)")
endif()

###############################################################################
# Dependencies
###############################################################################

##########Triangulaton library##############
find_package(Boost REQUIRED)
INCLUDE_DIRECTORIES(/usr/local/include/libqhullcpp)
INCLUDE_DIRECTORIES(/usr/local/include/libqhull_r)
SET(qhullLibs qhullcpp qhull_r)
LINK_DIRECTORIES(/usr/local/lib)

##########Triangulaton library##############


include(FetchContent)

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen Version: ${EIGEN3_VERSION_STRING} ${EIGEN3_VERSION}")

find_package(OpenMP)

# Give preference to MKL
if(CLIPPER_ENABLE_MKL)
    find_package(MKL)
    if(MKL_FOUND)
        message(STATUS "MKL found at: ${MKL_LIBRARIES}")
    else()
        message(STATUS "MKL not found.")
    endif()
endif()

if(CLIPPER_ENABLE_BLAS)
    # OpenBLAS is generally better than ATLAS
    set(BLA_VENDOR OpenBLAS)
    find_package(BLAS)
    if(BLAS_FOUND)
        message(STATUS "BLAS found at: ${BLAS_LIBRARIES}")
    else()
        message(STATUS "BLAS not found.")
    endif()
endif()

if(CLIPPER_BUILD_BINDINGS_PYTHON)
  FetchContent_Declare(pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG        v2.9.2
  )
  FetchContent_MakeAvailable(pybind11)
endif()

if(CLIPPER_ENABLE_MAXCLIQUE)
    FetchContent_Declare(pmc
        GIT_REPOSITORY  https://github.com/jingnanshi/pmc
        GIT_TAG         16af470353a784352760ec2deea25fa7ed602450)
    FetchContent_MakeAvailable(pmc)
endif()

if(CLIPPER_ENABLE_SCS_SDR)
    include(FetchContent)
    FetchContent_Declare(scs
        GIT_REPOSITORY  https://github.com/cvxgrp/scs
        GIT_TAG         3.2.3)
    FetchContent_MakeAvailable(scs)
endif()

###############################################################################
# Targets
###############################################################################

find_package(clipper REQUIRED)

if(MKL_FOUND)
    target_include_directories(clipper PRIVATE ${MKL_INCLUDE_DIR})
    target_compile_definitions(clipper PRIVATE -DEIGEN_USE_MKL_ALL)
    target_link_libraries(clipper PRIVATE ${MKL_LIBRARIES})
elseif(BLAS_FOUND)
    target_compile_definitions(clipper PRIVATE -DEIGEN_USE_BLAS=1)
    target_link_libraries(clipper PRIVATE BLAS::BLAS)
endif()

if(CLIPPER_ENABLE_MAXCLIQUE)
    target_compile_definitions(clipper PUBLIC CLIPPER_HAS_PMC)
    target_link_libraries(clipper PRIVATE pmc)
endif()

if(CLIPPER_ENABLE_SCS_SDR)
    target_compile_definitions(clipper PUBLIC CLIPPER_HAS_SCS)
    target_link_libraries(clipper PRIVATE scs::scsdir)
endif()

##########Triangulaton target##############
add_library(triangulation SHARED)
target_sources(triangulation PRIVATE
  src/triangulation/observation.cpp)
target_include_directories(triangulation PRIVATE include ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(triangulation ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${qhullLibs})

##########Semantic Clipper target##############
add_library(semantic_clipper SHARED)
target_sources(semantic_clipper PRIVATE
  src/semantic_clipper.cpp)
target_link_libraries(semantic_clipper PRIVATE clipper triangulation ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
target_include_directories(semantic_clipper PRIVATE include ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

###############################################################################
# Extras
###############################################################################

if(CLIPPER_BUILD_BINDINGS_PYTHON)
    message(STATUS "Building Python bindings.")
    add_subdirectory(bindings/python)
endif()

if(CLIPPER_BUILD_BINDINGS_MATLAB)
    message(STATUS "Attempting to build MATLAB bindings.")
    add_subdirectory(bindings/matlab)
endif()

if(CLIPPER_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

if(CLIPPER_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()
