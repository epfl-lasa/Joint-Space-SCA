cmake_minimum_required(VERSION 2.8.3)
project(nn_model_eval LANGUAGES CXX)


SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


include_directories (${CMAKE_CURRENT_SOURCE_DIR})

## Declare a C++ library
add_library(nn_eval nn_eval.cpp)


