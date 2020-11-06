cmake_minimum_required(VERSION 3.5)
project(fcl-sampling)
SET(CMAKE_BUILD_TYPE "Release")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(fcl REQUIRED)
include_directories(${FCL_INCLUDE_DIRS})

find_package(ccd REQUIRED)
include_directories(${CCD_INCLUDE_DIR})

set(SDFAST  "model/sdfast/")
set(CODES   "model/")


include_directories (${CODES}/include/)
include_directories (${SDFAST}/include/)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) 

file(GLOB MODEL_SOURCES "${SDFAST}/*.c")
file(GLOB EXTRA_SOURCES "${CODES}/src/*.cpp")
#file(GLOB STACK_SOURCES "main.cpp")
set(SOURCES_main main.cpp mesh_utils.cpp mesh_utils.h)


#set(CMAKE_CXX_FLAGS "-std=gnu++1z")
set(CMAKE_CXX_FLAGS "-std=c++11 -fopenmp -o3")


add_library (Model ${MODEL_SOURCES} ${EXTRA_SOURCES})
add_executable (main ${SOURCES_main})

TARGET_LINK_LIBRARIES(main -lstdc++fs Model fcl ccd)

