# Copyright: 2012 iCub Facility, Istituto Italiano di Tecnologia
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(iCubWalking)
find_package(YARP)
find_package(fcl REQUIRED)
find_package(ccd REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

set(SDFAST "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/sdfast/")
set(CODES  "${CMAKE_CURRENT_SOURCE_DIR}/../")
set(IK 	   "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/IKCG_cvxgen/")
set(WRAPPER "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/wrapper/")
set(LPVDS  "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/lpvDS-lib/")

include_directories (${CODES}/include/)
include_directories (${LPVDS}/include/)
include_directories (${WRAPPER}/include/)
include_directories (${IK})
include_directories(${FCL_INCLUDE_DIRS})
include_directories(${CCD_INCLUDE_DIR})

file(GLOB MODEL_SOURCES "${SDFAST}/*.c")
file(GLOB IK_SOURCES 	"${IK}/*.c")
file(GLOB LPVDS_SOURCES "${LPVDS}/src/lpvDS/*.cpp")
file(GLOB WRAPPER_SOURCES "${WRAPPER}/src/*.cpp")
file(GLOB EXTRA_SOURCES "${CODES}/src/*.cpp")
file(GLOB STACK_SOURCES "../main.cpp")

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wno-deprecated")
SET(CMAKE_C_FLAGS "-D USE_USB_CONNECTION -Wno-implicit-function-declaration")

# code optimization
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")

# hardware
add_definitions(-DHARDWARE)

set(IMU	   "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/IMU/")
include_directories (${IMU}/include/)
file(GLOB IMU_SOURCES 	"${IMU}/src/*.c" "${IMU}/src/*.cpp")

## FOR LAGS_DS Settings ##
set(LAGSDS   "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/lagsDS-lib/")
include_directories (${LAGSDS}/include/)
file(GLOB LAGSDS_SOURCES "${LAGSDS}/src/lagsDS/*.cpp")
## GPR_Wrap Settings ##
find_package(gaussian_process_regression)
include_directories(${gaussian_process_regression_INCLUDE_DIRS})
file(GLOB GPR_SOURCES "${LAGSDS}/src/lagsDS/GPRwrap.cpp")

add_library(Dependencies ${MODEL_SOURCES} ${IK_SOURCES} ${IMU_SOURCES})
add_library(Model ${LPVDS_SOURCES} ${WRAPPER_SOURCES} ${EXTRA_SOURCES})
TARGET_LINK_LIBRARIES(Model -lstdc++fs ${YARP_LIBRARIES} pthread fcl ccd)
add_executable(main ${STACK_SOURCES} ${GPR_SOURCES} )
TARGET_LINK_LIBRARIES(main Model Dependencies)
