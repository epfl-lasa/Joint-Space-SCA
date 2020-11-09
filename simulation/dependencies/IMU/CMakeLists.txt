cmake_minimum_required(VERSION 3.5)
project(IMU)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

include_directories (include/)

file(GLOB IMU_SOURCES "src/*.c" "src/*.cpp" "main.cpp")

SET(CMAKE_C_FLAGS "-D USE_USB_CONNECTION")

add_executable (main ${IMU_SOURCES})
