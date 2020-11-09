project(lagsDS)
cmake_minimum_required(VERSION 2.8)

####################### CMAKE STUFF ##################
set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/test)
set(CMAKE_INSTALL_PREFIX    /usr/)
set(LIBRARY_OUTPUT_PATH     ${PROJECT_SOURCE_DIR}/lib )
set(LIB_INSTALL_PATH        ${CMAKE_INSTALL_PREFIX}lib/) 
set(CMAKE_MODULE_PATH       ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
set(INCLUDE_DIR             ${CMAKE_CURRENT_SOURCE_DIR}/include)    
set(SRC_DIR                 ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_CXX_FLAGS "-std=c++11")


####################### Adding GPR package [Work wit]##################
#find_package(gaussian_process_regression)
#include_directories(${gaussian_process_regression_INCLUDE_DIRS})

set(GPR "${CMAKE_CURRENT_SOURCE_DIR}/../gaussian_process_regression/")
include_directories (${GPR}/include/)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package (Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########


set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(H_FILES     ${INCLUDE_DIR}/lagsDS.h
                ${INCLUDE_DIR}/utils.h
                ${INCLUDE_DIR}/GMM.h
                ${INCLUDE_DIR}/GPRwrap.h)

## Specify additional locations of header files
include_directories(include ${EIGEN3_INCLUDE_DIRS} ${gaussian_process_regression_INCLUDE_DIRS})

## Declare a C++ library
set(SRCS src/lagsDS/lagsDS.cpp
         src/lagsDS/utils.cpp
         src/lagsDS/GMM.cpp
         src/lagsDS/GPRwrap.cpp)

## Declare a cpp library
add_library(${PROJECT_NAME} ${H_FILES}  ${SRCS})
target_link_libraries(${PROJECT_NAME} )



# Testing scripts
add_executable(test_lagsDS         src/test_lagsDS.cpp)
target_link_libraries(test_lagsDS  lagsDS)

add_executable(test_gpr            src/test_gpr.cpp)
target_link_libraries(test_gpr    lagsDS)



# INSTALL
message(status "    CMAKE_INSTALL_PREFIX:" ${CMAKE_INSTALL_PREFIX})
install(TARGETS ${PROJECT_NAME} DESTINATION lib EXPORT ${PROJECT_NAME}-targets)
install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/${PROJECT_NAME})
install(FILES ${SGD_INC} DESTINATION include/)
