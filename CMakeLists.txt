cmake_minimum_required(VERSION 3.0)
project(testing_custom_node)

file(GLOB SOURCES
     "src/*.cpp"
     "include/*.h")

include_directories(${PROJECT_SOURCE_DIR}/include)

#find gtsam library
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

find_package(Boost 1.50 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})


add_executable(${PROJECT_NAME} 
            #    src/test.cpp
               ${SOURCES})


target_link_libraries (${PROJECT_NAME} 
                       gtsam
                       Eigen3::Eigen)
