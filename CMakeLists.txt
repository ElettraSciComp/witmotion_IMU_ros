cmake_minimum_required(VERSION 3.8)
project(witmotion_ros)

# VERSIONING
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/witmotion-uart-qt/version")
    message(FATAL_ERROR "witmotion-uart-qt directory does not contain submodule data. Please update submodules to compile the underlying library in-place for ROS integration")
endif(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/witmotion-uart-qt/version")
execute_process(COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE COMMIT_ID
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
file(STRINGS "witmotion-uart-qt/version" PROJECT_VERSION)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(PROJECT_VERSION "${PROJECT_VERSION}")
else(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(PROJECT_VERSION "${PROJECT_VERSION}~dev_${COMMIT_ID}")
endif(CMAKE_BUILD_TYPE STREQUAL "Release")
message(STATUS "Building version ${PROJECT_VERSION}")
string(REGEX MATCH "[0-9]*\.[0-9]*\.[0-9]*" modified_version ${PROJECT_VERSION})
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/package.xml package_data)
string(REGEX REPLACE "\<version\>.*\<\/version\>" "\<version\>${modified_version}\<\/version\>" package_data "${package_data}")
file(WRITE  ${CMAKE_CURRENT_SOURCE_DIR}/package.xml "${package_data}")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    roslib
    roscpp
    tf2
    sensor_msgs
    geometry_msgs
    std_msgs
    std_srvs
    nav_msgs
    tf2_geometry_msgs
    rosgraph_msgs
)

catkin_package(CATKIN_DEPENDS
    message_runtime
    tf2
    sensor_msgs
    geometry_msgs
    std_msgs
    std_srvs
    nav_msgs
    tf2_geometry_msgs
    rosgraph_msgs
)

add_subdirectory(witmotion-uart-qt)
include_directories(${catkin_INCLUDE_DIRS} include witmotion-uart-qt/include)

find_package(Qt5 REQUIRED COMPONENTS Core SerialPort)
set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_SOURCES
    include/witmotion_ros.h
    )

add_library(witmotion_ros
    ${MOC_SOURCES}
    src/witmotion_ros.cpp
    )
target_link_libraries(witmotion_ros ${catkin_LIBRARIES} witmotion-uart Qt5::Core Qt5::SerialPort)

add_executable(witmotion_ros_node src/witmotion_ros_node.cpp)
target_link_libraries(witmotion_ros_node PRIVATE ${catkin_LIBRARIES} witmotion_ros)

