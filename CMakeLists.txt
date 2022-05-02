cmake_minimum_required(VERSION 3.8)
project(witmotion_ros)

add_compile_options(-std=gnu++14)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    roslib
    roscpp
    tf2
    sensor_msgs
    geometry_msgs
    std_msgs
    nav_msgs
)

catkin_package(CATKIN_DEPENDS
    message_runtime
    tf2
    sensor_msgs
    geometry_msgs
    std_msgs
    nav_msgs
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
target_link_libraries(witmotion_ros ${catkin_LIBRARIES} witmotion-uart)

add_executable(witmotion_ros_node src/witmotion_ros_node.cpp)
target_link_libraries(witmotion_ros_node PRIVATE ${catkin_LIBRARIES} witmotion_ros)

