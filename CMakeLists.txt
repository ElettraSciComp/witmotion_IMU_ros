cmake_minimum_required(VERSION 3.8)
project(witmotion_ros)

# Add support for C++14


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

add_subdirectory(witmotion-uart-qt)

include_directories(include)
include_directories(witmotion-uart-qt/include)


find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 COMPONENTS SerialPort REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_SOURCES
    include/witmotion_ros.h
    )

add_library(witmotion_ros
    ${MOC_SOURCES}
    src/witmotion_ros.cpp
    )

message(WARNING "tf2_geometry_msgs include dir: ${tf2_geometry_msgs_INCLUDE_DIRS} ")


set(dependencies
  tf2
  tf2_geometry_msgs
  rclcpp
  std_srvs
  sensor_msgs
  std_msgs
  rcl_interfaces
  rclcpp_lifecycle
  lifecycle_msgs
  )
ament_target_dependencies(witmotion_ros ${dependencies})

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES} ${rclcpp_LIBRARIES} ${std_srvs_LIBRARIES}
  ${sensor_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES} ${rosgraph_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${rcl_interfaces_LIBRARIES} ${rclcpp_lifecycle_LIBRARIES} ${lifecycle_msgs_LIBRARIES}) 

target_link_libraries(witmotion_ros  ${LIBRARY_DIRS} witmotion-uart Qt5::Core Qt5::SerialPort)


ament_export_dependencies(ament_cmake)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_srvs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rosgraph_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(witmotion_ros)
ament_export_dependencies(witmotion-uart)
ament_export_dependencies(Qt5Core)
ament_export_dependencies(Qt5SerialPort)
ament_export_targets(export_witmotion_ros HAS_LIBRARY_TARGET)
ament_export_targets(export_witmotion-uart HAS_LIBRARY_TARGET)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(witmotion_ros witmotion-uart ${LIBS})


install(
    TARGETS witmotion-uart
    EXPORT export_witmotion-uart
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib
)


install(
    TARGETS witmotion_ros
    EXPORT export_witmotion_ros
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib
)



add_executable(witmotion_ros_node src/witmotion_ros_node.cpp)
target_link_libraries(witmotion_ros_node witmotion_ros)
ament_target_dependencies(witmotion_ros_node Qt5Core)


#install( TARGETS witmotion_ros_node DESTINATION bin )


install(
  TARGETS witmotion_ros_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
