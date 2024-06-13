cmake_minimum_required(VERSION 3.8)
project(px4_bridge)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(controller_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_bridge_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(px4_ros_com REQUIRED)

include_directories(include)

set(PX4_BRIDGE_DEPS
  Eigen3
  rclcpp
  px4_msgs
  controller_msgs
  util
  nav_msgs
  px4_bridge_msgs
  tf2_ros
  tf2_eigen
  px4_ros_com
)

add_executable(px4_bridge src/px4_interface.cpp)
ament_target_dependencies(px4_bridge ${PX4_BRIDGE_DEPS})
target_include_directories(px4_bridge
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
install(TARGETS px4_bridge DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()
