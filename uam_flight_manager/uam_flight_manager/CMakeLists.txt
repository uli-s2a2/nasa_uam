cmake_minimum_required(VERSION 3.8)
project(uam_flight_manager)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(uam_navigator REQUIRED)
find_package(uam_mapping REQUIRED)
find_package(uam_control REQUIRED)


ament_package()
