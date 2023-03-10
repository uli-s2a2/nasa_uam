cmake_minimum_required(VERSION 3.8)
project(uam_mapping)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uam_mapping_msgs REQUIRED)

include_directories(include)

add_executable(obstacle_advertiser src/obstacle_advertiser.cpp)
ament_target_dependencies(obstacle_advertiser rclcpp Eigen3 shape_msgs uam_mapping_msgs)
install(TARGETS obstacle_advertiser DESTINATION lib/${PROJECT_NAME})


ament_package()
