cmake_minimum_required(VERSION 3.8)
project(uam_navigator)

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
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(uam_navigator_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(uam_util REQUIRED)
find_package(uam_vehicle_interface REQUIRED)
find_package(uam_planner_msgs REQUIRED)

option(RUN_SITL "Target build is simulation" ON)## RUN IN SIMULATION

if(RUN_SITL)
  message(STATUS "RUN_SITL ACTIVE")
  add_definitions(-DRUN_SITL=1)
endif()

include_directories(include)

add_library(navigator_modes SHARED
  src/navigator_modes/navigate_to_pose.cpp
  src/navigator_modes/loiter.cpp
  src/navigator_modes/takeoff.cpp)
ament_target_dependencies(navigator_modes rclcpp rclcpp_action Eigen3 nav_msgs uam_util uam_planner_msgs)
target_include_directories(navigator_modes PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/navigator_modes>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}/navigator_modes>)

add_library(navigator_core SHARED
  src/navigator.cpp)
target_link_libraries(navigator_core navigator_modes)
ament_target_dependencies(navigator_core uam_navigator_msgs nav_msgs)
target_include_directories(navigator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_executable(navigator_main src/navigator_main.cpp)
ament_target_dependencies(navigator_main rclcpp)
target_include_directories(navigator_main PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(navigator_main navigator_core)
install(TARGETS navigator_main DESTINATION lib/${PROJECT_NAME})

ament_package()
