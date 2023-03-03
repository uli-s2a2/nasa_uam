cmake_minimum_required(VERSION 3.8)
project(uam_vehicle_interface)

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
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(uam_control_msgs REQUIRED)
find_package(uam_flight_manager_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(uam_util REQUIRED)
find_package(nav_msgs REQUIRED)

option(RUN_SITL "Target build is simulation" ON)## RUN IN SIMULATION

if(RUN_SITL)
  message(STATUS "RUN_SITL ACTIVE")
  add_definitions(-DRUN_SITL=1)
endif()

include_directories(include)

add_executable(px4_interface src/px4_interface.cpp)
ament_target_dependencies(px4_interface Eigen3 rclcpp px4_msgs uam_flight_manager_msgs uam_control_msgs uam_util nav_msgs)
target_include_directories(px4_interface
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
install(TARGETS px4_interface DESTINATION lib/${PROJECT_NAME})

ament_package()
