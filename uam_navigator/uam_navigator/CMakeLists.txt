cmake_minimum_required(VERSION 3.5)
project(uam_navigator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(uam_navigator_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(ompl REQUIRED)
find_package(geometric_shapes REQUIRED)

option(RUN_SITL "Target build is simulation" ON)## RUN IN SIMULATION

if(RUN_SITL)
  message(STATUS "RUN_SITL ACTIVE")
  add_definitions(-DRUN_SITL=1)
endif()

include_directories(include)

add_library(rrtx src/libs/rrtx.cpp)
ament_target_dependencies(rrtx rclcpp Eigen3 px4_msgs)
target_include_directories(rrtx PUBLIC
        ${OMPL_INCLUDE_DIRS}
        ${GEOMETRIC_SHAPES_INCLUDE_DIRS}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(rrtx ${OMPL_LIBRARIES})

add_executable(navigator src/navigator.cpp)
ament_target_dependencies(navigator rclcpp Eigen3 nav_msgs uam_navigator_msgs px4_msgs)
#target_include_directories(navigator PUBLIC
#        ${OMPL_INCLUDE_DIRS}
#        ${GEOMETRIC_SHAPES_INCLUDE_DIRS}
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>)
target_link_libraries(navigator rrtx)
install(TARGETS navigator DESTINATION lib/${PROJECT_NAME})


############
# Install ##
############

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
