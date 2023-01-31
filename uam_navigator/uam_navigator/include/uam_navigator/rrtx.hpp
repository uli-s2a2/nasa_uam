#pragma once
// ROS2 includes
//#include <rclcpp/rclcpp.hpp>
//#include <rclcpp/clock.hpp>

//#include <nav_msgs/msg/path.hpp>
//#include <nav_msgs/msg/odometry.hpp>
//#include <px4_msgs/msg/rc_channels.hpp>
//#include <px4_msgs/msg/vehicle_local_position.hpp>
//#include <mav_rrtqx_msgs/msg/path.hpp>
// other libraries
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <geometric_shapes/bodies.h>
#include <Eigen/Dense>


namespace ob = ompl::base;
namespace og = ompl::geometric;


#define RRTX_CALLBACK_RATE_MS 10
#define RRTX_PLANNER_MAX_SOLVE_TIME_S 1.0
#define RRTX_ARENA_X_BOUNDS_LOW 0.0
#define RRTX_ARENA_X_BOUNDS_HIGH 5.0
#define RRTX_ARENA_Y_BOUNDS_LOW 0.0
#define RRTX_ARENA_Y_BOUNDS_HIGH 5.0
#define RRTX_ARENA_Z_BOUNDS_LOW  0
#define RRTX_ARENA_Z_BOUNDS_HIGH 5
#define RRTX_ADMISSIBLE_WINDOW 0.1

class RRTX
{
public:
	RRTX();
	virtual ~RRTX() = default;
private:
	// Class Variables
	std::shared_ptr<ob::SE3StateSpace> space_ptr;
	std::shared_ptr<ob::SpaceInformation> space_info_ptr;
	std::shared_ptr<ob::ProblemDefinition> prob_def_ptr;
	std::shared_ptr<og::RRTXstatic> rrtxstatic_ptr;
	std::shared_ptr<ob::Planner> planner_ptr;
	std::vector<bodies::Box> obstacles_;
	ob::PlannerStatus planner_status_;
	uint8_t current_waypoint_{0};
	double admissible_window_{0};

	// Class methods
	void plan(Eigen::Vector3d start, Eigen::Vector3d goal);
	void publish_obstacle_markers();
	void publish_waypoint();
	void terminal_state_evaluation();
	bool isStateValid(const ob::State *state) const;

}; // Class RRTX
