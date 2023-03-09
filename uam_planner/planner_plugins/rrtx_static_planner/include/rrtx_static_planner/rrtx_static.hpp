#pragma once

#include "Eigen/Dense"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateSpace.h"
#include "ompl/config.h"
#include "ompl/base/DiscreteMotionValidator.h"
#include "ompl/geometric/planners/rrt/RRTXstatic.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "geometric_shapes/bodies.h"
#include "uam_planner/planner_base.hpp"
#include "uam_planner/planner_exceptions.hpp"
#include "uam_mapping_msgs/msg/obstacle_array.hpp"

#define RRTX_CALLBACK_RATE_MS 10
#define RRTX_PLANNER_MAX_SOLVE_TIME_S 5.0
#define RRTX_ARENA_X_BOUNDS_LOW -0.5
#define RRTX_ARENA_X_BOUNDS_HIGH 5.0
#define RRTX_ARENA_Y_BOUNDS_LOW -0.5
#define RRTX_ARENA_Y_BOUNDS_HIGH 5.0
#define RRTX_ARENA_Z_BOUNDS_LOW  0.3
#define RRTX_ARENA_Z_BOUNDS_HIGH 1.0
#define RRTX_ADMISSIBLE_WINDOW 0.1

namespace rrtx_static_planner
{

class RrtxStatic: public uam_planner::PlannerBase
{
public:
	RrtxStatic();
	~RrtxStatic();

	void configure(
			const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
			std::string name) override;

	void cleanup() override;

	void activate() override;

	void deactivate() override;

	nav_msgs::msg::Path create_path(
			const geometry_msgs::msg::PoseStamped & start,
			const geometry_msgs::msg::PoseStamped & goal) override;

	using ObstacleMap = std::unordered_map<uint8_t, bodies::BodyPtr>;
protected:
	rclcpp::Subscription<uam_mapping_msgs::msg::ObstacleArray>::SharedPtr obstacle_sub_;
	rclcpp::Clock::SharedPtr clock_;
	rclcpp::Logger logger_{rclcpp::get_logger("planner_server")};
	std::string name_, global_frame_;
	rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

	ompl::base::StateSpacePtr state_space_ptr_;
	ompl::base::SpaceInformationPtr  space_info_ptr_;
	ompl::base::ProblemDefinitionPtr prob_def_ptr_;
	ompl::base::PlannerPtr planner_ptr_;

	ObstacleMap obstacles_;
	uint8_t current_waypoint_{0};
	double admissible_window_{0};

	// Class functions
	bool isStateValid(const ompl::base::State *state) const;

};
}