#pragma once

#include "navigator/navigator_modes/navigator_mode.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "planner_msgs/action/compute_path_to_pose.hpp"

namespace navigator
{

class NavigateToPose: public NavigatorMode
{
public:
	using PlannerActionT = planner_msgs::action::ComputePathToPose;
	using PlannerGoalHandleActionT = rclcpp_action::ClientGoalHandle<PlannerActionT>;
	NavigateToPose();
	~NavigateToPose();

protected:
	bool configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
	               std::shared_ptr<navigator::Navigator> navigator,
	               std::string nav_mode) override;
	bool activate(navigator_msgs::action::NavigatorCommand::Goal::ConstSharedPtr goal) override;
	bool deactivate() override;
	bool cleanup() override;
	void publishNavigatorSetpoint() override;
	bool missionComplete();
	// void goalResponseCallback(std::shared_future<PlannerGoalHandleActionT::SharedPtr> future);
	void resultCallback(const PlannerGoalHandleActionT::WrappedResult & result);
	void updateWaypoint();
	void onLoopCallback();

	// ROS2
	rclcpp::TimerBase::SharedPtr timer_;

	// Class Variables
	double waypoint_position_tolerance_;
	double goal_position_tolerance_;
	double goal_velocity_tolerance_;
	double update_frequency_;
	std::string planner_name_;
	geometry_msgs::msg::PoseStamped goal_pose_;
	geometry_msgs::msg::PoseStamped start_pose_;
	nav_msgs::msg::Odometry vehicle_odom_;
	nav_msgs::msg::Path path_;
	std::uint8_t current_path_waypoint_{0};
	bool path_received_{false};
	bool path_requested_{false};
	bool planner_response_received_{false};
	rclcpp_action::Client<PlannerActionT>::SharedPtr planner_client_ptr_;
	bool mission_complete_{false};
};

}