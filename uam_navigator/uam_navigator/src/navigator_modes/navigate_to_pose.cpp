#include "navigate_to_pose.hpp"

namespace uam_navigator
{

NavigateToPose::NavigateToPose()
{
}
NavigateToPose::~NavigateToPose()
{
}

bool NavigateToPose::configure()
{
	auto node = node_.lock();

	RCLCPP_INFO(node->get_logger(), "Configuring");

	node->declare_parameter("navigate_to_pose.planner", std::string("RrtxStatic"));
	node->get_parameter("navigate_to_pose.planner", planner_name_);
	node->declare_parameter("navigate_to_pose.waypoint_position_tolerance", 0.1);
	node->get_parameter("navigate_to_pose.waypoint_position_tolerance", waypoint_position_tolerance_);
	node->declare_parameter("navigate_to_pose.goal_position_tolerance", 0.1);
	node->get_parameter("navigate_to_pose.goal_position_tolerance", goal_position_tolerance_);
	node->declare_parameter("navigate_to_pose.goal_velocity_tolerance", 0.1);
	node->get_parameter("navigate_to_pose.goal_velocity_tolerance", goal_velocity_tolerance_);

	planner_client_ptr_ = rclcpp_action::create_client<ActionT>(node, "compute_path_to_pose");

	return true;
}

bool NavigateToPose::activate(const nav_msgs::msg::Odometry & start, const nav_msgs::msg::Odometry & goal)
{
	if (!path_requested_){
		auto node = node_.lock();
		RCLCPP_INFO(node->get_logger(), "Activating navigate to pose flight mode");

		start_pose_.header.stamp = node->get_clock()->now();
		start_pose_.pose = start.pose.pose;
		goal_pose_.header.stamp = goal.header.stamp;
		goal_pose_.pose = goal.pose.pose;

		auto planner_goal_msg = ActionT::Goal();

		planner_goal_msg.planner_id = planner_name_;
		planner_goal_msg.start = start_pose_;
		planner_goal_msg.goal = goal_pose_;

		RCLCPP_INFO(node->get_logger(), "Requesting path from planner (%s)", planner_name_.c_str());

		auto send_goal_options = rclcpp_action::Client<ActionT>::SendGoalOptions();
		send_goal_options.goal_response_callback =
				std::bind(&NavigateToPose::goal_response_callback, this, std::placeholders::_1);
		send_goal_options.result_callback =
				std::bind(&NavigateToPose::result_callback, this, std::placeholders::_1);
		this->planner_client_ptr_->async_send_goal(planner_goal_msg, send_goal_options);
		current_path_waypoint_ = 0;
		path_requested_ = true;
	}
	return path_received_;
}

void NavigateToPose::goal_response_callback(std::shared_future<GoalHandleActionT::SharedPtr> future)
{
	auto node = node_.lock();
	auto goal_handle = future.get();
	if (!goal_handle) {
		RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
	} else {
		RCLCPP_INFO(node->get_logger(), "Goal accepted by server, waiting for result");
	}
}

void NavigateToPose::result_callback(const GoalHandleActionT::WrappedResult & result)
{
	auto node = node_.lock();

	switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED: break;
		case rclcpp_action::ResultCode::ABORTED: RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
			return;
		case rclcpp_action::ResultCode::CANCELED: RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
			return;
		default: RCLCPP_ERROR(node->get_logger(), "Unknown result code");
			return;
	}

	path_received_ = true;
	path_ = result.result->path;
}
bool NavigateToPose::deactivate()
{
	auto node = node_.lock();
	RCLCPP_INFO(node->get_logger(), "Deactivating navigate to pose flight mode");
	start_pose_ = geometry_msgs::msg::PoseStamped();
	goal_pose_ = geometry_msgs::msg::PoseStamped();
	path_received_ = false;
	path_requested_ = false;
	return true;
}

bool NavigateToPose::cleanup()
{
	node_.reset();
	return true;
}

nav_msgs::msg::Odometry
NavigateToPose::compute_position_setpoint(const nav_msgs::msg::Odometry & current_odom)
{
	auto node = node_.lock();
	(void)current_odom;
	nav_msgs::msg::Odometry odom;
	odom.header.frame_id = path_.poses[current_path_waypoint_].header.frame_id;
	odom.header.stamp = node->get_clock()->now();
	odom.child_frame_id = "baselink_frd";
	odom.pose.pose = path_.poses[current_path_waypoint_].pose;
	update_waypoint(current_odom);
	return odom;
}

void NavigateToPose::update_waypoint(const nav_msgs::msg::Odometry &current_odom)
{
	if (std::hypot(
			current_odom.pose.pose.position.x - path_.poses[current_path_waypoint_].pose.position.x,
			current_odom.pose.pose.position.y - path_.poses[current_path_waypoint_].pose.position.y,
			current_odom.pose.pose.position.z - path_.poses[current_path_waypoint_].pose.position.z) <= waypoint_position_tolerance_
		&& (size_t)(current_path_waypoint_ + 1) < path_.poses.size()) {
		current_path_waypoint_ += 1;
	}
}

bool NavigateToPose::mission_complete(const nav_msgs::msg::Odometry & current_odom)
{
	bool mission_complete = false;
	if (std::hypot(
			current_odom.pose.pose.position.x - goal_pose_.pose.position.x,
			current_odom.pose.pose.position.y - goal_pose_.pose.position.y,
			current_odom.pose.pose.position.z - goal_pose_.pose.position.z) <= goal_position_tolerance_
		&&
		std::hypot(
				current_odom.twist.twist.linear.x,
				current_odom.twist.twist.linear.y,
				current_odom.twist.twist.linear.z) <= goal_velocity_tolerance_) {
		mission_complete = true;
	}
	return mission_complete;
}

} // namespace uam_navigator