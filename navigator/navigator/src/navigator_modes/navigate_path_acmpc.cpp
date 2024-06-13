#include <chrono>

#include "navigator/navigator_modes/navigate_path_acmpc.hpp"
#include "navigator/navigator.hpp"

namespace navigator
{

NavigatePathACMPC::NavigatePathACMPC()
{
}
NavigatePathACMPC::~NavigatePathACMPC()
{
}

bool NavigatePathACMPC::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                               std::shared_ptr<navigator::Navigator> navigator,
                               std::string nav_mode)
{
	node_ = parent;
	auto node = node_.lock();
	nav_mode_name_ = nav_mode;
	logger_ = node->get_logger();
	clock_ = node->get_clock();
	navigator_ = navigator;

	RCLCPP_INFO(logger_, "Configuring");
	
	if (!node->has_parameter("navigate_path_acmpc.planner")){
		node->declare_parameter("navigate_path_acmpc.planner", rclcpp::ParameterValue(std::string("rrtx_static")));
	}

	planner_name_ = node->get_parameter("navigate_path_acmpc.planner").as_string();
	RCLCPP_INFO(logger_, "Planner: %s", planner_name_.c_str());

	if (!node->has_parameter("navigate_path_acmpc.waypoint_position_tolerance")){
		node->declare_parameter("navigate_path_acmpc.waypoint_position_tolerance", rclcpp::ParameterValue(0.5));
	}

	waypoint_position_tolerance_ = node->get_parameter("navigate_path_acmpc.waypoint_position_tolerance").as_double();
	RCLCPP_INFO(logger_, "Waypoint position tolerance: %g", waypoint_position_tolerance_);

	if (!node->has_parameter("navigate_path_acmpc.goal_positio%n_tolerance")){
		node->declare_parameter("navigate_path_acmpc.goal_position_tolerance", rclcpp::ParameterValue(0.4));
	}
	
	goal_position_tolerance_ = node->get_parameter("navigate_path_acmpc.goal_position_tolerance").as_double();

	if (!node->has_parameter("navigate_path_acmpc.goal_velocity_tolerance")){
		node->declare_parameter("navigate_path_acmpc.goal_velocity_tolerance", rclcpp::ParameterValue(3.0));
	}

	goal_velocity_tolerance_ = node->get_parameter("navigate_path_acmpc.goal_velocity_tolerance").as_double();

	if (!node->has_parameter("navigate_path_acmpc.update_frequency")){
		node->declare_parameter("navigate_path_acmpc.update_frequency", rclcpp::ParameterValue(100.0));
	}

	update_frequency_ = node->get_parameter("navigate_path_acmpc.update_frequency").as_double();

	planner_client_ptr_ = rclcpp_action::create_client<PlannerActionT>(node, "/compute_path_to_pose");
	return true;
}

bool NavigatePathACMPC::activate(navigator_msgs::action::NavigatorCommand::Goal::ConstSharedPtr goal)
{

	auto node = node_.lock();
	//TODO: check configured state before activating

	RCLCPP_INFO(logger_, "Attempting to activate navigate path flight mode");
	if (!planner_client_ptr_->wait_for_action_server()) {
		RCLCPP_ERROR(logger_, "Planner server is not available after waiting");
		return false;
	}
	nav_msgs::msg::Odometry tmp_odom = navigator_->getCurrentOdom();
	// Remove when I migrate to just map static frame
	if (tmp_odom.header.frame_id != "map_ned") {
		vehicle_odom_.header.frame_id = tmp_odom.header.frame_id;
		vehicle_odom_.pose.pose = tmp_odom.pose.pose;
		vehicle_odom_.twist.twist = tmp_odom.twist.twist;
	} else if (tmp_odom.header.frame_id == "map_ned") {
		vehicle_odom_.header.frame_id = "map";
		vehicle_odom_.pose.pose.position.x = tmp_odom.pose.pose.position.y;
		vehicle_odom_.pose.pose.position.y  = tmp_odom.pose.pose.position.x;
		vehicle_odom_.pose.pose.position.z  = -tmp_odom.pose.pose.position.z;
		vehicle_odom_.twist.twist.linear.x = tmp_odom.twist.twist.linear.y;
		vehicle_odom_.twist.twist.linear.y = tmp_odom.twist.twist.linear.x;
		vehicle_odom_.twist.twist.linear.z = -tmp_odom.twist.twist.linear.z;
	}
	start_pose_.header.stamp = vehicle_odom_.header.stamp;
	start_pose_.header.frame_id = vehicle_odom_.header.frame_id;
	start_pose_.pose = vehicle_odom_.pose.pose;
	goal_pose_.header.stamp = goal->goal.header.stamp;
	goal_pose_.header.frame_id = goal->goal.header.frame_id;
	goal_pose_.pose = goal->goal.pose;
	auto planner_goal_msg = PlannerActionT::Goal();

	planner_goal_msg.planner_id = planner_name_;
	planner_goal_msg.start = start_pose_;
	planner_goal_msg.goal = goal_pose_;

	RCLCPP_INFO(logger_, "Requesting path from planner (%s)", planner_name_.c_str());

	path_received_ = false;
	planner_response_received_ = false;
	mission_complete_ = false;
	auto send_goal_options = rclcpp_action::Client<PlannerActionT>::SendGoalOptions();
	// send_goal_options.goal_response_callback =
	//  		std::bind(&NavigatePathACMPC::goalResponseCallback, this, std::placeholders::_1);
	send_goal_options.result_callback =
	 		std::bind(&NavigatePathACMPC::resultCallback, this, std::placeholders::_1);

	planner_client_ptr_->async_send_goal(planner_goal_msg, send_goal_options);

	while (rclcpp::ok() && !planner_response_received_) {}

	if (path_received_) {
		timer_ = node->create_wall_timer(
				std::chrono::duration<double>(1.0/update_frequency_),
				std::bind(&NavigatePathACMPC::onLoopCallback, this));
	}

	return path_received_;
}

void NavigatePathACMPC::onLoopCallback()
{
	nav_msgs::msg::Odometry tmp_odom = navigator_->getCurrentOdom();
	// Remove when I migrate to just map static frame
	if (tmp_odom.header.frame_id != "map_ned") {
		vehicle_odom_.header.frame_id = tmp_odom.header.frame_id;
		vehicle_odom_.pose.pose = tmp_odom.pose.pose;
		vehicle_odom_.twist.twist = tmp_odom.twist.twist;
	} else if (tmp_odom.header.frame_id == "map_ned") {
		vehicle_odom_.header.frame_id = "map";
		vehicle_odom_.pose.pose.position.x = tmp_odom.pose.pose.position.y;
		vehicle_odom_.pose.pose.position.y  = tmp_odom.pose.pose.position.x;
		vehicle_odom_.pose.pose.position.z  = -tmp_odom.pose.pose.position.z;
		vehicle_odom_.twist.twist.linear.x = tmp_odom.twist.twist.linear.y;
		vehicle_odom_.twist.twist.linear.y = tmp_odom.twist.twist.linear.x;
		vehicle_odom_.twist.twist.linear.z = -tmp_odom.twist.twist.linear.z;
	}
	updateWaypoint();

	if (missionComplete() && !mission_complete_) {
		mission_complete_ = true;
		navigator_->loiter();
	}
}

// void NavigatePathACMPC::goalResponseCallback(std::shared_future<PlannerGoalHandleActionT::SharedPtr> future)
// {
// 	auto node = node_.lock();
// 	auto goal_handle = future.get();
// 	if (!goal_handle) {
// 		RCLCPP_ERROR(logger_, "Goal was rejected by server");
// 		planner_response_received_ = true;
// 	} else {
// 		RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result");
// 	}
// }

void NavigatePathACMPC::resultCallback(const PlannerGoalHandleActionT::WrappedResult & result)
{
	auto node = node_.lock();

	planner_response_received_ = true;
	switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:
			path_received_ = true;
			current_path_waypoint_ = 0;
			path_ = result.result->path;
			return;
		case rclcpp_action::ResultCode::ABORTED: RCLCPP_ERROR(logger_, "Goal was aborted");
			return;
		case rclcpp_action::ResultCode::CANCELED: RCLCPP_ERROR(logger_, "Goal was canceled");
			return;
		default: RCLCPP_ERROR(logger_, "Unknown result code");
			return;
	}
}
bool NavigatePathACMPC::deactivate()
{
	auto node = node_.lock();
	RCLCPP_INFO(logger_, "Deactivating navigate to pose flight mode");
	timer_->cancel();
	timer_.reset();
	start_pose_ = geometry_msgs::msg::PoseStamped();
	goal_pose_ = geometry_msgs::msg::PoseStamped();
	path_ = nav_msgs::msg::Path();
	path_received_ = false;
	path_requested_ = false;
	mission_complete_ = false;

	return true;
}

bool NavigatePathACMPC::cleanup()
{
	node_.reset();
	navigator_.reset();
	return true;
}

void NavigatePathACMPC::publishNavigatorSetpoint()
{
	auto node = node_.lock();
	navigator_msgs::msg::NavigatorTrajectorySetpoint trajectory_setpoint;
	trajectory_setpoint.header.frame_id = "world";
	trajectory_setpoint.header.stamp = clock_->now();
	trajectory_setpoint.path = path_;
	trajectory_setpoint.current_waypoint = current_path_waypoint_;
	trajectory_setpoint.controller_type = trajectory_setpoint.CONTROLLER_TYPE_ACMPC;

	navigator_->publishTrajectorySetpoint(trajectory_setpoint);
}

void NavigatePathACMPC::updateWaypoint()
{
	auto distance_to_wp = std::hypot(
			vehicle_odom_.pose.pose.position.x - path_.poses[current_path_waypoint_].pose.position.x,
			vehicle_odom_.pose.pose.position.y - path_.poses[current_path_waypoint_].pose.position.y,
			vehicle_odom_.pose.pose.position.z - path_.poses[current_path_waypoint_].pose.position.z);
	if (distance_to_wp <= waypoint_position_tolerance_
		&& (size_t)(current_path_waypoint_ + 1) < path_.poses.size()) {
		current_path_waypoint_ += 1;
	}
}

bool NavigatePathACMPC::missionComplete()
{
	bool mission_complete = false;
	if (std::hypot(
			vehicle_odom_.pose.pose.position.x - goal_pose_.pose.position.x,
			vehicle_odom_.pose.pose.position.y - goal_pose_.pose.position.y,
			vehicle_odom_.pose.pose.position.z - goal_pose_.pose.position.z) <= goal_position_tolerance_
		&&
		std::hypot(
				vehicle_odom_.twist.twist.linear.x,
				vehicle_odom_.twist.twist.linear.y,
				vehicle_odom_.twist.twist.linear.z) <= goal_velocity_tolerance_) {
		mission_complete = true;
	}
	auto node = node_.lock();

	return mission_complete;
}

} // namespace navigator
