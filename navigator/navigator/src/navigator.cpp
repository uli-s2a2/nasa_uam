#include "util/node_utils.hpp"
#include "navigator/navigator.hpp"

namespace navigator
{
Navigator::Navigator(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("navigator", options), default_navigator_plugin_ids_{"navigate_to_pose_rrtx_static", "navigate_path_acmpc"}
{
	this->declare_parameter("navigator_plugins", rclcpp::ParameterValue(default_navigator_plugin_ids_));
	this->get_parameter("navigator_plugins", navigator_plugin_ids_);
	vehicle_name_ = this->get_namespace();
}

Navigator::~Navigator()
{
}

util::CallbackReturn
Navigator::on_configure(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Configuring");

	auto node = shared_from_this();

	navigators_.insert({"Takeoff", std::make_shared<Takeoff>()});
//	navigators_.insert({NavMode::NAV_MODE_LAND, std::make_unique<Land>()});
	navigators_.insert({"Loiter", std::make_shared<Loiter>()});
	navigators_.insert({"navigate_to_pose_rrtx_static", std::make_shared<NavigateToPose>()});
	navigators_.insert({"navigate_path_acmpc", std::make_shared<NavigatePathACMPC>()});
	position_setpoint_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>("navigator/position_setpoint", 10);
	trajectory_setpoint_publisher_ = node->create_publisher<navigator_msgs::msg::NavigatorTrajectorySetpoint>("navigator/trajectory_setpoint", 10);
	navigator_status_publisher_ = node->create_publisher<navigator_msgs::msg::NavigatorStatus>("navigator/navigator_status", 10);

	for (const auto & navigator : navigators_) {
		navigator.second->configure(node, navSharedFromThis(), navigator.first);
	}

	navigator_transitions_.insert({"Takeoff", std::vector<std::string>{"Loiter", "Land"}});
	auto tmp = navigator_plugin_ids_;
	tmp.emplace_back("Land");
	navigator_transitions_.insert({"Loiter", tmp});
	for (const auto & nav_mode : navigator_plugin_ids_) {
		navigator_transitions_.insert({nav_mode, std::vector<std::string>{"Loiter", "Land"}});
	}
	navigator_transitions_.insert({"Idle", std::vector<std::string>{"Takeoff"}});

	timer_ = node->create_wall_timer(std::chrono::milliseconds(10),
								  std::bind(&Navigator::onLoop, this));

	vehicle_odometry_sub_ =
			node->create_subscription<nav_msgs::msg::Odometry>(
					"odom",
					10,
					[this](const nav_msgs::msg::Odometry::UniquePtr msg)
					{
						vehicle_odom_ = *msg;
					});

	current_nav_mode_ = "Idle";
	requested_nav_mode_ = "Idle";

	nav_command_action_server_ = std::make_unique<ActionServerNavigatorCommand>(
			shared_from_this(),
			"send_navigator_command",
			std::bind(&Navigator::commandCallback, this),
			nullptr,
			std::chrono::milliseconds(500),
			true);

	nav_command_action_client_ = rclcpp_action::create_client<ActionNavigatorCommand>(node, "send_navigator_command");

	return util::CallbackReturn::SUCCESS;
}

util::CallbackReturn
Navigator::on_activate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Activating");

	position_setpoint_publisher_->on_activate();
	trajectory_setpoint_publisher_->on_activate();
	navigator_status_publisher_->on_activate();
	nav_command_action_server_->activate();

	auto node = shared_from_this();
	return util::CallbackReturn::SUCCESS;
}

util::CallbackReturn
Navigator::on_deactivate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Deactivating");

	position_setpoint_publisher_->on_deactivate();
	trajectory_setpoint_publisher_->on_deactivate();
	navigator_status_publisher_->on_deactivate();
	nav_command_action_server_->deactivate();

	NavigatorMap ::iterator it;
	for (it = navigators_.begin(); it != navigators_.end(); ++it) {
		it->second->deactivate();
	}

	return util::CallbackReturn::SUCCESS;
}

util::CallbackReturn
Navigator::on_cleanup(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Cleaning up");

	vehicle_odometry_sub_.reset();
	position_setpoint_publisher_.reset();
	trajectory_setpoint_publisher_.reset();
	navigator_status_publisher_.reset();
	nav_command_action_server_.reset();
	vehicle_odom_ = nav_msgs::msg::Odometry();
	timer_->reset();

	NavigatorMap ::iterator it;
	for (it = navigators_.begin(); it != navigators_.end(); ++it) {
		it->second->cleanup();
	}

	navigators_.clear();
	return util::CallbackReturn::SUCCESS;
}

util::CallbackReturn
Navigator::on_shutdown(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Shutting down");

	vehicle_odometry_sub_.reset();
	position_setpoint_publisher_.reset();
	trajectory_setpoint_publisher_.reset();
	navigator_status_publisher_.reset();
	timer_->reset();

	return util::CallbackReturn::SUCCESS;
}

void Navigator::onLoop()
{
	auto node = shared_from_this();

	navigator_msgs::msg::NavigatorStatus nav_status_msg;
	nav_status_msg.stamp = node->get_clock()->now();
	nav_status_msg.nav_mode = current_nav_mode_;
	navigator_status_publisher_->publish(nav_status_msg);

	if (current_nav_mode_ == "Idle") {
		return;
	}

	navigators_[current_nav_mode_]->publishNavigatorSetpoint();

}

void Navigator::loiter()
{
	ActionNavigatorCommandGoal nav_command_request;
	nav_command_request.command = "Loiter";
	nav_command_action_client_->async_send_goal(nav_command_request);
}

void Navigator::takeoff()
{
	ActionNavigatorCommandGoal nav_command_request;
	nav_command_request.command = "Takeoff";
	nav_command_action_client_->async_send_goal(nav_command_request);
}

void Navigator::land()
{
	ActionNavigatorCommandGoal nav_command_request;
	nav_command_request.command = "Land";
	nav_command_action_client_->async_send_goal(nav_command_request);
}

void Navigator::publishOdometrySetpoint(nav_msgs::msg::Odometry odom_msg)
{
	position_setpoint_publisher_->publish(odom_msg);
}

void Navigator::publishTrajectorySetpoint(navigator_msgs::msg::NavigatorTrajectorySetpoint trajectory_setpoint_msg)
{
	trajectory_setpoint_publisher_->publish(trajectory_setpoint_msg);
}


void Navigator::commandCallback()
{

	auto goal = nav_command_action_server_->get_current_goal();
	auto result = std::make_shared<ActionNavigatorCommand::Result>();

	RCLCPP_DEBUG(get_logger(), "Received command %s", goal->command.c_str());

	try {
		if (isServerInactive(nav_command_action_server_) || isCancelRequested(nav_command_action_server_)) {
			return;
		}
		if (navigators_.find(goal->command) == navigators_.end()) {
			throw navigator::InvalidNavigator("Navigator plugin " + goal->command + " is invalid");
		}
		if (std::find(
				navigator_transitions_[current_nav_mode_].begin(),
				navigator_transitions_[current_nav_mode_].end(),
				goal->command) != navigator_transitions_[current_nav_mode_].end()) {
			if (navigators_[goal->command]->activate(goal)) {
				auto previous_nav_mode = current_nav_mode_;
				current_nav_mode_ = goal->command;
				mission_complete_ = false;
				if (previous_nav_mode != "Idle") {
					navigators_[previous_nav_mode]->deactivate();
				}
				nav_command_action_server_->succeeded_current(result);
			} else {
				throw navigator::NavigatorModeActivationFailed("Failed to activate " + goal->command);
			}
		} else if (std::find(
				navigator_plugin_ids_.begin(),
				navigator_plugin_ids_.end(),
				current_nav_mode_) != navigator_plugin_ids_.end() &&
				std::find(
						navigator_plugin_ids_.begin(),
						navigator_plugin_ids_.end(),
						goal->command) != navigator_plugin_ids_.end()) {
			auto tmp_goal = *goal;
			loiter();
			nav_command_action_client_->async_send_goal(tmp_goal);
		} else {
			throw navigator::InvalidTransition("Navigator plugin " + goal->command + " invalid transition");
		}
	} catch (navigator::InvalidNavigator & ex) {
		result->error_code = ActionNavigatorCommandGoal::NAV_RESPONSE_INVALID_NAVIGATOR;
		nav_command_action_server_->terminate_current(result);
	} catch (navigator::InvalidTransition & ex) {
		result->error_code = ActionNavigatorCommandGoal::NAV_RESPONSE_INVALID_TRANSITION;
		nav_command_action_server_->terminate_current(result);
	} catch (navigator::NavigatorModeActivationFailed & ex) {
		result->error_code = ActionNavigatorCommandGoal::NAV_RESPONSE_NAVIGATOR_MODE_ACTIVATION_FAILED;
		nav_command_action_server_->terminate_current(result);
	}
}

template<typename T>
bool Navigator::isServerInactive(std::unique_ptr<util::SimpleActionServer<T>> & action_server)
{
	if (action_server == nullptr || !action_server->is_server_active()) {
		RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
		return true;
	}

	return false;
}

template<typename T>
bool Navigator::isCancelRequested(
		std::unique_ptr<util::SimpleActionServer<T>> & action_server)
{
	if (action_server->is_cancel_requested()) {
		RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling navigator command action.");
		action_server->terminate_all();
		return true;
	}

	return false;
}

} // namespace navigator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigator::Navigator)