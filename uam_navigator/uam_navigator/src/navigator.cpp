#include "navigator.hpp"
#include "uam_util/qos_profiles.hpp"
namespace uam_navigator
{
Navigator::Navigator(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("navigator", "", options)
{

}

Navigator::~Navigator()
{
}

uam_util::CallbackReturn
Navigator::on_configure(const rclcpp_lifecycle::State &state)
{
	RCLCPP_INFO(get_logger(), "Configuring");

	auto node = shared_from_this();
	navigators_.insert({NavMode::NAV_MODE_TAKEOFF, std::make_unique<Takeoff>()});
//	navigators_.insert({NavMode::NAV_MODE_LAND, std::make_unique<Land>()});
	navigators_.insert({NavMode::NAV_MODE_LOITER, std::make_unique<Loiter>()});
	navigators_.insert({NavMode::NAV_MODE_NAVIGATE_TO_POSE, std::make_unique<NavigateToPose>()});
	position_setpoint_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>("/uam_navigator/position_setpoint", uam_util::px4_qos_pub);

	navigators_[NAV_MODE_TAKEOFF]->on_configure(node, NAV_MODE_TAKEOFF);
//	navigators_[NAV_MODE_LAND]->on_configure(node, &nav_mode_muxer_, NAV_MODE_LAND);
	navigators_[NAV_MODE_LOITER]->on_configure(node, NAV_MODE_LOITER);
	navigators_[NAV_MODE_NAVIGATE_TO_POSE]->on_configure(node, NAV_MODE_NAVIGATE_TO_POSE);
	navigator_transitions_[NAV_MODE_TAKEOFF] = {NAV_MODE_LOITER, NAV_MODE_LAND};
	navigator_transitions_[NAV_MODE_NAVIGATE_TO_POSE] = {NAV_MODE_LOITER, NAV_MODE_LAND};
	navigator_transitions_[NAV_MODE_LOITER] = {NAV_MODE_NAVIGATE_TO_POSE, NAV_MODE_LAND};
	navigator_transitions_[NAV_MODE_IDLE] = {NAV_MODE_TAKEOFF};

	timer_ = node->create_wall_timer(std::chrono::milliseconds(10),
								  std::bind(&Navigator::on_loop, this));

	vehicle_odometry_sub_ =
			node->create_subscription<nav_msgs::msg::Odometry>(
					"/uam_vehicle_interface/odometry",
					uam_util::px4_qos_sub,
					[this](const nav_msgs::msg::Odometry::UniquePtr msg)
					{
						vehicle_odom_ = *msg;
					});

	navigator_command_sub_ =
			node->create_subscription<uam_navigator_msgs::msg::NavCmd>(
					"/uam_navigator/command",
					10,
					[this](const uam_navigator_msgs::msg::NavCmd::UniquePtr msg)
					{
						requested_nav_mode_ = static_cast<NavMode>(msg->nav_cmd);
						std::cout << "Requested navigator command: " << requested_nav_mode_ << std::endl;
					});

	current_nav_mode_ = NAV_MODE_IDLE;
	requested_nav_mode_ = NAV_MODE_IDLE;
	return uam_util::CallbackReturn::SUCCESS;
}

uam_util::CallbackReturn
Navigator::on_activate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(get_logger(), "Activating");

	position_setpoint_publisher_->on_activate();

	auto node = shared_from_this();
	return uam_util::CallbackReturn::SUCCESS;
}

uam_util::CallbackReturn
Navigator::on_deactivate(const rclcpp_lifecycle::State &state)
{
	RCLCPP_INFO(get_logger(), "Deactivating");

	position_setpoint_publisher_->on_deactivate();

	NavigatorMap ::iterator it;
	for (it = navigators_.begin(); it != navigators_.end(); ++it) {
		it->second->on_deactivate();
	}

	return uam_util::CallbackReturn::SUCCESS;
}

uam_util::CallbackReturn
Navigator::on_cleanup(const rclcpp_lifecycle::State &state)
{
	RCLCPP_INFO(get_logger(), "Cleaning up");

	vehicle_odometry_sub_.reset();
	position_setpoint_publisher_.reset();
	vehicle_odom_ = nav_msgs::msg::Odometry();
	timer_->reset();

	NavigatorMap ::iterator it;
	for (it = navigators_.begin(); it != navigators_.end(); ++it) {
		it->second->on_cleanup();
	}

	navigators_.clear();
	return uam_util::CallbackReturn::SUCCESS;
}

uam_util::CallbackReturn
Navigator::on_shutdown(const rclcpp_lifecycle::State &state)
{
	RCLCPP_INFO(get_logger(), "Shutting down");

	vehicle_odometry_sub_.reset();
	position_setpoint_publisher_.reset();
	timer_->reset();

	return uam_util::CallbackReturn::SUCCESS;
}

void Navigator::on_loop()
{

	auto node = shared_from_this();

	// Auto transition
	if (current_nav_mode_ != NAV_MODE_IDLE) {
		if (navigators_[current_nav_mode_]->mission_complete(vehicle_odom_)) {
			switch (current_nav_mode_) {
				case NAV_MODE_TAKEOFF:
					requested_nav_mode_ = NAV_MODE_LOITER;
				case NAV_MODE_NAVIGATE_TO_POSE:
					requested_nav_mode_ = NAV_MODE_LOITER;
				case NAV_MODE_IDLE:
				case NAV_MODE_LAND:
				case NAV_MODE_LOITER:
					break;
			}
		}
	}

	// Transition from command
	if (current_nav_mode_ != requested_nav_mode_) {
		if (std::find(
				navigator_transitions_[current_nav_mode_].begin(),
				navigator_transitions_[current_nav_mode_].end(),
				requested_nav_mode_) != navigator_transitions_[current_nav_mode_].end()) {
			bool activate_request_success = false;
			switch (requested_nav_mode_) {
				case NAV_MODE_TAKEOFF:
				case NAV_MODE_LOITER:
				{
					activate_request_success = navigators_[requested_nav_mode_]->on_activate(vehicle_odom_, nav_msgs::msg::Odometry());
					break;
				}
				case NAV_MODE_NAVIGATE_TO_POSE:
				{
					nav_msgs::msg::Odometry goal;
					goal.header.stamp = node->get_clock()->now();
					goal.header.frame_id = "map_ned";
					goal.pose.pose.position.x = 4.7;
					goal.pose.pose.position.y = 4.7;
					goal.pose.pose.position.z = -0.6;
					activate_request_success = navigators_[requested_nav_mode_]->on_activate(vehicle_odom_, goal);
					break;
				}
				case NAV_MODE_LAND:
				{
					RCLCPP_DEBUG(node->get_logger(), "Land mode not implemented yet.");
					requested_nav_mode_ = current_nav_mode_;
					break;
				}
			}
			if (activate_request_success) {
				std::cout << "Activate request success... " << std::endl;
				if (current_nav_mode_ != NAV_MODE_IDLE) {
					navigators_[current_nav_mode_]->on_deactivate();
				}
				current_nav_mode_ = requested_nav_mode_;
			}
		} else {
			RCLCPP_DEBUG(node->get_logger(), "Requested navigator flight mode invalid transition.");
			requested_nav_mode_ = current_nav_mode_;
		}
	}
	if (current_nav_mode_ != NAV_MODE_IDLE) {
		nav_msgs::msg::Odometry odom_setpoint;
		odom_setpoint = navigators_[current_nav_mode_]->compute_position_setpoint(vehicle_odom_);
		position_setpoint_publisher_->publish(odom_setpoint);
	}
}

//void Navigator::terminal_state_evaluation()
//{
//	double initial_distance;
//	double relative_distance;
//
//	auto path_ptr = prob_def_ptr->getSolutionPath()->as<og::PathGeometric>();
//	auto current_state = path_ptr->getState(current_waypoint_)->as<ob::SE3StateSpace::StateType>();
//	Eigen::Vector3d current_waypoint_vec(current_state->getX(), current_state->getY(), current_state->getZ());
//	ob::SE3StateSpace::StateType *reference_state;
//	if (current_waypoint_ + 1 == path_ptr->getStateCount()) {
//		reference_state = path_ptr->getState(current_waypoint_)->as<ob::SE3StateSpace::StateType>();
//	} else {
//		reference_state = path_ptr->getState(current_waypoint_ + 1)->as<ob::SE3StateSpace::StateType>();
//	}
//	Eigen::Vector3d reference_waypoint_vec(reference_state->getX(), reference_state->getY(), reference_state->getZ());
//	Eigen::Vector3d mav_pose(mav_local_position_.y, mav_local_position_.x, -mav_local_position_.z);
//	initial_distance = (current_waypoint_vec - reference_waypoint_vec).norm();
//	relative_distance = (mav_pose - reference_waypoint_vec).norm();
//	std::cout << "initial distance:" << initial_distance << std::endl;
//	std::cout << "relative distance:" << relative_distance << std::endl;
//	if ((relative_distance <= admissible_window_ * initial_distance) && (current_waypoint_+1 < path_ptr->getStateCount()))
//	{
//		geometry_msgs::msg::PoseStamped path_pose_stamped_msg;
//		path_pose_stamped_msg.pose.position.x = reference_waypoint_vec.x();
//		path_pose_stamped_msg.pose.position.y = reference_waypoint_vec.y();
//		path_pose_stamped_msg.pose.position.z = reference_waypoint_vec.z();
//		path_pose_stamped_msg.header.stamp = get_clock()->now();
//		explored_path_.poses.push_back(path_pose_stamped_msg);
//		current_waypoint_ += 1;
//	}
//}

}