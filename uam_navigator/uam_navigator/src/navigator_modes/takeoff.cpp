#include "takeoff.hpp"


namespace uam_navigator
{

Takeoff::Takeoff()
{
}
Takeoff::~Takeoff()
{
}

bool Takeoff::configure()
{
	auto node = node_.lock();

	RCLCPP_INFO(node->get_logger(), "Configuring");

	node->declare_parameter("takeoff.altitude", 0.6);
	node->get_parameter("takeoff.altitude", takeoff_altitude_);
	node->declare_parameter("takeoff.position_tolerance", 0.05);
	node->get_parameter("takeoff.position_tolerance", takeoff_position_tolerance_);
	node->declare_parameter("takeoff.velocity_tolerance", 0.3);
	node->get_parameter("takeoff.velocity_tolerance", takeoff_velocity_tolerance_);
	return true;
}

bool Takeoff::activate(const nav_msgs::msg::Odometry & start, const nav_msgs::msg::Odometry & goal)
{
	auto node = node_.lock();
	RCLCPP_INFO(node->get_logger(), "Activating takeoff flight mode");

	(void)goal;
	takeoff_position_.header.frame_id = start.header.frame_id;
	takeoff_position_.header.stamp = start.header.stamp;
	takeoff_position_.pose.position.x = start.pose.pose.position.x;
	takeoff_position_.pose.position.y = start.pose.pose.position.y;
	takeoff_position_.pose.position.z = -takeoff_altitude_;
	return true;
}

bool Takeoff::deactivate()
{
	auto node = node_.lock();
	RCLCPP_INFO(node->get_logger(), "Deactivating takeoff flight mode");
	takeoff_position_ = geometry_msgs::msg::PoseStamped();
	return true;
}

bool Takeoff::cleanup()
{
	node_.reset();
	return true;
}

nav_msgs::msg::Odometry
Takeoff::compute_position_setpoint(const nav_msgs::msg::Odometry & current_odom)
{
	auto node = node_.lock();
	(void)current_odom;
	nav_msgs::msg::Odometry odom;
	odom.header.frame_id = takeoff_position_.header.frame_id;
	odom.header.stamp = node->get_clock()->now();
	odom.child_frame_id = "baselink_frd";
	odom.pose.pose = takeoff_position_.pose;

	return odom;
}

bool Takeoff::mission_complete(const nav_msgs::msg::Odometry & current_odom)
{
	bool mission_complete = false;
	if (std::hypot(
			current_odom.pose.pose.position.x - takeoff_position_.pose.position.x,
			current_odom.pose.pose.position.y - takeoff_position_.pose.position.y,
			current_odom.pose.pose.position.z - takeoff_position_.pose.position.z) <= takeoff_position_tolerance_
		&&
		std::hypot(
			current_odom.twist.twist.linear.x,
			current_odom.twist.twist.linear.y,
			current_odom.twist.twist.linear.z) <= takeoff_velocity_tolerance_) {
		mission_complete = true;
	}
	return mission_complete;
}

} // namespace uam_navigator