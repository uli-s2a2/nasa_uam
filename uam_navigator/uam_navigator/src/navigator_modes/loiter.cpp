#include "loiter.hpp"


namespace uam_navigator
{


Loiter::Loiter()
{
}

Loiter::~Loiter()
{
}

bool Loiter::configure()
{
	auto node = node_.lock();

	RCLCPP_INFO(node->get_logger(), "Configuring");

	return true;
}

bool Loiter::activate(const nav_msgs::msg::Odometry & start, const nav_msgs::msg::Odometry & goal)
{
	auto node = node_.lock();
	RCLCPP_INFO(node->get_logger(), "Activating loiter flight mode");

	(void)goal;
	loiter_position_.header.frame_id = start.header.frame_id;
	loiter_position_.header.stamp = start.header.stamp;
	loiter_position_.pose.position.x = start.pose.pose.position.x;
	loiter_position_.pose.position.y = start.pose.pose.position.y;
	loiter_position_.pose.position.z = start.pose.pose.position.z;
	return true;
}

bool Loiter::deactivate()
{
	auto node = node_.lock();
	RCLCPP_INFO(node->get_logger(), "Deactivating loiter flight mode");
	loiter_position_ = geometry_msgs::msg::PoseStamped();
	return true;
}

bool Loiter::cleanup()
{
	node_.reset();
	return true;
}

nav_msgs::msg::Odometry
Loiter::compute_position_setpoint(const nav_msgs::msg::Odometry & current_odom)
{
	auto node = node_.lock();
	(void)current_odom;
	nav_msgs::msg::Odometry odom;
	odom.header.frame_id = loiter_position_.header.frame_id;
	odom.header.stamp = node->get_clock()->now();
	odom.child_frame_id = "baselink_frd";
	odom.pose.pose = loiter_position_.pose;
	return odom;
}


}