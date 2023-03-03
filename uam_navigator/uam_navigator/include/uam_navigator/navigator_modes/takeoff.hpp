#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navigator_mode.hpp"
namespace uam_navigator
{

class Takeoff: public NavigatorMode
{
public:
	Takeoff();
	~Takeoff();

protected:
	bool configure() override;
	bool activate(const nav_msgs::msg::Odometry & start, const nav_msgs::msg::Odometry & goal) override;
	bool deactivate() override;
	bool cleanup() override;
	nav_msgs::msg::Odometry compute_position_setpoint(const nav_msgs::msg::Odometry & current_odom) override;
	bool mission_complete(const nav_msgs::msg::Odometry & current_odom) override;

	double takeoff_altitude_;
	double takeoff_position_tolerance_;
	double takeoff_velocity_tolerance_;
	geometry_msgs::msg::PoseStamped takeoff_position_;
};

}
