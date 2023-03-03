#pragma once

#include <string>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_action_server.hpp"
#include "uam_util/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace uam_navigator
{

enum NavMode{
	NAV_MODE_IDLE = 0,
	NAV_MODE_TAKEOFF = 1,
	NAV_MODE_LAND = 2,
	NAV_MODE_LOITER = 3,
	NAV_MODE_NAVIGATE_TO_POSE = 4
};

class Navigator;

// Code design based on the ROS2 Nav2 project navigator
class NavigatorModeMuxer
{
public:
NavigatorModeMuxer() : current_navigator_(NAV_MODE_IDLE) {};

bool is_navigating()
{
	std::scoped_lock lock(mutex_);
	return (current_navigator_ != NAV_MODE_IDLE);
}

void start_navigating(const NavMode & navigator_mode)
{
	std::scoped_lock lock(mutex_);
	if (current_navigator_ != NAV_MODE_IDLE) {
		RCLCPP_ERROR(
				rclcpp::get_logger("NavigatorMuxer"),
				"Navigation start request while another navigation task is running.");
	}
	current_navigator_ = navigator_mode;
}

void stop_navigating(const NavMode & navigator_mode)
{
	std::scoped_lock lock(mutex_);
	if (current_navigator_ != navigator_mode) {
		RCLCPP_ERROR(
				rclcpp::get_logger("NavigatorMuxer"),
				"Navigation stop request sent while another navigation task is running.");
	} else {
		current_navigator_ = NAV_MODE_IDLE;
	}
}
protected:
NavMode current_navigator_;
std::mutex mutex_;
}; // class NavigatorModeMuxer


class NavigatorModeBase
{
public:
	using UniquePtr = std::unique_ptr<NavigatorModeBase>;

	NavigatorModeBase() = default;
	~NavigatorModeBase() = default;

	virtual bool on_configure(
			const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
			NavigatorModeMuxer * nav_mode_muxer,
			const NavMode & nav_mode) = 0;
	virtual bool on_activate(const nav_msgs::msg::Odometry & start, const nav_msgs::msg::Odometry & goal) = 0;
	virtual bool on_deactivate() = 0;
	virtual bool on_cleanup() = 0;
	virtual nav_msgs::msg::Odometry compute_position_setpoint(const nav_msgs::msg::Odometry & current_odom) = 0;
	virtual bool mission_complete(const nav_msgs::msg::Odometry & current_odom) {(void)current_odom;return false;}
}; // class NavigatorModeMuxer


class NavigatorMode : public NavigatorModeBase
{
public:
	NavigatorMode() = default;
	~NavigatorMode() = default;

protected:
	bool on_configure(
			const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
			NavigatorModeMuxer * nav_mode_muxer,
			const NavMode & nav_mode) final
	{
		node_ = parent;
		nav_mode_muxer_ = nav_mode_muxer;
		nav_mode_ = nav_mode;

		bool ok = true;
		if(!configure()) {
			ok = false;
		}
		return ok;
	}

	bool on_activate(const nav_msgs::msg::Odometry & start, const nav_msgs::msg::Odometry & goal) final
	{
		bool ok = true;
		if (nav_mode_muxer_->is_navigating()) {
			return false;
		}
		if (!activate(start, goal)) {
			return false;
		}
		nav_mode_muxer_->start_navigating(nav_mode_);
		return ok;
	}

	bool on_deactivate() final
	{
		bool ok = true;
		if(!deactivate()) {
			return false;
		}
		nav_mode_muxer_->stop_navigating(nav_mode_);
		return ok;
	}

	bool on_cleanup() final
	{
		bool ok = true;
		if(!cleanup()) {
			ok = false;
		}
		nav_mode_muxer_ = nullptr;
		return ok;
	}

	virtual bool configure() = 0;
	virtual bool activate(const nav_msgs::msg::Odometry & start, const nav_msgs::msg::Odometry & goal) = 0;
	virtual bool deactivate() = 0;
	virtual bool cleanup() = 0;

	NavigatorModeMuxer * nav_mode_muxer_{nullptr};
	NavMode nav_mode_{NAV_MODE_IDLE};
	rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
}; // class NavigatorMode

} // namespace uam_navigator