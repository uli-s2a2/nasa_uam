// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/clock.hpp"

// other libraries
#include <Eigen/Dense>
#include <chrono>
#include "navigator_mode.hpp"
#include "land.hpp"
#include "loiter.hpp"
#include "takeoff.hpp"
#include "navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "uam_navigator_msgs/msg/nav_cmd.hpp"

namespace uam_navigator
{

//using nav_state = std::variant()

class Navigator: public rclcpp_lifecycle::LifecycleNode
{
public:
	explicit Navigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
	~Navigator();

	using NavigatorMap = std::unordered_map<NavMode, NavigatorModeBase::UniquePtr>;
	using TransitionMap = std::unordered_map<NavMode, std::vector<NavMode>>;

	uam_util::CallbackReturn  on_configure(const rclcpp_lifecycle::State & state) override;
	uam_util::CallbackReturn  on_activate(const rclcpp_lifecycle::State & state) override;
	uam_util::CallbackReturn  on_deactivate(const rclcpp_lifecycle::State & state) override;
	uam_util::CallbackReturn  on_cleanup(const rclcpp_lifecycle::State & state) override;
	uam_util::CallbackReturn  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
	// ----------------------- Publishers --------------------------
	rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr position_setpoint_publisher_;


	// ----------------------- Subscribers --------------------------
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_odometry_sub_;
	rclcpp::Subscription<uam_navigator_msgs::msg::NavCmd>::SharedPtr navigator_command_sub_;

	// ROS2 Variables
	rclcpp::TimerBase::SharedPtr timer_;
	nav_msgs::msg::Odometry vehicle_odom_;

	// Class Variables
	TransitionMap navigator_transitions_;
	NavigatorMap navigators_;
	NavigatorModeMuxer nav_mode_muxer_;
	NavMode current_nav_mode_;
	NavMode requested_nav_mode_;
	bool nav_mode_complete_{false};

	// Class methods
	void on_loop();
}; // Class RRTX
}

