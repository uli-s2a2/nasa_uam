#include <functional>
#include <memory>
#include <thread>
#include "uam_navigator/navigator_modes/navigator_mode.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "uam_planner_msgs/action/compute_path_to_pose.hpp"


namespace uam_navigator
{

class NavigateToPose: public NavigatorMode
{
public:
	using ActionT = uam_planner_msgs::action::ComputePathToPose;
	using GoalHandleActionT = rclcpp_action::ClientGoalHandle<ActionT>;
	NavigateToPose();
	~NavigateToPose();

protected:
	bool configure() override;
	bool activate(const nav_msgs::msg::Odometry & start, const nav_msgs::msg::Odometry & goal) override;
	bool deactivate() override;
	bool cleanup() override;
	nav_msgs::msg::Odometry compute_position_setpoint(const nav_msgs::msg::Odometry & current_odom) override;
	bool mission_complete(const nav_msgs::msg::Odometry & current_odom) override;
	void goal_response_callback(const GoalHandleActionT::SharedPtr & goal_handle);
	void result_callback(const GoalHandleActionT::WrappedResult & result);
	void update_waypoint(const nav_msgs::msg::Odometry & current_odom);

	double waypoint_position_tolerance_;
	double goal_position_tolerance_;
	double goal_velocity_tolerance_;
	std::string planner_name_;
	geometry_msgs::msg::PoseStamped goal_pose_;
	geometry_msgs::msg::PoseStamped start_pose_;
	nav_msgs::msg::Path path_;
	std::uint8_t current_path_waypoint_;
	bool path_received_{false};
	bool path_requested_{false};
	rclcpp_action::Client<ActionT>::SharedPtr planner_client_ptr_;

};

}