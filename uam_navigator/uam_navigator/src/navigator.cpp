// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

// other libraries
#include <uam_navigator/navigator.hpp>



using namespace uam_navigator;

Navigator::Navigator(): rclcpp::Node("uam_navigator")
{

}

void Navigator::terminal_state_evaluation()
{
	double initial_distance;
	double relative_distance;

	auto path_ptr = prob_def_ptr->getSolutionPath()->as<og::PathGeometric>();
	auto current_state = path_ptr->getState(current_waypoint_)->as<ob::SE3StateSpace::StateType>();
	Eigen::Vector3d current_waypoint_vec(current_state->getX(), current_state->getY(), current_state->getZ());
	ob::SE3StateSpace::StateType *reference_state;
	if (current_waypoint_ + 1 == path_ptr->getStateCount()) {
		reference_state = path_ptr->getState(current_waypoint_)->as<ob::SE3StateSpace::StateType>();
	} else {
		reference_state = path_ptr->getState(current_waypoint_ + 1)->as<ob::SE3StateSpace::StateType>();
	}
	Eigen::Vector3d reference_waypoint_vec(reference_state->getX(), reference_state->getY(), reference_state->getZ());
	Eigen::Vector3d mav_pose(mav_local_position_.y, mav_local_position_.x, -mav_local_position_.z);
	initial_distance = (current_waypoint_vec - reference_waypoint_vec).norm();
	relative_distance = (mav_pose - reference_waypoint_vec).norm();
	std::cout << "initial distance:" << initial_distance << std::endl;
	std::cout << "relative distance:" << relative_distance << std::endl;
	if ((relative_distance <= admissible_window_ * initial_distance) && (current_waypoint_+1 < path_ptr->getStateCount()))
	{
		geometry_msgs::msg::PoseStamped path_pose_stamped_msg;
		path_pose_stamped_msg.pose.position.x = reference_waypoint_vec.x();
		path_pose_stamped_msg.pose.position.y = reference_waypoint_vec.y();
		path_pose_stamped_msg.pose.position.z = reference_waypoint_vec.z();
		path_pose_stamped_msg.header.stamp = get_clock()->now();
		explored_path_.poses.push_back(path_pose_stamped_msg);
		current_waypoint_ += 1;
	}
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<uam_navigator::Navigator>());
	rclcpp::shutdown();
	return 0;
}