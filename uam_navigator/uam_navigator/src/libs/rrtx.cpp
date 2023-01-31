#include <uam_navigator/rrtx.hpp>


namespace ob = ompl::base;
namespace og = ompl::geometric;


#define RRTX_CALLBACK_RATE_MS 10
#define RRTX_PLANNER_MAX_SOLVE_TIME_S 1.0
#define RRTX_ARENA_X_BOUNDS_LOW 0.0
#define RRTX_ARENA_X_BOUNDS_HIGH 5.0
#define RRTX_ARENA_Y_BOUNDS_LOW 0.0
#define RRTX_ARENA_Y_BOUNDS_HIGH 5.0
#define RRTX_ARENA_Z_BOUNDS_LOW  0
#define RRTX_ARENA_Z_BOUNDS_HIGH 5
#define RRTX_ADMISSIBLE_WINDOW 0.1


RRTX::RRTX()
{
	space_ptr = std::make_shared<ob::SE3StateSpace>();
	ob::RealVectorBounds bounds(3);
	bounds.setLow(0, RRTX_ARENA_X_BOUNDS_LOW);
	bounds.setHigh(0, RRTX_ARENA_X_BOUNDS_HIGH);
	bounds.setLow(1, RRTX_ARENA_Y_BOUNDS_LOW);
	bounds.setHigh(1, RRTX_ARENA_Y_BOUNDS_HIGH);
	bounds.setLow(2, RRTX_ARENA_Z_BOUNDS_LOW);
	bounds.setHigh(2, RRTX_ARENA_Z_BOUNDS_HIGH);
	space_ptr->setBounds(bounds);
	space_ptr->setValidSegmentCountFactor(100);
	space_info_ptr = std::make_shared<ob::SpaceInformation>(space_ptr);
	space_info_ptr->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });
	space_info_ptr->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(space_info_ptr));
	space_info_ptr->setStateValidityCheckingResolution(0.01);
	prob_def_ptr = std::make_shared<ob::ProblemDefinition>(space_info_ptr);
	rrtxstatic_ptr = std::make_shared<og::RRTXstatic>(space_info_ptr);
	planner_ptr = std::static_pointer_cast<ob::Planner>(rrtxstatic_ptr);
	planner_ptr->setProblemDefinition(prob_def_ptr);

	//TODO: Temporarily hardcoding for first test.
	Eigen::Vector3d start(0.0, 0.0, 1.0);
	Eigen::Vector3d goal(4.8,4.8,1.0);



	plan(start, goal);

//	geometry_msgs::msg::PoseStamped path_pose_stamped_msg;
//	path_pose_stamped_msg.pose.position.x = start.x();
//	path_pose_stamped_msg.pose.position.y = start.y();
//	path_pose_stamped_msg.pose.position.z = start.z();
//	path_pose_stamped_msg.header.stamp = get_clock()->now();
//	explored_path_.poses.push_back(path_pose_stamped_msg);
//	explored_path_.header.frame_id = "map";

	admissible_window_ = RRTX_ADMISSIBLE_WINDOW;

	// ----------------------- Publishers --------------------------
//		obstacle_marker_pub_ =
//				this->create_publisher<visualization_msgs::msg::MarkerArray>("RRTX/obstacle_markers", qos_pub);
//	explored_path_pub_ =
//			this->create_publisher<nav_msgs::msg::Path>("RRTX/explored_path", qos_pub);
//	frontier_path_pub_ =
//			this->create_publisher<nav_msgs::msg::Path>("RRTX/frontier_path", qos_pub);
//	odom_ref_pub_ =
//			this->create_publisher<nav_msgs::msg::Odometry>( "/RRTX/reference_trajectory", qos_pub);



	if (planner_status_)
	{
		terminal_state_evaluation();
		publish_waypoint();
	}
}


void RRTX::plan(Eigen::Vector3d start, Eigen::Vector3d goal)
{
	ob::ScopedState<ob::SE3StateSpace> start_state(space_ptr);
	start_state->setXYZ(start.x(), start.y(), start.z());
	start_state->rotation().setIdentity();

	ob::ScopedState<ob::SE3StateSpace> goal_state(space_ptr);
	goal_state->setXYZ(goal.x(), goal.y(), goal.z());
	goal_state->rotation().setIdentity();

	prob_def_ptr->setStartAndGoalStates(start_state, goal_state);
	planner_ptr->setup();

	space_info_ptr->printSettings(std::cout);
	prob_def_ptr->print(std::cout);

	planner_status_ = planner_ptr->ob::Planner::solve(RRTX_PLANNER_MAX_SOLVE_TIME_S);

	prob_def_ptr->print(std::cout);
}

//void RRTX::publish_paths()
//{
//	nav_msgs::msg::Path frontier_path;
//	geometry_msgs::msg::PoseStamped path_pose_stamped_msg;
//	auto path_ptr = prob_def_ptr->getSolutionPath()->as<og::PathGeometric>();
//
//	frontier_path.poses.reserve(path_ptr->getStateCount() - current_waypoint_);
//	for(size_t i = current_waypoint_; i < path_ptr->getStateCount(); i++)
//	{
//		auto state = path_ptr->getState(i)->as<ob::SE3StateSpace::StateType>();
//		path_pose_stamped_msg.pose.position.x = state->getX();
//		path_pose_stamped_msg.pose.position.y = state->getY();
//		path_pose_stamped_msg.pose.position.z = state->getZ();
//		frontier_path.poses.push_back(path_pose_stamped_msg);
//	}
//
//	frontier_path.header.stamp = get_clock()->now();
//	frontier_path.header.frame_id = "map";
//	explored_path_pub_->publish(explored_path_);
//	frontier_path_pub_->publish(frontier_path);
//}

bool RRTX::isStateValid(const ob::State *state) const
{
	const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

	Eigen::Vector3d node(se3state->getX(),se3state->getY(),se3state->getZ());

	bool collision = false;
	for(const bodies::Box& obstacle : obstacles_)
	{
		collision |= obstacle.containsPoint(node);
	}

	return (space_info_ptr->satisfiesBounds(se3state) && (!collision));
}