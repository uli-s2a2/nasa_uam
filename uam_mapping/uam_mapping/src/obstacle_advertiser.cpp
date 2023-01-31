#include <uam_mapping/obstacle_advertiser.hpp>

using namespace uam_mapping;
using namespace std::chrono;

ObstacleAdvertiser::ObstacleAdvertiser() : Node("uam_obstacle_advertiser")
{


	// ----------------------- Publishers --------------------------
	auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();
	obstacle_array_pub_ =
			this->create_publisher<uam_mapping_msgs::msg::ObstacleArray>("uam_mapping/obstacles", qos_pub);

	// ----------------------- Subscribers --------------------------
	auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

	auto timer_callback = [this]() -> void
	{
		publish_obstacles();
	};

	timer_ = this->create_wall_timer(100ms, timer_callback);
}

void ObstacleAdvertiser::publish_obstacles()
{
	uam_mapping_msgs::msg::ObstacleArray obstacles;

	uam_mapping_msgs::msg::Obstacle obstacle1;
	uam_mapping_msgs::msg::Obstacle obstacle2;
	uam_mapping_msgs::msg::Obstacle obstacle3;
	uam_mapping_msgs::msg::Obstacle obstacle4;
	uam_mapping_msgs::msg::Obstacle obstacle5;

	obstacles.header.frame_id = "map";
	obstacles.header.stamp = this->get_clock()->now();
	obstacle1.obstacle.type = shape_msgs::msg::SolidPrimitive::BOX;
	obstacle1.obstacle_id = 0;
	obstacle1.obstacle.dimensions = {0.5, 0.5, 5.0};
	obstacle1.pose.position.x = 1.25;
	obstacle1.pose.position.y = 3.75;
	obstacle1.pose.position.z = obstacle1.obstacle.dimensions[2] / 2.0;
//
//	shapes::Box obstacle_shape(0.5, 0.5, 5.0);
//	bodies::Box obstacle1(&obstacle_shape);
//	bodies::Box obstacle2(&obstacle_shape);
//	bodies::Box obstacle3(&obstacle_shape);
//	bodies::Box obstacle4(&obstacle_shape);
//	bodies::Box obstacle5(&obstacle_shape);
//
//	Eigen::Isometry3d obstacle_pose;
//	obstacle_pose.setIdentity();
//
//	obstacle_pose.translation() = Eigen::Vector3d(1.25,3.75,obstacle_shape.size[2] / 2.0);
//	obstacle1.setPose(obstacle_pose);
//	obstacle_pose.translation() = Eigen::Vector3d(3.75,3.75,obstacle_shape.size[2] / 2.0);
//	obstacle2.setPose(obstacle_pose);
//	obstacle_pose.translation() = Eigen::Vector3d(3.75,1.25,obstacle_shape.size[2] / 2.0);
//	obstacle3.setPose(obstacle_pose);
//	obstacle_pose.translation() = Eigen::Vector3d(1.25,1.25,obstacle_shape.size[2] / 2.0);
//	obstacle4.setPose(obstacle_pose);
//	obstacle_pose.translation() = Eigen::Vector3d(2.5,2.5,obstacle_shape.size[2] / 2.0);
//	obstacle5.setPose(obstacle_pose);
//
//	obstacle1.setScale(1.1);
//	obstacle2.setScale(1.1);
//	obstacle3.setScale(1.1);
//	obstacle4.setScale(1.1);
//	obstacle5.setScale(1.1);

	obstacles.obstacles.push_back(obstacle1);
//	obstacles_.obstacles.push_back(obstacle2);
//	obstacles_.obstacles.push_back(obstacle3);
//	obstacles_.obstacles.push_back(obstacle4);
//	obstacles_.obstacles.push_back(obstacle5);
	obstacle_array_pub_->publish(obstacles);
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<uam_mapping::ObstacleAdvertiser>());
	rclcpp::shutdown();
	return 0;
}