#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometric_shapes/bodies.h>
#include <uam_mapping_msgs/msg/obstacle.hpp>
#include <uam_mapping_msgs/msg/obstacle_array.hpp>

namespace uam_mapping
{
class ObstacleAdvertiser: public rclcpp::Node
{
public:
	ObstacleAdvertiser();
	virtual ~ObstacleAdvertiser() = default;

private:

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<uam_mapping_msgs::msg::ObstacleArray>::SharedPtr obstacle_array_pub_;

	void publish_obstacles();
};
}