#include <uam_visualization/visualization.hpp>


using namespace uam_visualization;

Visualization::Visualization() : Node("uam_visualization")
{
	// ----------------------- Publishers --------------------------
	obstacle_marker_pub_ =
			this->create_publisher<visualization_msgs::msg::MarkerArray>("uam_visualization/obstacle_markers", 10);
	explored_path_pub_ =
			this->create_publisher<nav_msgs::msg::Path>("uam_visualization/explored_path", 10);
	frontier_path_pub_ =
			this->create_publisher<nav_msgs::msg::Path>("uam_visualization/frontier_path", 10);

	// ----------------------- Subscribers --------------------------
	obstacle_array_sub_ =
			this->create_subscription<uam_mapping_msgs::msg::ObstacleArray>(
					"uam_mapping/obstacles", 10, std::bind(&Visualization::publishObstacleMarkers, this, std::placeholders::_1));
}

void Visualization::publishObstacleMarkers(const uam_mapping_msgs::msg::ObstacleArray::SharedPtr msg)
{
	visualization_msgs::msg::MarkerArray obstacle_marker_array;
	for(auto & obstacle : msg->obstacles)
	{
		visualization_msgs::msg::Marker obstacle_marker;
		obstacle_marker.header.stamp = this->get_clock()->now();
		obstacle_marker.header.frame_id = msg->header.frame_id;
		obstacle_marker.id = obstacle.obstacle_id;
		obstacle_marker.pose.position = obstacle.pose.position;
		obstacle_marker.color.r = 1.0;
		obstacle_marker.color.g = 0.0;
		obstacle_marker.color.b = 0.0;
		obstacle_marker.color.a = 1.0;
		obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
		obstacle_marker.ns = "obstacles";

		switch (obstacle.obstacle.type) {
			case shape_msgs::msg::SolidPrimitive::BOX:
				obstacle_marker.type = visualization_msgs::msg::Marker::CUBE;
				obstacle_marker.scale.x = obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
				obstacle_marker.scale.y = obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
				obstacle_marker.scale.z = obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];
				break;
			case shape_msgs::msg::SolidPrimitive::SPHERE:
				obstacle_marker.type = visualization_msgs::msg::Marker::SPHERE;
				obstacle_marker.scale.x = 2 * obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS];
				obstacle_marker.scale.y = 2 * obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS];
				obstacle_marker.scale.z = 2 * obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS];
				break;
			case shape_msgs::msg::SolidPrimitive::CYLINDER:
				obstacle_marker.type = visualization_msgs::msg::Marker::CYLINDER;
				obstacle_marker.scale.x = 2 * obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS];
				obstacle_marker.scale.y = 2 * obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS];
				obstacle_marker.scale.z = obstacle.obstacle.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT];
		}

		obstacle_marker_array.markers.push_back(obstacle_marker);
	}
	obstacle_marker_pub_->publish(obstacle_marker_array);
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<uam_visualization::Visualization>());
	rclcpp::shutdown();
	return 0;
}