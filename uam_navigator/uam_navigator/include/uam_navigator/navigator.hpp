// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

// other libraries
#include <Eigen/Dense>
#include <chrono>

namespace uam_navigator
{

class Navigator: public rclcpp::Node
{
public:
	Navigator();
	virtual ~Navigator() = default;
private:
	// ----------------------- Publishers --------------------------


	// ----------------------- Subscribers --------------------------

	// Class Variables

	// Class methods
	void terminal_state_evaluation();

}; // Class RRTX
}

