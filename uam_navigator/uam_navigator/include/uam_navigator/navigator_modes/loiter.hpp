#include "uam_navigator/navigator_modes/navigator_mode.hpp"

namespace uam_navigator
{

class Loiter: public NavigatorMode
{
public:
	Loiter();
	~Loiter();

protected:
	bool configure() override;
	bool activate(const nav_msgs::msg::Odometry & start, const nav_msgs::msg::Odometry & goal) override;
	bool deactivate() override;
	bool cleanup() override;
	nav_msgs::msg::Odometry compute_position_setpoint(const nav_msgs::msg::Odometry & current_odom) override;

	geometry_msgs::msg::PoseStamped loiter_position_;
};

}