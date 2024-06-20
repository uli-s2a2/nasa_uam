#ifndef RVIZ2_GUI_HPP
#define RVIZ2_GUI_HPP

#include <QtWidgets>
#include <QBasicTimer>

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"

#include "navigator_msgs/action/navigator_command.hpp"
#include "navigator_msgs/msg/navigator_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/notify_setpoints_stop.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
class QPushButton;
class QComboBox;

namespace crazyflie_rviz2_gui
{

class ScenarioWorker : public QObject {
    Q_OBJECT
public:
    ScenarioWorker(int scenario) {scenario_ = scenario;}

public Q_SLOTS:
    void runScenario();

signals:
    void finished();
	void cfTakeoff(int index);
	void cfLand(int index);
	void cfGoTo(int index, double x, double y, double z, double duration, bool relative);
	void cfRequestPlan();

private:
	int scenario_;
};

class CrazyflieInterfacePanel : public rviz_common::Panel
{
	Q_OBJECT
public:
	explicit CrazyflieInterfacePanel(QWidget * parent = 0);
	virtual ~CrazyflieInterfacePanel();

	void onInitialize() override;
	void save(rviz_common::Config config) const override;
	void load(const rviz_common::Config & config) override;

public Q_SLOTS:
	void sendTakeoffLandCommand();
	void navigatorTakeoff();
	void navigatorLand();
	void runScenarioThread();
	void updateScenarioVisuals(int index);
	void cfAllTakeoff();
	void cfTakeoff(int index);
	void cfLand(int index);
	void cfAllLand();
	void cfGoTo(int index, double x, double y, double z, double duration, bool relative);
	void cfLoiter();
	void cfCommanderRelaxPriority();
	void commandNavigatorGoalPose(double x, double y);

private:
	using ActionNavigatorCommand = navigator_msgs::action::NavigatorCommand;
	using ActionNavigatorCommandGoal = ActionNavigatorCommand::Goal;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_odometry_sub_;
	rclcpp::Subscription<navigator_msgs::msg::NavigatorStatus>::SharedPtr navigator_status_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr navigator_goal_command_sub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vports_marker_pub_;

	rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr all_takeoff_client_;
	std::vector<rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr> cf_takeoff_clients_;

	rclcpp::Client<crazyflie_interfaces::srv::Land>::SharedPtr all_land_client_;
	std::vector<rclcpp::Client<crazyflie_interfaces::srv::Land>::SharedPtr> cf_land_clients_;

	std::vector<rclcpp::Client<crazyflie_interfaces::srv::GoTo>::SharedPtr> cf_go_to_clients_;

	rclcpp::Client<crazyflie_interfaces::srv::NotifySetpointsStop>::SharedPtr notify_setpoints_stop_client_;
	// rclcpp::Subscription<px4_bridge_msgs::msg::VehicleInterfaceStatus>::SharedPtr vehicle_interface_status_sub_;
	// rclcpp::Publisher<px4_bridge_msgs::msg::VehicleInterfaceCommand>::SharedPtr vehicle_interface_command_pub_;

	std::string base_frame_;

	rclcpp_action::Client<ActionNavigatorCommand>::SharedPtr nav_command_action_client_;
	nav_msgs::msg::Odometry vehicle_odom_;

	QWidget * widget_;

	// Vehicle Buttons
	// Arm / Disarm
	QPushButton * arm_disarm_button_{nullptr};
	QState * armed_{nullptr};
	QState * disarmed_{nullptr};
	QStateMachine arm_disarm_state_machine_;

	// Navigator Mode
	QState * takeoff_{nullptr};
	QState * loiter_{nullptr};
	QState * navigate_{nullptr};
	QState * land_{nullptr};
	QState * idle_{nullptr};
	QStateMachine navigator_mode_state_machine_;
	QPushButton * takeoff_land_button_{nullptr};

	// Scenario selector
	QPushButton * scenario_push_button_{nullptr};
	QComboBox * scenario_selector_box_{nullptr};

	// Vehicle Status
	QLabel * label_navigator_mode_;
	QLabel * label_vehicle_status_;

	// Vehicle Odometry
	QLabel * label_frame_id_odometry_;
	QLabel * label_position_odometry_x_;
	QLabel * label_position_odometry_y_;
	QLabel * label_position_odometry_z_;
	QLabel * label_velocity_odometry_x_;
	QLabel * label_velocity_odometry_y_;
	QLabel * label_velocity_odometry_z_;

	void updateOdometry(nav_msgs::msg::Odometry::UniquePtr msg);
	void updateNavigatorStatus(navigator_msgs::msg::NavigatorStatus::UniquePtr msg);
	void goalPoseCallback(geometry_msgs::msg::PoseStamped::UniquePtr msg);
	// void updateVehicleInterfaceStatus(px4_bridge_msgs::msg::VehicleInterfaceStatus::UniquePtr msg);
	void updateVehicleStatus();
};

struct StringEvent : public QEvent
{
	StringEvent(const QString &val)
			: QEvent(QEvent::Type(QEvent::User+1)),
			  value(val) {}

	QString value;
};

class StringTransition : public QAbstractTransition
{
public:
	StringTransition(const QString &value)
			: m_value(value) {}

protected:
	bool eventTest(QEvent *e) override
	{
		if (e->type() != QEvent::Type(QEvent::User+1)) // StringEvent
			return false;
		StringEvent *se = static_cast<StringEvent*>(e);
		return (m_value == se->value);
	}

	virtual void onTransition(QEvent *) {}

private:
	QString m_value;
};

}


#endif //RVIZ2_GUI_HPP
