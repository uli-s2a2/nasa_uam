#include "crazyflie_rviz2_gui/crazyflie_rviz2_gui.hpp"

#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QCheckBox>
#include <QtUiTools/QtUiTools>

#include <ctype.h>
#include <memory>
#include <vector>
#include <utility>
#include <chrono>
#include <string>

#include "rviz_common/display_context.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

namespace crazyflie_rviz2_gui
{

CrazyflieInterfacePanel::CrazyflieInterfacePanel(QWidget * parent) : Panel(parent)
{
	std::string directory = ament_index_cpp::get_package_share_directory("crazyflie_rviz2_gui").append("/resource/gui.ui");
	QFile file(directory.c_str());
	file.open(QIODevice::ReadOnly);
	QUiLoader loader;
	widget_ = loader.load(&file, parent);
	file.close();

	label_navigator_mode_      = widget_->findChild<QLabel*>("navigator_mode_msg");
	label_frame_id_odometry_   = widget_->findChild<QLabel*>("frame_id_odometry_msg");
	label_position_odometry_x_ = widget_->findChild<QLabel*>("position_odometry_msg_x");
	label_position_odometry_y_ = widget_->findChild<QLabel*>("position_odometry_msg_y");
	label_position_odometry_z_ = widget_->findChild<QLabel*>("position_odometry_msg_z");
	label_velocity_odometry_x_ = widget_->findChild<QLabel*>("velocity_odometry_msg_x");
	label_velocity_odometry_y_ = widget_->findChild<QLabel*>("velocity_odometry_msg_y");
	label_velocity_odometry_z_ = widget_->findChild<QLabel*>("velocity_odometry_msg_z");

	// Arm / Disarm
	arm_disarm_button_ = widget_->findChild<QPushButton*>("arm_disarm_button");

	disarmed_ = new QState();
	disarmed_->setObjectName("disarmed");
	disarmed_->assignProperty(arm_disarm_button_, "text", "Arm");

	armed_ = new QState();
	armed_->setObjectName("armed");
	armed_->assignProperty(arm_disarm_button_, "text", "Disarm");

	arm_disarm_state_machine_.addState(armed_);
	arm_disarm_state_machine_.addState(disarmed_);
	arm_disarm_state_machine_.setInitialState(disarmed_);
	arm_disarm_state_machine_.start();

	// Navigator Modes
	takeoff_land_button_ = widget_->findChild<QPushButton*>("takeoff_land_button");

	idle_ = new QState();
	idle_->setObjectName("Idle");
	idle_->assignProperty(takeoff_land_button_, "text", "Takeoff");
	idle_->assignProperty(takeoff_land_button_,"enabled", true);
	idle_->assignProperty(arm_disarm_button_,"enabled", true);
	idle_->assignProperty(label_navigator_mode_, "text", "Idle");

	takeoff_ = new QState();
	takeoff_->setObjectName("Takeoff");
	takeoff_->assignProperty(takeoff_land_button_, "text", "Land");
	takeoff_->assignProperty(takeoff_land_button_,"enabled", true);
	takeoff_->assignProperty(arm_disarm_button_,"enabled", false);
	takeoff_->assignProperty(label_navigator_mode_, "text", "Takeoff");

	loiter_ = new QState();
	loiter_->setObjectName("Loitering");
	loiter_->assignProperty(takeoff_land_button_, "text", "Land");
	loiter_->assignProperty(takeoff_land_button_,"enabled", true);
	loiter_->assignProperty(arm_disarm_button_,"enabled", false);
	loiter_->assignProperty(label_navigator_mode_, "text", "Loiter");

	navigate_ = new QState();
	navigate_->setObjectName("Navigating");
	navigate_->assignProperty(takeoff_land_button_, "text", "Land");
	navigate_->assignProperty(takeoff_land_button_,"enabled", true);
	navigate_->assignProperty(arm_disarm_button_,"enabled", false);
	navigate_->assignProperty(label_navigator_mode_, "text", "Navigating");

	land_ = new QState();
	land_->setObjectName("Landing");
	land_->assignProperty(takeoff_land_button_, "text", "Land");
	land_->assignProperty(takeoff_land_button_,"enabled", true);
	land_->assignProperty(arm_disarm_button_,"enabled", false);
	land_->assignProperty(label_navigator_mode_, "text", "Landing");

	QObject::connect(takeoff_, &QState::entered, this, &CrazyflieInterfacePanel::cfTakeoff);
	QObject::connect(land_, &QState::entered, this, &CrazyflieInterfacePanel::cfLand);
	QObject::connect(loiter_, &QState::entered, this, &CrazyflieInterfacePanel::cfLoiter);
	QObject::connect(navigate_, &QState::exited, this, &CrazyflieInterfacePanel::cfCommanderRelaxPriority);

	StringTransition *t_idle_to_takeoff = new StringTransition("Takeoff");
	StringTransition *t_takeoff_to_loiter = new StringTransition("Loiter");
	StringTransition *t_takeoff_to_land = new StringTransition("Land");
	StringTransition *t_loiter_to_navigate = new StringTransition("Navigate");
	StringTransition *t_loiter_to_land = new StringTransition("Land");
	StringTransition *t_navigate_to_land = new StringTransition("Land");
	StringTransition *t_navigate_to_loiter = new StringTransition("Loiter");
	StringTransition *t_land_to_idle = new StringTransition("Idle");

	t_idle_to_takeoff->setTargetState(takeoff_);
	t_takeoff_to_loiter->setTargetState(loiter_);
	t_takeoff_to_land->setTargetState(land_);
	t_loiter_to_navigate->setTargetState(navigate_);
	t_loiter_to_land->setTargetState(land_);
	t_navigate_to_loiter->setTargetState(loiter_);
	t_navigate_to_land->setTargetState(land_);
	t_land_to_idle->setTargetState(idle_);
	
	idle_->addTransition(t_idle_to_takeoff);
	takeoff_->addTransition(t_takeoff_to_land);
	takeoff_->addTransition(t_takeoff_to_loiter);
	loiter_->addTransition(t_loiter_to_land);
	loiter_->addTransition(t_loiter_to_navigate);
	navigate_->addTransition(t_navigate_to_land);
	navigate_->addTransition(t_navigate_to_loiter);
	land_->addTransition(t_land_to_idle);
	navigator_mode_state_machine_.addState(idle_);
	navigator_mode_state_machine_.addState(takeoff_);
	navigator_mode_state_machine_.addState(loiter_);
	navigator_mode_state_machine_.addState(navigate_);
	navigator_mode_state_machine_.addState(land_);
	navigator_mode_state_machine_.setInitialState(idle_);
	navigator_mode_state_machine_.start();

	// Kill Switch
	kill_switch_button_ = widget_->findChild<QPushButton*>("kill_switch_button");
	kill_switch_active_ = new QState();
	kill_switch_active_->setObjectName("kill_switch_active");
	kill_switch_active_->assignProperty(kill_switch_button_, "text", "Kill Switch ON");
	kill_switch_active_->assignProperty(kill_switch_button_,"enabled", true);
	kill_switch_active_->assignProperty(kill_switch_button_, "styleSheet", "background-color:rgb(0,255,0);");

	kill_switch_inactive_ = new QState();
	kill_switch_inactive_->setObjectName("kill_switch_inactive");
	kill_switch_inactive_->assignProperty(kill_switch_button_, "text", "Kill Switch OFF");
	kill_switch_inactive_->assignProperty(kill_switch_button_,"enabled", true);
	kill_switch_inactive_->assignProperty(kill_switch_button_, "styleSheet", "background-color:rgb(255,0,0);");

	StringTransition *t_kill_switch_inactive_to_active = new StringTransition("kill_switch_active");
	StringTransition *t_kill_switch_active_to_inactive = new StringTransition("kill_switch_inactive");
	t_kill_switch_inactive_to_active->setTargetState(kill_switch_active_);
	t_kill_switch_active_to_inactive->setTargetState(kill_switch_inactive_);
	kill_switch_active_->addTransition(t_kill_switch_active_to_inactive);
	kill_switch_inactive_->addTransition(t_kill_switch_inactive_to_active);

	kill_switch_state_machine_.addState(kill_switch_inactive_);
	kill_switch_state_machine_.addState(kill_switch_active_);
	kill_switch_state_machine_.setInitialState(kill_switch_active_);
	kill_switch_state_machine_.start();

	connect(takeoff_land_button_, SIGNAL(clicked()), this, SLOT(sendTakeoffLandCommand()));
	connect(kill_switch_button_, SIGNAL(clicked()), this, SLOT(updateKillSwitch()));

	QGridLayout * main_layout = new QGridLayout;

	main_layout->addWidget(widget_);

	setLayout(main_layout);
}

CrazyflieInterfacePanel::~CrazyflieInterfacePanel()
{
}

void CrazyflieInterfacePanel::save(rviz_common::Config config) const
{
	Panel::save(config);
}

void CrazyflieInterfacePanel::load(const rviz_common::Config & config)
{
	Panel::load(config);
}

void CrazyflieInterfacePanel::onInitialize()
{
	auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
	node->declare_parameter("base_frame", rclcpp::ParameterValue(std::string("base_link")));
	node->get_parameter("base_frame", base_frame_);
	vehicle_odometry_sub_ =
			node->create_subscription<nav_msgs::msg::Odometry>(
			"/cf_1/odom",
			10,
			std::bind(&CrazyflieInterfacePanel::updateOdometry,this, std::placeholders::_1));
	navigator_status_sub_ =
			node->create_subscription<navigator_msgs::msg::NavigatorStatus>(
			"/cf_1/navigator/navigator_status",
			10,
			std::bind(&CrazyflieInterfacePanel::updateNavigatorStatus,this, std::placeholders::_1));
	navigator_goal_command_sub_ =
			node->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/goal_pose",
			10,
			std::bind(&CrazyflieInterfacePanel::commandNavigatorGoalPose, this, std::placeholders::_1));
	// vehicle_interface_command_pub_ =
	// 		node->create_publisher<px4_bridge_msgs::msg::VehicleInterfaceCommand>(
	// 				"/vehicle_interface/vehicle_interface_commands",
	// 				10);
	// vehicle_interface_status_sub_ =
	// 		node->create_subscription<px4_bridge_msgs::msg::VehicleInterfaceStatus>(
	// 				"/vehicle_interface/vehicle_interface_status",
	// 				10,
	// 				std::bind(&CrazyflieInterfacePanel::updateVehicleInterfaceStatus, this, std::placeholders::_1));
	nav_command_action_client_ = rclcpp_action::create_client<ActionNavigatorCommand>(node, "/cf_1/send_navigator_command");
	takeoff_client_ = node->create_client<crazyflie_interfaces::srv::Takeoff>("/all/takeoff");
	land_client_ = node->create_client<crazyflie_interfaces::srv::Land>("/all/land");
	go_to_client_ = node->create_client<crazyflie_interfaces::srv::GoTo>("/cf_1/go_to");
	notify_setpoints_stop_client_ = node->create_client<crazyflie_interfaces::srv::NotifySetpointsStop>("/cf_1/notify_setpoints_stop");
}

void CrazyflieInterfacePanel::cfTakeoff()
{
	auto request = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
	request->group_mask = 0;
	request->height = 1.0;
	request->duration.sec = 3;
	request->duration.nanosec = 0;
	auto future_result = takeoff_client_->async_send_request(request);
}

void CrazyflieInterfacePanel::cfLand()
{
	auto request = std::make_shared<crazyflie_interfaces::srv::Land::Request>();
	request->group_mask = 0;
	request->height = 0.0;
	request->duration.sec = 5;
	auto future_result = land_client_->async_send_request(request);
}

void CrazyflieInterfacePanel::cfLoiter()
{
	auto request = std::make_shared<crazyflie_interfaces::srv::GoTo::Request>();
	request->group_mask = 0;
	request->relative = false;
	request->goal.x = vehicle_odom_.pose.pose.position.x;
	request->goal.y = vehicle_odom_.pose.pose.position.y;
	request->goal.z = 1.0;
	request->yaw = 0.0;
	request->duration.sec = 3;
	auto future_result = go_to_client_->async_send_request(request);
}

void CrazyflieInterfacePanel::cfCommanderRelaxPriority()
{
	auto request = std::make_shared<crazyflie_interfaces::srv::NotifySetpointsStop::Request>();

	request->group_mask = 0;
	request->remain_valid_millisecs = 0;

	auto future_result = notify_setpoints_stop_client_->async_send_request(request);
}

void CrazyflieInterfacePanel::updateOdometry(const nav_msgs::msg::Odometry::UniquePtr msg)
{
	vehicle_odom_ = *msg;
	label_frame_id_odometry_->setText(QString(msg->header.frame_id.c_str()));
	label_position_odometry_x_->setText(QString::number(msg->pose.pose.position.x,'f',3));
	label_position_odometry_y_->setText(QString::number(msg->pose.pose.position.y,'f',3));
	label_position_odometry_z_->setText(QString::number(msg->pose.pose.position.z,'f',3));
	label_velocity_odometry_x_->setText(QString::number(msg->twist.twist.linear.x,'f',3));
	label_velocity_odometry_y_->setText(QString::number(msg->twist.twist.linear.y,'f',3));
	label_velocity_odometry_z_->setText(QString::number(msg->twist.twist.linear.z,'f',3));
}

void CrazyflieInterfacePanel::updateNavigatorStatus(const navigator_msgs::msg::NavigatorStatus::UniquePtr msg)
{
	if (msg->nav_mode == "Idle")
		navigator_mode_state_machine_.postEvent(new StringEvent("Idle"));
	else if (msg->nav_mode == "Land")
		navigator_mode_state_machine_.postEvent(new StringEvent("Land"));
	else if (msg->nav_mode == "Loiter")
		navigator_mode_state_machine_.postEvent(new StringEvent("Loiter"));
	else if (msg->nav_mode == "Takeoff")
		navigator_mode_state_machine_.postEvent(new StringEvent("Takeoff"));
	else
		navigator_mode_state_machine_.postEvent(new StringEvent("Navigate"));

}

// void CrazyflieInterfacePanel::updateVehicleInterfaceStatus(const px4_bridge_msgs::msg::VehicleInterfaceStatus::UniquePtr msg)
// {
// 	if (msg->vehicle_kill_switch_enabled) {
// 		kill_switch_state_machine_.postEvent(new StringEvent("kill_switch_active"));
// 	} else {
// 		kill_switch_state_machine_.postEvent(new StringEvent("kill_switch_inactive"));
// 	}
// }

void CrazyflieInterfacePanel::commandNavigatorGoalPose(geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
	ActionNavigatorCommandGoal nav_command_request;
	nav_command_request.command = "navigate_path_acmpc";
	nav_command_request.goal.header = msg->header;
	nav_command_request.goal.pose.position.x = msg->pose.position.x;
	nav_command_request.goal.pose.position.y = msg->pose.position.y;
	nav_command_request.goal.pose.position.z = 1.0;
	nav_command_action_client_->async_send_goal(nav_command_request);
}

void CrazyflieInterfacePanel::updateVehicleStatus()
{

}

void CrazyflieInterfacePanel::updateKillSwitch()
{
	if (kill_switch_state_machine_.configuration().contains(kill_switch_active_)) {
		kill_switch_state_machine_.postEvent(new StringEvent("kill_switch_inactive"));
	} else if (kill_switch_state_machine_.configuration().contains(kill_switch_inactive_)) {
		kill_switch_state_machine_.postEvent(new StringEvent("kill_switch_active"));
	}
}

void CrazyflieInterfacePanel::sendArmDisarmCommand()
{

}

void CrazyflieInterfacePanel::sendTakeoffLandCommand()
{
	if (navigator_mode_state_machine_.configuration().contains(idle_)) {
		ActionNavigatorCommandGoal nav_command_request;
		nav_command_request.command = "Takeoff";
		nav_command_action_client_->async_send_goal(nav_command_request);
	} else if (navigator_mode_state_machine_.configuration().contains(takeoff_)) {
		ActionNavigatorCommandGoal nav_command_request;
		nav_command_request.command = "Land";
		nav_command_action_client_->async_send_goal(nav_command_request);
	} else if (navigator_mode_state_machine_.configuration().contains(loiter_)) {
		ActionNavigatorCommandGoal nav_command_request;
		nav_command_request.command = "Land";
		nav_command_action_client_->async_send_goal(nav_command_request);
	} else if (navigator_mode_state_machine_.configuration().contains(navigate_)) {
		ActionNavigatorCommandGoal nav_command_request;
		nav_command_request.command = "Land";
		nav_command_action_client_->async_send_goal(nav_command_request);
	}
}

}

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(crazyflie_rviz2_gui::CrazyflieInterfacePanel, rviz_common::Panel)