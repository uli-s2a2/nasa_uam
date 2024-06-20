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

	QObject::connect(takeoff_, &QState::entered, this, [this] {CrazyflieInterfacePanel::cfTakeoff(0);});
	QObject::connect(land_, &QState::entered, this, [this] {CrazyflieInterfacePanel::cfLand(0);});
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
	scenario_push_button_ = widget_->findChild<QPushButton*>("run_scenario_button");
	scenario_selector_box_ = widget_->findChild<QComboBox*>("scenario_selector_box");

	connect(takeoff_land_button_, SIGNAL(clicked()), this, SLOT(sendTakeoffLandCommand()));
	connect(scenario_push_button_, SIGNAL(clicked()), this, SLOT(runScenarioThread()));
	connect(scenario_selector_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(updateScenarioVisuals(int)));

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
			std::bind(&CrazyflieInterfacePanel::goalPoseCallback, this, std::placeholders::_1));

	vports_marker_pub_ = 
			node->create_publisher<visualization_msgs::msg::MarkerArray>(
				"/visualization/vports",
				10);
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

	cf_go_to_clients_.push_back(node->create_client<crazyflie_interfaces::srv::GoTo>("/cf_1/go_to"));
	cf_go_to_clients_.push_back(node->create_client<crazyflie_interfaces::srv::GoTo>("/cf_2/go_to"));
	cf_go_to_clients_.push_back(node->create_client<crazyflie_interfaces::srv::GoTo>("/cf_3/go_to"));
	cf_go_to_clients_.push_back(node->create_client<crazyflie_interfaces::srv::GoTo>("/cf_4/go_to"));

	all_land_client_ = node->create_client<crazyflie_interfaces::srv::Land>("/all/land");
	cf_land_clients_.push_back(node->create_client<crazyflie_interfaces::srv::Land>("/cf_1/land"));
	cf_land_clients_.push_back(node->create_client<crazyflie_interfaces::srv::Land>("/cf_2/land"));
	cf_land_clients_.push_back(node->create_client<crazyflie_interfaces::srv::Land>("/cf_3/land"));
	cf_land_clients_.push_back(node->create_client<crazyflie_interfaces::srv::Land>("/cf_4/land"));

	all_takeoff_client_ = node->create_client<crazyflie_interfaces::srv::Takeoff>("/all/takeoff");
	cf_takeoff_clients_.push_back(node->create_client<crazyflie_interfaces::srv::Takeoff>("/cf_1/takeoff"));
	cf_takeoff_clients_.push_back(node->create_client<crazyflie_interfaces::srv::Takeoff>("/cf_2/takeoff"));
	cf_takeoff_clients_.push_back(node->create_client<crazyflie_interfaces::srv::Takeoff>("/cf_3/takeoff"));
	cf_takeoff_clients_.push_back(node->create_client<crazyflie_interfaces::srv::Takeoff>("/cf_4/takeoff"));

	notify_setpoints_stop_client_ = node->create_client<crazyflie_interfaces::srv::NotifySetpointsStop>("/cf_1/notify_setpoints_stop");
}

void CrazyflieInterfacePanel::cfAllTakeoff()
{
	auto request = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
	request->group_mask = 0;
	request->height = 1.0;
	request->duration.sec = 3;
	request->duration.nanosec = 0;
	auto future_result = all_takeoff_client_->async_send_request(request);
}

void CrazyflieInterfacePanel::cfTakeoff(int indx)
{
	auto request = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
	request->group_mask = 0;
	request->height = 1.0;
	request->duration.sec = 3;
	request->duration.nanosec = 0;
	auto future_result = cf_takeoff_clients_[indx]->async_send_request(request);
}

void CrazyflieInterfacePanel::cfAllLand()
{
	auto request = std::make_shared<crazyflie_interfaces::srv::Land::Request>();
	request->group_mask = 0;
	request->height = 0.0;
	request->duration.sec = 5;
	auto future_result = all_land_client_->async_send_request(request);
}

void CrazyflieInterfacePanel::cfLand(int index)
{
	auto request = std::make_shared<crazyflie_interfaces::srv::Land::Request>();
	request->group_mask = 0;
	request->height = 0.0;
	request->duration.sec = 5;
	auto future_result = cf_land_clients_[index]->async_send_request(request);
}

void CrazyflieInterfacePanel::cfGoTo(int index, double x, double y, double z, double duration, bool relative)
{
	auto request = std::make_shared<crazyflie_interfaces::srv::GoTo::Request>();
	request->group_mask = 0;
	request->relative = relative;
	request->goal.x = x;
	request->goal.y = y;
	request->goal.z = z;
	request->yaw = 0.0;
	request->duration.sec = duration;
	auto future_result = cf_go_to_clients_[index]->async_send_request(request);
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
	auto future_result = cf_go_to_clients_[0]->async_send_request(request);
}

void CrazyflieInterfacePanel::cfCommanderRelaxPriority()
{
	auto request = std::make_shared<crazyflie_interfaces::srv::NotifySetpointsStop::Request>();

	request->group_mask = 0;
	request->remain_valid_millisecs = 0;

	auto future_result = notify_setpoints_stop_client_->async_send_request(request);
}

void CrazyflieInterfacePanel::updateScenarioVisuals(int index)
{

	auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

	visualization_msgs::msg::MarkerArray vertiports;
	switch (index) {
		case 1:
			std::vector<geometry_msgs::msg::Pose> vport_poses;
			geometry_msgs::msg::Pose pose;
			pose.position.x = 0.65;
			pose.position.y = 0.5;
			pose.position.z = 0.0;
			vport_poses.push_back(pose);

			pose.position.x = 0.75;
			pose.position.y = 2.0;
			pose.position.z = 0.0;
			vport_poses.push_back(pose);

			pose.position.x = 3.2;
			pose.position.y = 3.2;
			pose.position.z = 0.0;
			vport_poses.push_back(pose);

			pose.position.x = 2.0;
			pose.position.y = 2.4;
			pose.position.z = 0.0;
			vport_poses.push_back(pose);

			pose.position.x = 6.8;
			pose.position.y = 1.0;
			pose.position.z = 0.0;
			vport_poses.push_back(pose);

			// Create a marker for each vertiport location
			for (size_t i = 0; i < vport_poses.size(); i++) {		
				visualization_msgs::msg::Marker vport;
				vport.header.frame_id = "world";
				vport.header.stamp = node->now();
				vport.id = i + 30;
				vport.pose = vport_poses[i];
				vport.scale.x = 10.0;
				vport.scale.y = 10.0;
				vport.scale.z = 10.0;
				vport.action = visualization_msgs::msg::Marker::ADD;
				vport.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
				vport.mesh_resource = "package://crazyflie_rviz2_gui/resource/vport3.dae";
				vport.mesh_use_embedded_materials = true;
				vertiports.markers.push_back(vport);
			}

			vports_marker_pub_->publish(vertiports);
			break;
	}
}

void ScenarioWorker::runScenario()
{
	switch (scenario_)
	{
		case 1:
			cfTakeoff(2);
			QThread::sleep(3);
			cfGoTo(2, 5.0, 1.5, 1.0, 12, false);
			QThread::sleep(5);
			cfTakeoff(3);
			QThread::sleep(3);
			cfGoTo(3, 5.0, 1.5, 1.0, 14, false);
			QThread::sleep(8);
			cfGoTo(2, 6.9, 1.1, 1.0, 6, false);
			cfTakeoff(1);
			QThread::sleep(3);
			cfGoTo(1, 2.7, 2.7, 1.0, 8, false);
			QThread::sleep(4);
			cfLand(2);
			cfGoTo(3, 6.9, 0.9, 1.0, 6, false);
			QThread::sleep(7);
			cfLand(3);
			cfGoTo(1, 5.0, 1.5, 1.0, 8, false);
			QThread::sleep(8);
			cfGoTo(1, 6.7, 1.1, 1.0, 6, false);
			QThread::sleep(7);
			cfLand(1);
			navigatorTakeoff();
			QThread::sleep(10);
			acmpcGoToGoal(6.7, 0.9);
			emit finished();
			break;
	}
}

void CrazyflieInterfacePanel::goalPoseCallback(geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
	commandNavigatorGoalPose(msg->pose.position.x, msg->pose.position.y);
}

void CrazyflieInterfacePanel::commandNavigatorGoalPose(double x, double y)
{
	ActionNavigatorCommandGoal nav_command_request;
	nav_command_request.command = "navigate_path_acmpc";
	nav_command_request.goal.header.frame_id = "world";
	// nav_command_request.goal.header.stamp = 
	nav_command_request.goal.pose.position.x = x;
	nav_command_request.goal.pose.position.y = y;
	nav_command_request.goal.pose.position.z = 1.0;
	nav_command_action_client_->async_send_goal(nav_command_request);
}

void CrazyflieInterfacePanel::runScenarioThread()
{
	if (!navigator_mode_state_machine_.configuration().contains(idle_))
			return;

	auto selected_scenario_idx = scenario_selector_box_->currentIndex();

	ScenarioWorker *worker = new ScenarioWorker(selected_scenario_idx);
	QThread *thread = new QThread();

	worker->moveToThread(thread);

	connect(worker, SIGNAL(cfGoTo(int, double, double, double, double, bool)), this, SLOT(cfGoTo(int, double, double, double, double, bool)));
	connect(worker, SIGNAL(acmpcGoToGoal(double, double)), this, SLOT(commandNavigatorGoalPose(double, double)));
	connect(worker, SIGNAL(navigatorTakeoff()), this, SLOT(navigatorTakeoff()));
	connect(worker, SIGNAL(cfTakeoff(int)), this, SLOT(cfTakeoff(int)));
	connect(worker, SIGNAL(cfLand(int)), this, SLOT(cfLand(int)));
	connect(thread, SIGNAL(started()), worker, SLOT(runScenario()));
	connect(worker, SIGNAL(finished()), thread, SLOT(quit()));
	connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
	connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));

	thread->start();	
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

void CrazyflieInterfacePanel::updateVehicleStatus()
{

}

void CrazyflieInterfacePanel::navigatorTakeoff()
{
	ActionNavigatorCommandGoal nav_command_request;
	nav_command_request.command = "Takeoff";
	nav_command_action_client_->async_send_goal(nav_command_request);
}

void CrazyflieInterfacePanel::navigatorLand()
{
	ActionNavigatorCommandGoal nav_command_request;
	nav_command_request.command = "Land";
	nav_command_action_client_->async_send_goal(nav_command_request);
}

void CrazyflieInterfacePanel::sendTakeoffLandCommand()
{
	if (navigator_mode_state_machine_.configuration().contains(idle_)) {
		navigatorTakeoff();
	} else if (navigator_mode_state_machine_.configuration().contains(takeoff_)) {
		navigatorLand();
	} else if (navigator_mode_state_machine_.configuration().contains(loiter_)) {
		navigatorLand();
	} else if (navigator_mode_state_machine_.configuration().contains(navigate_)) {
		navigatorLand();
	}
}

}  // namespace crazyflie_rviz2_gui
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(crazyflie_rviz2_gui::CrazyflieInterfacePanel, rviz_common::Panel)