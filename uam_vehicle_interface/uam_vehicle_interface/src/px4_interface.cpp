#include "uam_vehicle_interface/px4_interface.hpp"
#include "uam_util/qos_profiles.hpp"

namespace uam_vehicle_interface
{

Px4Interface::Px4Interface()
		: rclcpp::Node("uam_vehicle_interface")
{
	// ----------------------- Publishers --------------------------
	vehicle_command_pub_ =
			this->create_publisher<px4_msgs::msg::VehicleCommand>(
					"/fmu/in/vehicle_command", uam_util::px4_qos_pub);
	offboard_control_mode_pub_ =
			this->create_publisher<px4_msgs::msg::OffboardControlMode>(
					"/fmu/in/offboard_control_mode", uam_util::px4_qos_pub);
	vehicle_rates_setpoint_pub_ =
			this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
					"/fmu/in/vehicle_rates_setpoint", uam_util::px4_qos_pub);
	vehicle_attitude_setpoint_pub_ =
			this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
					"/fmu/in/vehicle_attitude_setpoint", uam_util::px4_qos_pub);
	vehicle_local_position_setpoint_pub_ =
			this->create_publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>(
					"/fmu/in/vehicle_local_position_setpoint", uam_util::px4_qos_pub);
	vehicle_odometry_pub_ =
			this->create_publisher<nav_msgs::msg::Odometry>(
					"/uam_vehicle_interface/odometry", uam_util::px4_qos_pub);
	tf_dynamic_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
	// ----------------------- Subscribers --------------------------
	battery_status_sub_ =
			this->create_subscription<px4_msgs::msg::BatteryStatus>(
					"/fmu/out/battery_status", uam_util::px4_qos_sub,
					[this](const px4_msgs::msg::BatteryStatus::UniquePtr msg)
					{
						battery_status_ = *msg;
					});
	attitude_setpoint_sub_ =
			this->create_subscription<uam_control_msgs::msg::AttitudeSetpoint>(
					"/uam_control/attitude_setpoint",
					uam_util::px4_qos_sub,
					std::bind(&Px4Interface::publish_attitude_setpoint, this, std::placeholders::_1));
	vehicle_status_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleStatus>(
					"/fmu/out/vehicle_status",
					uam_util::px4_qos_sub,
					[this](const px4_msgs::msg::VehicleStatus::UniquePtr msg)
					{
						vehicle_status_ = *msg;
					});
//	local_position_sub_ =
//		this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos_sub,
//			[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
//				local_position_ = *msg;
//			});
//	attitude_sub_ =
//			this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos_sub,
//           [this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
//               attitude_ = *msg;
//           });
	vehicle_odometry_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleOdometry>(
					"/fmu/out/vehicle_odometry",
					uam_util::px4_qos_sub,
					std::bind(&Px4Interface::vehicle_odometry_callback, this, std::placeholders::_1));
	channels_sub_ =
			this->create_subscription<px4_msgs::msg::RcChannels>(
					"/fmu/out/rc_channels",
					uam_util::px4_qos_sub,
					[this](const px4_msgs::msg::RcChannels::UniquePtr msg)
					{
//						channels_ = *msg;
//						if (msg->channels[msg->function[px4_msgs::msg::RcChannels::FUNCTION_OFFBOARD]] >= 0.75) {
//							if (!offboard_control_enable_) {
//								enable_offboard_control();
//							}
//						} else {
//							offboard_control_enable_ = false;
//						}
					});
	vehicle_interface_commands_sub_ =
			this->create_subscription<uam_vehicle_interface_msgs::msg::VehicleInterfaceCommands>(
					"/uam_vehicle_interface/commands",
					10,
					[this](const uam_vehicle_interface_msgs::msg::VehicleInterfaceCommands::UniquePtr msg)
					{
						switch (msg->vehicle_cmd) {
							case uam_vehicle_interface_msgs::msg::VehicleInterfaceCommands::VEHICLE_INTERFACE_CMD_ENABLE:
								offboard_control_enable_ = true;
								break;
							case uam_vehicle_interface_msgs::msg::VehicleInterfaceCommands::VEHICLE_INTERFACE_CMD_DISABLE:
								offboard_control_enable_ = false;
								break;
						}
					});

	setup_static_transforms();
}

/**
* @brief Publish the offboard control mode.
*        For this example, only position and altitude controls are active.
*/
void Px4Interface::publish_offboard_control_mode()
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = int(get_clock()->now().nanoseconds() / 1000);
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = true;
	msg.body_rate = false;

	offboard_control_mode_pub_->publish(msg);
}


/**
 * @brief Send a command to Arm the vehicle
 */
void Px4Interface::arm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void Px4Interface::disarm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void Px4Interface::publish_vehicle_command(uint16_t command, float param1,
                                           float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = int(get_clock()->now().nanoseconds() / 1000);
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_pub_->publish(msg);
}

//void Px4Interface::publish_rate_control(mavRateControl control) {
//	px4_msgs::msg::VehicleRatesSetpoint vehicle_rates;
//	if (channels_.channels[OFFBOARD_ENABLE_CHANNEL - 1] >= 0.75
//			&& vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
//		if (vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD && offboard_counter_ == 10) {
//			publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
//		} else if (offboard_counter_ < 11) {
//			offboard_counter_++;
//		}
//
//		vehicle_rates.timestamp = int(get_clock()->now().nanoseconds() / 1000);
//		vehicle_rates.roll = control.roll_rate;
//		vehicle_rates.pitch = control.pitch_rate;
//		vehicle_rates.yaw = control.yaw_rate;
//		vehicle_rates.thrust_body[2] = fmin(-compute_relative_thrust(control.thrust_normalized * mass_),0.0);
//
//		publish_offboard_control_mode();
//		vehicle_rates_setpoint_pub_->publish(vehicle_rates);
//	} else {
//		offboard_counter_ = 0;
//	}
//}

void Px4Interface::publish_attitude_setpoint(const uam_control_msgs::msg::AttitudeSetpoint::SharedPtr msg)
{
	px4_msgs::msg::VehicleAttitudeSetpoint vehicle_attitude;

	if (offboard_control_enable_ && vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
		if (vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD && offboard_counter_ == 10) {
			publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		} else if (offboard_counter_ < 11) {
			offboard_counter_++;
		}
		vehicle_attitude.timestamp = int(get_clock()->now().nanoseconds() / 1000);
		vehicle_attitude.roll_body = msg->roll_body;
		vehicle_attitude.pitch_body = msg->pitch_body;
		vehicle_attitude.yaw_body = msg->yaw_body;
		vehicle_attitude.q_d = msg->q_d;
		vehicle_attitude.thrust_body[2] = fminf(-compute_relative_thrust(msg->thrust_body_normalized * mass_), 0.0);

		publish_offboard_control_mode();
		vehicle_attitude_setpoint_pub_->publish(vehicle_attitude);
	} else {
		offboard_counter_ = 0;
	}
}

void Px4Interface::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
	nav_msgs::msg::Odometry vehicle_odom;
	vehicle_odom.header.stamp = get_clock()->now();
	vehicle_odom.header.frame_id = "map_ned";
	vehicle_odom.child_frame_id = "base_link_frd";
	vehicle_odom.pose.pose.position.x = msg->position[0];
	vehicle_odom.pose.pose.position.y = msg->position[1];
	vehicle_odom.pose.pose.position.z = msg->position[2];
	vehicle_odom.pose.pose.orientation.w = msg->q[0];
	vehicle_odom.pose.pose.orientation.x = msg->q[1];
	vehicle_odom.pose.pose.orientation.y = msg->q[2];
	vehicle_odom.pose.pose.orientation.z = msg->q[3];
	vehicle_odom.twist.twist.linear.x = msg->velocity[0];
	vehicle_odom.twist.twist.linear.y = msg->velocity[1];
	vehicle_odom.twist.twist.linear.z = msg->velocity[2];
	vehicle_odom.twist.twist.angular.x = msg->angular_velocity[0];
	vehicle_odom.twist.twist.angular.y = msg->angular_velocity[1];
	vehicle_odom.twist.twist.angular.z = msg->angular_velocity[2];
	vehicle_odometry_pub_->publish(vehicle_odom);

	Eigen::Vector3d pose_ned(msg->position[0],msg->position[1],msg->position[2]);
	Eigen::Translation3d pose_enu(px4_ros_com::frame_transforms::ned_to_enu_local_frame(pose_ned));
	Eigen::Quaterniond q(msg->q[0],msg->q[1],msg->q[2],msg->q[3]);
	Eigen::Quaterniond q_base_link = px4_ros_com::frame_transforms::aircraft_to_baselink_orientation(
			px4_ros_com::frame_transforms::ned_to_enu_orientation(q));
	Eigen::Affine3d base_link = pose_enu * q_base_link;
	geometry_msgs::msg::TransformStamped tf = tf2::eigenToTransform(base_link);
	tf.header.stamp = this->get_clock()->now();
	tf.header.frame_id = "map";
	tf.child_frame_id = "base_link";

	tf_dynamic_broadcaster_->sendTransform(tf);
}

void Px4Interface::setup_static_transforms()
{
	Eigen::Quaterniond q_enu_to_ned = px4_ros_com::frame_transforms::enu_to_ned_orientation(Eigen::Quaterniond(1.0,0.0,0.0,0.0));
	Eigen::Affine3d enu_to_ned_transform(q_enu_to_ned);
	geometry_msgs::msg::TransformStamped tf_map_ned = tf2::eigenToTransform(enu_to_ned_transform);
	tf_map_ned.header.stamp = this->get_clock()->now();
	tf_map_ned.header.frame_id = "map";
	tf_map_ned.child_frame_id = "map_ned";
	tf_static_broadcaster_->sendTransform(tf_map_ned);


	Eigen::Quaterniond q_base_link_to_aircraft = px4_ros_com::frame_transforms::baselink_to_aircraft_orientation(Eigen::Quaterniond(1.0,0.0,0.0,0.0));
	Eigen::Affine3d base_link_aircraft_transform(q_base_link_to_aircraft);
	geometry_msgs::msg::TransformStamped tf_base_link_frd = tf2::eigenToTransform(base_link_aircraft_transform);
	tf_base_link_frd.header.stamp = this->get_clock()->now();
	tf_base_link_frd.header.frame_id = "base_link";
	tf_base_link_frd.child_frame_id = "base_link_frd";
	tf_static_broadcaster_->sendTransform(tf_base_link_frd);
}

//void Px4Interface::publish_local_position_setpoint(mavLocalPositionSetpoint control)
//{
//	px4_msgs::msg::VehicleLocalPositionSetpoint vehicle_local_position_setpoint;
//	if (channels_.channels[OFFBOARD_ENABLE_CHANNEL - 1] >= 0.75
//			&& vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
//		if (vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD && offboard_counter_ == 10) {
//			publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
//		} else if (offboard_counter_ < 11) {
//			offboard_counter_++;
//		}
//
//		vehicle_local_position_setpoint.timestamp = int(get_clock()->now().nanoseconds() / 1000);
//		vehicle_local_position_setpoint.x = control.x;
//		vehicle_local_position_setpoint.y = control.y;
//		vehicle_local_position_setpoint.z = control.z;
//		vehicle_local_position_setpoint.vx = control.vx;
//		vehicle_local_position_setpoint.vy = control.vy;
//		vehicle_local_position_setpoint.vz = control.vz;
//		vehicle_local_position_setpoint.acceleration[0] = control.ax;
//		vehicle_local_position_setpoint.acceleration[1] = control.ay;
//		vehicle_local_position_setpoint.acceleration[2] = control.az;
//		vehicle_local_position_setpoint.thrust[2] = fmin(-compute_relative_thrust(control.thrust_normalized * mass_),0.0);
//
//		publish_offboard_control_mode();
//		vehicle_local_position_setpoint_pub_->publish(vehicle_local_position_setpoint);
//	} else {
//		offboard_counter_ = 0;
//	}
//}

void Px4Interface::enable_offboard_control()
{
	RCLCPP_INFO(get_logger(), "Enabling offboard control");
	uam_control_msgs::msg::AttitudeSetpoint::SharedPtr msg = std::make_shared<uam_control_msgs::msg::AttitudeSetpoint>();
	msg->roll_body = 0.f;
	msg->pitch_body = 0.f;
	msg->yaw_body = 0.f;
	msg->q_d = {1.f, 0.f, 0.f, 0.f};
	msg->thrust_body_normalized = 9.8f;
	uint8_t attempts = 0;
	offboard_control_enable_ = true;
	while (vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
			vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
		publish_attitude_setpoint(msg);
		if (attempts < 20) {
			attempts++;
		} else {
			break;
		}
	}
}

float Px4Interface::compute_relative_thrust(const float &collective_thrust) const
{
#ifdef RUN_SITL
	float motor_speed = sqrtf(collective_thrust / (4 * motor_constant_));
	float thrust_command = (motor_speed - motor_velocity_armed_) / motor_input_scaling_;
//	float rel_thrust = (collective_thrust - min_thrust_) / (max_thrust_ - min_thrust_);
//	return (0.54358075f * rel_thrust + 0.f * sqrtf(3.6484f * rel_thrust + 0.00772641f) - 0.021992793f);
	return thrust_command;
#else
	if (battery_status_.voltage_filtered_v > 14.0) {
		float rel_thrust = (collective_thrust - min_thrust_) / (max_thrust_ - min_thrust_);
		return (0.54358075 * rel_thrust + 0.25020242 * sqrt(3.6484 * rel_thrust + 0.00772641) - 0.021992793) *
				(1 - 0.0779 * (battery_status_.voltage_filtered_v - 16.0));
	} else {
		float rel_thrust = (collective_thrust - min_thrust_) / (max_thrust_ - min_thrust_);
		return (0.54358075 * rel_thrust + 0.25020242 * sqrt(3.6484 * rel_thrust + 0.00772641) - 0.021992793);
	}
#endif
}
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<uam_vehicle_interface::Px4Interface>());
	rclcpp::shutdown();
	return 0;
}


