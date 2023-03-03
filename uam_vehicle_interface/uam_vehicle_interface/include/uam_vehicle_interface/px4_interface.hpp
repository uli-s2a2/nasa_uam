#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <uam_control_msgs/msg/attitude_setpoint.hpp>
#include "nav_msgs/msg/odometry.hpp"

namespace uam_vehicle_interface
{

static constexpr uint8_t OFFBOARD_ENABLE_CHANNEL = 7;
static constexpr uint8_t POSITION_SETPOINT_CHANNEL = 5;

class Px4Interface : public rclcpp::Node
{
public:
	Px4Interface();
private:
	// ----------------------- Publishers --------------------------
	rclcpp::Publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr vehicle_local_position_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_odometry_pub_;

	// ----------------------- Subscribers --------------------------
	rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr channels_sub_;
//	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
//	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
//	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
	rclcpp::Subscription<uam_control_msgs::msg::AttitudeSetpoint>::SharedPtr  attitude_setpoint_sub_;

	// Class Variables
	bool offboard_control_state_;
	uint8_t offboard_counter_{0};
	float mass_{0.7f};
	float max_thrust_{50.5f};
	float min_thrust_{0.5492f};

	// ROS2 Variables
	px4_msgs::msg::BatteryStatus battery_status_;
	px4_msgs::msg::VehicleStatus vehicle_status_;
	px4_msgs::msg::RcChannels channels_;
//	px4_msgs::msg::VehicleLocalPosition local_position_;
//	px4_msgs::msg::VehicleAttitude attitude_;
	px4_msgs::msg::VehicleOdometry vehicle_odometry_;

	// Private class methods
	void publish_attitude_setpoint(const uam_control_msgs::msg::AttitudeSetpoint::SharedPtr msg);
	void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
	void publish_offboard_control_mode();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,float param2 = 0.0);
	float compute_relative_thrust(const float &collective_thrust) const;
	void enable_offboard_control();
	void arm();
	void disarm();
};
}