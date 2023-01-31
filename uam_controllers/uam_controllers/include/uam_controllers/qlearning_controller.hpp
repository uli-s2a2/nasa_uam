#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <uam_navigator_msgs/msg/path.hpp>
#include <Eigen/Dense>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <mav_base/mav_base.hpp>
#include <mav_base/mav_util.h>

namespace uam_controllers
{

#define QLEARNING_CALLBACK_RATE_MS 10
#define QLEARNING_STATE_VECTOR_SIZE 6
#define QLEARNING_CONTROL_VECTOR_SIZE 3
#define AUGMENTED_STATE_VECTOR_SIZE (QLEARNING_STATE_VECTOR_SIZE + QLEARNING_CONTROL_VECTOR_SIZE)
#define CRITIC_WEIGHT_VECTOR_SIZE (AUGMENTED_STATE_VECTOR_SIZE)*(AUGMENTED_STATE_VECTOR_SIZE + 1) / 2


class mavState: public Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, 1> {
public:
	// Constructors
	mavState()
			: Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, 1>()
	{};

	typedef Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, 1> Base;

	mavState(const Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, 1> &other)
			: Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, 1>(other)
	{
	};

	//Operators
	mavState &operator=(const Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, 1> &other)
	{
		this->Base::operator=(other);
		return *this;
	}

	template<typename OtherDerived>
	Eigen::Product<mavState, OtherDerived> operator*(const MatrixBase<OtherDerived> &other) const
	{
		this->Base::operator*(other);
		return *this;
	}

	EIGEN_DEVICE_FUNC
	EIGEN_STRONG_INLINE Scalar&
	vx() { return (*this)[3]; }


	EIGEN_DEVICE_FUNC
	EIGEN_STRONG_INLINE Scalar&
	vy() { return (*this)[4]; }


	EIGEN_DEVICE_FUNC
	EIGEN_STRONG_INLINE Scalar&
	vz() { return (*this)[5]; }

//	EIGEN_DEVICE_FUNC
//	EIGEN_STRONG_INLINE Scalar&
//	ax() { return (*this)[6]; }
//
//
//	EIGEN_DEVICE_FUNC
//	EIGEN_STRONG_INLINE Scalar&
//	ay() { return (*this)[7]; }
//
//
//	EIGEN_DEVICE_FUNC
//	EIGEN_STRONG_INLINE Scalar&
//	az() { return (*this)[8]; }
};

typedef Eigen::Vector<double, QLEARNING_CONTROL_VECTOR_SIZE> control_vector_t;
typedef Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, QLEARNING_STATE_VECTOR_SIZE> state_matrix_t;
typedef Eigen::Matrix<double, QLEARNING_CONTROL_VECTOR_SIZE, QLEARNING_CONTROL_VECTOR_SIZE> control_matrix_t;

typedef Eigen::Vector<double, AUGMENTED_STATE_VECTOR_SIZE> augmented_state_vector_t;
typedef Eigen::Matrix<double, AUGMENTED_STATE_VECTOR_SIZE, AUGMENTED_STATE_VECTOR_SIZE> augmented_state_matrix_t;
typedef Eigen::Vector<double, CRITIC_WEIGHT_VECTOR_SIZE> critic_weight_vector_t;
typedef Eigen::Vector<double, CRITIC_WEIGHT_VECTOR_SIZE> critic_augmented_state_kronecker_t;
typedef Eigen::Matrix<double, CRITIC_WEIGHT_VECTOR_SIZE, CRITIC_WEIGHT_VECTOR_SIZE> critic_radial_basis_matrix_t;
typedef Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, QLEARNING_CONTROL_VECTOR_SIZE> actor_weight_matrix_t;
typedef Eigen::Matrix<double, QLEARNING_STATE_VECTOR_SIZE, QLEARNING_STATE_VECTOR_SIZE> actor_radial_basis_matrix_t;
typedef Eigen::Matrix<double, CRITIC_WEIGHT_VECTOR_SIZE, AUGMENTED_STATE_VECTOR_SIZE * AUGMENTED_STATE_VECTOR_SIZE> svec_matrix_t;

class QLearningController {
public:
	QLearningController();
	rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_interface() {return mav_base_->get_node_base_interface();};
private:
	// ----------------------- Publishers --------------------------



	// ----------------------- Subscribers --------------------------
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr navigator_setpoint_sub_;

	// Message Callback Variables
//	std::atomic<uint64_t> timestamp_;
//	px4_msgs::msg::BatteryStatus mav_battery_status_;
//	px4_msgs::msg::VehicleStatus mav_vehicle_status_;
//	px4_msgs::msg::RcChannels mav_channels_;
//	px4_msgs::msg::VehicleOdometry mav_odom_;
	mavState navigator_setpoint_;

	// Class Variables
	std::shared_ptr<mav_base::MavBase> mav_base_;
	px4_msgs::msg::VehicleLocalPosition mav_local_position_;
	px4_msgs::msg::VehicleAttitude mav_attitude_;
	px4_msgs::msg::RcChannels mav_rc_channels_;
	uint8_t mav_id_;
	rclcpp::TimerBase::SharedPtr timer_;
	double begin_time_{0};
	double current_time_{0};
	mavState state_vector_; // x,y,z,vx,vy,vz
	control_vector_t control_vector_;
	augmented_state_vector_t augmented_state_vector_;
	state_matrix_t state_penalty_matrix_;
	control_matrix_t control_penalty_matrix_;
	state_matrix_t terminal_riccati_matrix_;
	critic_weight_vector_t critic_weight_vector_;
	critic_augmented_state_kronecker_t critic_augmented_state_kronecker_;
	critic_augmented_state_kronecker_t critic_augmented_state_kronecker_prior_;
	critic_radial_basis_matrix_t critic_radial_basis_matrix_;
	actor_weight_matrix_t actor_weight_matrix_;
	actor_radial_basis_matrix_t actor_radial_basis_matrix_;
	double critic_convergence_rate{0.1};
	double actor_convergence_rate{0.01};
	double time_resolution{ QLEARNING_CALLBACK_RATE_MS / 1000.0 };
	double Q_function_;
	svec_matrix_t vec_to_svec_transform_matrix_;

	// Class methods
	void setup();
	void critic();
	void actor();
	void estimate_Q();
	void compute_control();

	critic_radial_basis_matrix_t compute_critic_radial_basis_matrix(double t);
	actor_radial_basis_matrix_t compute_actor_radial_basis_matrix(double t);
	void compute_svec_matrix();
	void update();
	void publish_control_mellinger();
//	void publish_control_fbl();
//	void arm() const;
//	void disarm() const;
//	void publish_control();
//	void publish_offboard_control_mode() const;
//	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
//	                             float param2 = 0.0) const;
};// Class QLearningController
}