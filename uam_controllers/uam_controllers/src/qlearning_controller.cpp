#include <uam_controllers/qlearning_controller.hpp>

using namespace uam_controllers;
using namespace mav_base;
using namespace Eigen;

QLearningController::QLearningController() {
	compute_svec_matrix();
	state_vector_.setZero();
	control_vector_.setZero();
	control_vector_(2) = -2.0;
	navigator_setpoint_.setZero();
	critic_augmented_state_kronecker_prior_.setZero();
	// set initial weight
	critic_weight_vector_ <<   48.28,-0.00,-0.00,54.14,-0.00,0.00,14.14,-0.00,0.00,48.28,-0.00,-0.00,54.14,-0.00,-0.00,14.14,-0.00,48.28,-0.00,-0.00,54.14,-0.00,-0.00,14.14,54.14,-0.00,-0.00,20.00,-0.00,-0.00,54.14,-0.00,-0.00,20.00,-0.00,54.14,-0.00,-0.00,20.00,5.00,0.00,0.00,5.00,0.00,5.00;
	actor_weight_matrix_ <<    -2.00,0.00,-0.00,0.00,-2.00,0.00,0.00,0.00,-2.00,-2.83,0.00,0.00,0.00,-2.83,0.00,0.00,0.00,-2.83;
	terminal_riccati_matrix_ << 28.28,-0.00,-0.00,10.00,-0.00,0.00,-0.00,28.28,-0.00,-0.00,10.00,-0.00,-0.00,-0.00,28.28,-0.00,-0.00,10.00,10.00,-0.00,-0.00,14.14,-0.00,-0.00,-0.00,10.00,-0.00,-0.00,14.14,-0.00,0.00,-0.00,10.00,-0.00,-0.00,14.14;
	state_penalty_matrix_.setZero();
	state_penalty_matrix_.diagonal() << 20.0, 20.0, 20.0, 10.0, 10.0, 10.0;
	control_penalty_matrix_.setZero();
	control_penalty_matrix_.diagonal() << 5.0, 5.0, 5.0;
	mav_base_ = std::make_shared<MavBase>("qlearning_controller");
	begin_time_ = mav_base_->get_clock()->now().seconds();
	// ----------------------- Subscribers --------------------------
	auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
	navigator_setpoint_sub_ =
			mav_base_->create_subscription<nav_msgs::msg::Odometry>(
					"/navigator/setpoints", qos_sub,
					[this](const nav_msgs::msg::Odometry::UniquePtr msg) {
						navigator_setpoint_.x() = msg->pose.pose.position.x;
						navigator_setpoint_.y() = msg->pose.pose.position.y;
						navigator_setpoint_.z() = msg->pose.pose.position.z;
						navigator_setpoint_.vx() = msg->twist.twist.linear.x;
						navigator_setpoint_.vy() = msg->twist.twist.linear.y;
						navigator_setpoint_.vz() = msg->twist.twist.linear.z;
					});

	// ----------------------- Publishers ---------------------------
	auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();


	auto timer_callback = [this]() -> void
	{
		if (this->mav_base_->get_vehicle_status().arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED
				&& this->mav_base_->get_vehicle_status().nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
			//------- Q-Learning -------------
			setup();
			critic();
			estimate_Q();
			actor();
			compute_control();
			critic_augmented_state_kronecker_prior_ = critic_augmented_state_kronecker_;

			// ------ Hardware Control ----------
			publish_control_mellinger();
		} else {
			begin_time_ = mav_base_->get_clock()->now().seconds();
		}
	};

	timer_ = this->mav_base_->create_wall_timer(std::chrono::milliseconds(QLEARNING_CALLBACK_RATE_MS), timer_callback);
}

void QLearningController::setup()
{
	current_time_ = (mav_base_->get_clock()->now().seconds() - begin_time_);
	mav_local_position_ = mav_base_->get_local_position();
	mav_attitude_ = mav_base_->get_attitude();
#ifndef RUN_SITL
	mav_rc_channels_ = mav_base_->get_channels();
#endif
	std::cout << "reference_state:\n" <<navigator_setpoint_ << std::endl;

	state_vector_ << mav_local_position_.x - navigator_setpoint_.x(),
			mav_local_position_.y - navigator_setpoint_.y(),
			mav_local_position_.z - navigator_setpoint_.z(),
			mav_local_position_.vx - navigator_setpoint_.vx(),
			mav_local_position_.vy - navigator_setpoint_.vy(),
			mav_local_position_.vz - navigator_setpoint_.vz();
	augmented_state_vector_ << state_vector_, control_vector_;
	critic_augmented_state_kronecker_ = vec_to_svec_transform_matrix_ * kroneckerProduct(augmented_state_vector_, augmented_state_vector_);
	critic_radial_basis_matrix_ = compute_critic_radial_basis_matrix(current_time_);
	actor_radial_basis_matrix_ = compute_actor_radial_basis_matrix(current_time_);
}
void QLearningController::critic()
{
	critic_augmented_state_kronecker_t sigma;
	critic_augmented_state_kronecker_t sigma_f;

	sigma << critic_radial_basis_matrix_ * (critic_augmented_state_kronecker_ - critic_augmented_state_kronecker_prior_);
	sigma_f << critic_augmented_state_kronecker_;

	double critic_error1 = critic_weight_vector_.transpose() * (critic_radial_basis_matrix_ * critic_augmented_state_kronecker_
			- compute_critic_radial_basis_matrix(current_time_ - time_resolution) * critic_augmented_state_kronecker_prior_)
			+ 0.5 * ((state_vector_.transpose() * state_penalty_matrix_).dot(state_vector_)
			+ (control_vector_.transpose() * control_penalty_matrix_).dot(control_vector_)) * time_resolution;
	double critic_error2 = 0.5 * (state_vector_.transpose() * terminal_riccati_matrix_).dot(state_vector_)
			- (critic_weight_vector_.transpose() * critic_radial_basis_matrix_).dot(critic_augmented_state_kronecker_);

	critic_weight_vector_t critic_weight_dot = -critic_convergence_rate
			* ((1.0/pow(1 + sigma.transpose()*sigma, 2))*sigma*critic_error1
			+ (1.0/pow(1 + sigma_f.transpose()*sigma_f, 2))*sigma_f*critic_error2);

	critic_weight_vector_ += critic_weight_dot * time_resolution;
}
void QLearningController::actor()
{
	augmented_state_matrix_t Q_hat;
	Q_hat << (vec_to_svec_transform_matrix_.transpose() *
				(2 * critic_radial_basis_matrix_.transpose() * critic_weight_vector_)
				).reshaped(AUGMENTED_STATE_VECTOR_SIZE,AUGMENTED_STATE_VECTOR_SIZE);
	control_matrix_t Quu = Q_hat.bottomRightCorner<QLEARNING_CONTROL_VECTOR_SIZE, QLEARNING_CONTROL_VECTOR_SIZE>();
	control_vector_t actor_error = actor_weight_matrix_.transpose() * actor_radial_basis_matrix_ * state_vector_
			+ Quu.inverse() * Q_hat.bottomLeftCorner<QLEARNING_CONTROL_VECTOR_SIZE, QLEARNING_STATE_VECTOR_SIZE>() * state_vector_;
	actor_weight_matrix_t actor_weight_dot = -actor_convergence_rate * state_vector_ * actor_error.transpose();

	actor_weight_matrix_ += actor_weight_dot * time_resolution;
}
void QLearningController::estimate_Q()
{
	Q_function_ = critic_weight_vector_.transpose() * critic_radial_basis_matrix_ * critic_augmented_state_kronecker_;
}
void QLearningController::compute_control()
{
	control_vector_ = actor_weight_matrix_.transpose() * actor_radial_basis_matrix_ * state_vector_;
}

critic_radial_basis_matrix_t QLearningController::compute_critic_radial_basis_matrix(double t)
{
	(void) t; // fix unused time argument warnings
	return critic_radial_basis_matrix_t::Identity();
}
actor_radial_basis_matrix_t QLearningController::compute_actor_radial_basis_matrix(double t)
{
	(void) t; // fix unused time argument warnings
	return actor_radial_basis_matrix_t::Identity();
}
void QLearningController::compute_svec_matrix()
{
	svec_matrix_t vec_to_svec_transform_matrix;
	vec_to_svec_transform_matrix.setZero();
	size_t n = AUGMENTED_STATE_VECTOR_SIZE;
	for (size_t j = 0; j < n; ++j)
	{
		for(size_t i = j; i < n; ++i)
		{
			critic_augmented_state_kronecker_t uij;
			uij.setZero();
			uij(j*n + i - (j+1)*j/2) = 1.0;
			augmented_state_matrix_t Tij;
			Tij.setZero();
			if (i == j)
			{
				Tij(i, j) = 1.0;
			} else {
				Tij(i, j) = 1.0/sqrt(2.0);
				Tij(j, i) = 1.0/sqrt(2.0);
			}

			vec_to_svec_transform_matrix += uij * Tij.reshaped().transpose();
		}
	}
	vec_to_svec_transform_matrix_ = vec_to_svec_transform_matrix;
}
void QLearningController::publish_control_mellinger()
{
	Vector3d thrust_des;
	mavAttitudeControl mav_control;
	Vector3d x_c_des;
	double yaw_des = EIGEN_PI / 2.0;
	Vector3d z_axis;
	Vector3d x_B_desired;
	Vector3d y_B_desired;
	Vector3d z_B_desired;
	Matrix3d R_des;
	Matrix3d e_R_matrix;
	Vector3d e_R;
	double thrust_proj;
	Vector3d rpy;
	Vector3d rpy_des;

	Quaterniond q(mav_attitude_.q[0],mav_attitude_.q[1],mav_attitude_.q[2],mav_attitude_.q[3]);
	rpy = mav_base::quaternion_to_rpy_wrap(mav_attitude_.q[0],mav_attitude_.q[1],mav_attitude_.q[2],mav_attitude_.q[3]);
	Matrix3d R = dcm(rpy(0), rpy(1), rpy(2));
	//Matrix3d R = q.toRotationMatrix();
	z_axis = R.col(2);

	thrust_des = control_vector_ + Vector3d(0,0,-mav_base::GRAVITY);
	thrust_proj = thrust_des.dot(-z_axis);

	x_c_des << cos(yaw_des), sin(yaw_des), 0;
	z_B_desired = -thrust_des.normalized();
	y_B_desired = (z_B_desired.cross(x_c_des)).normalized();
	x_B_desired = y_B_desired.cross(z_B_desired);
	R_des.col(0) = x_B_desired;
	R_des.col(1) = y_B_desired;
	R_des.col(2) = z_B_desired;

	e_R_matrix = (R_des.transpose() * R - R.transpose() * R_des);
	e_R << 0.5 * e_R_matrix(2,1),
			0.5 * e_R_matrix(0,2),
			0.5 * e_R_matrix(1,0);
	rpy_des = rpy - e_R;


	mav_control.roll = rpy_des(0);
	mav_control.pitch = rpy_des(1);
	mav_control.yaw = rpy_des(2);
	mav_control.thrust_normalized = thrust_proj;
	mav_control.q_d = rpy_to_quaternion(rpy_des(0), rpy_des(1), rpy_des(2));
	mav_base_->publish_attitude_control(mav_control);
}


int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<uam_controllers::QLearningController>()->get_node_interface());
	rclcpp::shutdown();
	return 0;
}