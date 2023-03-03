#include <uam_control/qlearning_controller.hpp>

using namespace uam_control;
using namespace rclcpp;
using namespace Eigen;

QLearningController::QLearningController() : rclcpp::Node("uam_control")
{
	compute_svec_matrix();
	state_vector_.setZero();
	control_vector_.setZero();
	control_vector_(2) = -2.0;
	navigator_setpoint_.setZero();
	critic_augmented_state_kronecker_prior_.setZero();
	// set initial weight
	critic_weight_vector_ <<   48.28,-0.00,-0.00,54.14,-0.00,0.00,14.14,-0.00,0.00,48.28,-0.00,-0.00,54.14,-0.00,
								-0.00,14.14,-0.00,48.28,-0.00,-0.00,54.14,-0.00,-0.00,14.14,54.14,-0.00,-0.00,20.00,
								-0.00,-0.00,54.14,-0.00,-0.00,20.00,-0.00,54.14,-0.00,-0.00,20.00,5.00,0.00,0.00,5.00,0.00,5.00;
	actor_weight_matrix_ <<    -2.00,0.00,-0.00,0.00,-2.00,0.00,0.00,0.00,-2.00,-2.83,0.00,0.00,0.00,-2.83,0.00,0.00,0.00,-2.83;
	terminal_riccati_matrix_ << 28.28,-0.00,-0.00,10.00,-0.00,0.00,-0.00,28.28,-0.00,-0.00,10.00,-0.00,-0.00,-0.00,
								28.28,-0.00,-0.00,10.00,10.00,-0.00,-0.00,14.14,-0.00,-0.00,-0.00,10.00,-0.00,-0.00,
								14.14,-0.00,0.00,-0.00,10.00,-0.00,-0.00,14.14;
	state_penalty_matrix_.setZero();
	state_penalty_matrix_.diagonal() << 20.0, 20.0, 20.0, 10.0, 10.0, 10.0;
	control_penalty_matrix_.setZero();
	control_penalty_matrix_.diagonal() << 5.0, 5.0, 5.0;
	start_time_ = this->get_clock()->now();
	// ----------------------- Subscribers --------------------------
	auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
	navigator_setpoint_sub_ =
			this->create_subscription<nav_msgs::msg::Odometry>(
					"/navigator/position_setpoint", qos_sub,
					[this](const nav_msgs::msg::Odometry::UniquePtr msg) {
						if (vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED
								&& vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
							navigator_setpoint_.x() = msg->pose.pose.position.x;
							navigator_setpoint_.y() = msg->pose.pose.position.y;
							navigator_setpoint_.z() = msg->pose.pose.position.z;
							navigator_setpoint_.vx() = msg->twist.twist.linear.x;
							navigator_setpoint_.vy() = msg->twist.twist.linear.y;
							navigator_setpoint_.vz() = msg->twist.twist.linear.z;

							setup();
							compute_control();
							publish_control_mellinger();
						}
					});
	vehicle_status_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", qos_sub,
					[this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
						vehicle_status_ = *msg;
					});
	vehicle_odometry_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos_sub,
					[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
					    vehicle_odometry_ = *msg;
					});
	// ----------------------- Publishers ---------------------------
	auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

	vehicle_attitude_setpoint_pub_ =
			this->create_publisher<uam_control_msgs::msg::AttitudeSetpoint>("/uam_control/attitude_setpoint", qos_pub);
	qlearning_status_pub_ =
			this->create_publisher<uam_control_msgs::msg::QLearningStatus>("/uam_control/qlearning_status", qos_pub);

	auto timer_callback = [this]() -> void
	{
		if (can_learn()) {
			//------- Q-Learning -------------
			learn();
			publish_qlearning_status();
		} else {
			start_time_ = this->get_clock()->now();
		}
	};

	timer_ = this->create_wall_timer(std::chrono::milliseconds(QLEARNING_CALLBACK_RATE_MS), timer_callback);
}

bool QLearningController::can_learn()
{
	bool out;
	out = vehicle_odometry_.position[2] > QLEARNING_MINIMUM_LEARNING_ALTITUDE &&
			vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
			vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
	return out;
}

void QLearningController::setup()
{

	state_vector_ << vehicle_odometry_.position[0] - navigator_setpoint_.x(),
					 vehicle_odometry_.position[1] - navigator_setpoint_.y(),
					 vehicle_odometry_.position[2] - navigator_setpoint_.z(),
				 	 vehicle_odometry_.velocity[0] - navigator_setpoint_.vx(),
				 	 vehicle_odometry_.velocity[1] - navigator_setpoint_.vy(),
					 vehicle_odometry_.velocity[2] - navigator_setpoint_.vz();
}
void QLearningController::learn()
{
	double current_time = (this->get_clock()->now().seconds() - start_time_.seconds());

	// Setup
	setup();
	augmented_state_vector_ << state_vector_, control_vector_;
	critic_augmented_state_kronecker_ = vec_to_svec_transform_matrix_ * kroneckerProduct(augmented_state_vector_, augmented_state_vector_);
	critic_radial_basis_matrix_ = compute_critic_radial_basis_matrix(current_time);
	actor_radial_basis_matrix_ = compute_actor_radial_basis_matrix(current_time);

	// Critic
	critic_augmented_state_kronecker_t sigma;
	critic_augmented_state_kronecker_t sigma_f;

	sigma << critic_radial_basis_matrix_ * (critic_augmented_state_kronecker_ - critic_augmented_state_kronecker_prior_);
	sigma_f << critic_augmented_state_kronecker_;

	double critic_error1 = critic_weight_vector_.transpose() * (critic_radial_basis_matrix_ * critic_augmented_state_kronecker_
			- compute_critic_radial_basis_matrix(current_time - time_resolution) * critic_augmented_state_kronecker_prior_)
			+ 0.5 * ((state_vector_.transpose() * state_penalty_matrix_).dot(state_vector_)
			+ (control_vector_.transpose() * control_penalty_matrix_).dot(control_vector_)) * time_resolution;
	double critic_error2 = 0.5 * (state_vector_.transpose() * terminal_riccati_matrix_).dot(state_vector_)
			- (critic_weight_vector_.transpose() * critic_radial_basis_matrix_).dot(critic_augmented_state_kronecker_);

	critic_weight_vector_t critic_weight_dot = -critic_convergence_rate
			* ((1.0/pow(1 + sigma.transpose()*sigma, 2))*sigma*critic_error1
			+ (1.0/pow(1 + sigma_f.transpose()*sigma_f, 2))*sigma_f*critic_error2);

	critic_weight_vector_ += critic_weight_dot * time_resolution;
	critic_augmented_state_kronecker_prior_ = critic_augmented_state_kronecker_;


	// Actor
	augmented_state_matrix_t Q_hat;
	Q_hat << (vec_to_svec_transform_matrix_.transpose() *
				(2 * critic_radial_basis_matrix_.transpose() * critic_weight_vector_)
				).reshaped(AUGMENTED_STATE_VECTOR_SIZE,AUGMENTED_STATE_VECTOR_SIZE);
	control_matrix_t Quu = Q_hat.bottomRightCorner<QLEARNING_CONTROL_VECTOR_SIZE, QLEARNING_CONTROL_VECTOR_SIZE>();
	control_vector_t actor_error = actor_weight_matrix_.transpose() * actor_radial_basis_matrix_ * state_vector_
			+ Quu.inverse() * Q_hat.bottomLeftCorner<QLEARNING_CONTROL_VECTOR_SIZE, QLEARNING_STATE_VECTOR_SIZE>() * state_vector_;
	actor_weight_matrix_t actor_weight_dot = -actor_convergence_rate * state_vector_ * actor_error.transpose();

	actor_weight_matrix_ += actor_weight_dot * time_resolution;

	// Q value
	Q_function_ = critic_weight_vector_.transpose() * critic_radial_basis_matrix_ * critic_augmented_state_kronecker_;
}
void QLearningController::compute_control()
{
	double current_time = (this->get_clock()->now().seconds() - start_time_.seconds());
	control_vector_ = actor_weight_matrix_.transpose() * compute_actor_radial_basis_matrix(current_time) * state_vector_;
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
	uam_control_msgs::msg::AttitudeSetpoint attitude_setpoint;
	Vector3d x_c_des;
	double yaw_des = EIGEN_PI / 2.0;
	Vector3d z_axis;
	Vector3d x_B_desired, y_B_desired, z_B_desired;
	double thrust_proj;
	Vector3d rpy;

	Quaterniond q(vehicle_odometry_.q[0], vehicle_odometry_.q[1], vehicle_odometry_.q[2], vehicle_odometry_.q[3]);
	Matrix3d R = q.toRotationMatrix();
	//Matrix3d R = q.toRotationMatrix();
	z_axis = R.col(2);

	//TODO common config params
	thrust_des = control_vector_ + Vector3d(0,0,-9.80665);
	thrust_proj = thrust_des.dot(-z_axis);

	x_c_des << cos(yaw_des), sin(yaw_des), 0;
	z_B_desired = -thrust_des.normalized();
	y_B_desired = (z_B_desired.cross(x_c_des)).normalized();
	x_B_desired = y_B_desired.cross(z_B_desired);

	Matrix3d R_d;
	R_d.col(0) = x_B_desired;
	R_d.col(1) = y_B_desired;
	R_d.col(2) = z_B_desired;
	Vector3d rpy_d = R_d.eulerAngles(0,1,2);
	Quaterniond q_d(R_d);
//	e_R_matrix = (R_des.transpose() * R - R.transpose() * R_des);
//	e_R << 0.5 * e_R_matrix(2,1),
//			0.5 * e_R_matrix(0,2),
//			0.5 * e_R_matrix(1,0);
//	rpy_des = rpy - e_R;
	attitude_setpoint.header.stamp = this->get_clock()->now();
	attitude_setpoint.header.frame_id = "map_ned";
	attitude_setpoint.roll_body = rpy_d(0);
	attitude_setpoint.pitch_body = rpy_d(1);
	attitude_setpoint.yaw_body = rpy_d(2);
	attitude_setpoint.thrust_body_normalized = thrust_proj;
	attitude_setpoint.q_d[0] = q_d.w();
	attitude_setpoint.q_d[1] = q_d.x();
	attitude_setpoint.q_d[2] = q_d.y();
	attitude_setpoint.q_d[3] = q_d.z();
	vehicle_attitude_setpoint_pub_->publish(attitude_setpoint);
}

void QLearningController::publish_qlearning_status()
{
	uam_control_msgs::msg::QLearningStatus qlearning_status;

	qlearning_status.stamp = this->get_clock()->now();
	qlearning_status.start_time = start_time_;
	for(size_t i = 0; i < qlearning_status.actor_weight_matrix.size(); i++)
		qlearning_status.actor_weight_matrix[i] = *(actor_weight_matrix_.data() + i);

	for(size_t i = 0; i < qlearning_status.actor_weight_matrix.size(); i++)
		qlearning_status.critic_weight_vector[i] = critic_weight_vector_[i];

	for(size_t i = 0; i < qlearning_status.actor_weight_matrix.size(); i++)
		qlearning_status.control_vector[i] = control_vector_[i];

	qlearning_status_pub_->publish(qlearning_status);
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<uam_control::QLearningController>());
	rclcpp::shutdown();
	return 0;
}