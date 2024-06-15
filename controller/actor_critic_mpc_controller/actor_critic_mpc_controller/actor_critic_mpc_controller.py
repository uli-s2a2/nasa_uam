from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize
from actor_critic_mpc.envs.ActorCriticMpcBasicUAM import ActorCriticMpcBasicUAM
from actor_critic_mpc.utils.enums import QuadrotorModel
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from navigator_msgs.msg import NavigatorTrajectorySetpoint
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from crazyflie_interfaces.msg import LogDataGeneric
from crazyflie_interfaces.msg import AttitudeSetpoint

import os
from functools import partial
from tf_transformations import quaternion_matrix, euler_from_quaternion
from enum import Enum

from scipy.spatial.transform import Rotation

import importlib.resources
from time import time

class State:
    def __init__(self):
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.quaternion = np.zeros(4)
        self.uam_state = np.array([0.5, 0.5, 0.8])
        self.ready = False

    def set_position(self, position):
        self.position[0] = position[0]
        self.position[1] = position[1]
        self.position[2] = position[2]

    def set_velocity(self, velocity):
        self.velocity[0] = velocity[0]
        self.velocity[1] = velocity[1]
        self.velocity[2] = velocity[2]

    def set_quaternion(self, quaternion):
        self.quaternion[0] = quaternion[0]
        self.quaternion[1] = quaternion[1]
        self.quaternion[2] = quaternion[2]
        self.quaternion[3] = quaternion[3]

    def get_rotation_matrix(self):
        return quaternion_matrix(self.quaternion)

class ActorCriticMpc(Node):
    class Motors(Enum):
        MOTOR_CLASSIC = 1 # https://store.bitcraze.io/products/4-x-7-mm-dc-motor-pack-for-crazyflie-2 w/ standard props
        MOTOR_UPGRADE = 2 # https://store.bitcraze.io/collections/bundles/products/thrust-upgrade-bundle-for-crazyflie-2-x

    run_number = 8
    training_number = 1
    # steps = 2359296
    # steps = 2457600
    # steps = 1146880
    steps = 1835008
    # steps = 1671168
    model_path_zip = importlib.resources.path(f'actor_critic_mpc.saved_models.uam.body_rate_control_quaternion.Run{run_number}.training{training_number}',f'uam_actor_critic_mpc_quadrotor_{steps}_steps.zip')
    model_path_pkl = importlib.resources.path(f'actor_critic_mpc.saved_models.uam.body_rate_control_quaternion.Run{run_number}.training{training_number}',f'uam_actor_critic_mpc_quadrotor_vecnormalize_{steps}_steps.pkl')

    def __init__(self, n_agents:int, frequency:int=100):
        super().__init__('actor_critic_mpc_node')
        test_env = make_vec_env(ActorCriticMpcBasicUAM, env_kwargs=dict(model=QuadrotorModel.BODY_RATE_CTRL_QUATERNION, dt=0.05, render_mode='human'), n_envs=1)
        self.env = VecNormalize.load(self.model_path_pkl, test_env)
        self.model = PPO.load(self.model_path_zip, backprop=False, device='cpu', env=self.env)
        self.get_logger().info(f'Loaded model from {self.model_path_zip}')
        self.model.set_env(self.env)
        self.n_agents = n_agents
        self.cf_names = [f'cf_{agent_n}' for agent_n in range(1, self.n_agents+1)]

        self.cf_states_ready = np.full(self.n_agents, False, dtype=bool)
        # self.cf_states_ready = np.full((self.n_agents, 2), False, dtype=bool)
        self.cf_nav_ready = np.full(self.n_agents, False, dtype=bool)
        self.initialized_control = np.full(self.n_agents, False, dtype=bool)

        self.agent_states = [State() for _ in range(self.n_agents)]
        self.nav_setpoints = [NavigatorTrajectorySetpoint() for _ in range(self.n_agents)]

        self.last_navigator_msg_stamp = 0.
        self.nav_timeout = 0.5
        self.motors = self.Motors.MOTOR_UPGRADE

        self.m = 0.027
        self.g = 9.81
        self.action_std = np.array([2.0, 2.0, 2.0, 1.0*self.g])
        self.action_mean = np.array([0.0, 0.0, 0.0, self.g])

        self.start_time = None

        self.attitude_setpoint_publishers = []
        
        for n in range(n_agents):
            cf_name = self.cf_names[n]

            # self.create_subscription(
            #     PoseStamped,
            #     f'/{cf_name}/pose',
            #     partial(self._pose_msg_callback, index=n, cf_states=self.agent_states, ready=self.cf_states_ready),
            #     10)
            
            # self.create_subscription(
            #     LogDataGeneric,
            #     f'/{cf_name}/velocity',
            #     partial(self._velocity_msg_callback, index=n, cf_states=self.agent_states, ready=self.cf_states_ready),
            #     10)

            self.create_subscription(
                Odometry,
                f'/{cf_name}/odom',
                partial(self._odom_msg_callback, index=n, cf_states=self.agent_states, ready=self.cf_states_ready),
                10)
            
            self.create_subscription(
                NavigatorTrajectorySetpoint,
                f'/{cf_name}/navigator/trajectory_setpoint',
                partial(self._navigator_setpoint_msg_callback, index=n, cf_nav_setpoints=self.nav_setpoints, ready=self.cf_nav_ready, stamp=self.last_navigator_msg_stamp),
                10)
            
            self.attitude_setpoint_publishers.append(self.create_publisher(AttitudeSetpoint,
                                                                           f'/{cf_name}/cmd_attitude_setpoint',
                                                                           10))
            
        self.create_timer(1. / frequency, self._control_publisher_callback)
        self.create_timer(1. / 10, self._navigator_timeout_callback)

    # def _pose_msg_callback(self, msg:PoseStamped, index:int, cf_states:list[State], ready):
    #     cf_states[index].set_position([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    #     cf_states[index].set_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    #     ready[index][0] = True

    # def _velocity_msg_callback(self, msg:LogDataGeneric, index:int, cf_states:list[State], ready):
    #     cf_states[index].set_velocity([msg.values[0], msg.values[1], msg.values[2]])
    #     ready[index][1] = True

    def _odom_msg_callback(self, msg:Odometry, index:int, cf_states:list[State], ready):
        cf_states[index].set_position([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        cf_states[index].set_velocity([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        cf_states[index].set_quaternion([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])
        ready[index] = True
    
    def _navigator_setpoint_msg_callback(self, msg:NavigatorTrajectorySetpoint, index:int, cf_nav_setpoints:list[NavigatorTrajectorySetpoint], ready, stamp):
        if msg.controller_type == NavigatorTrajectorySetpoint.CONTROLLER_TYPE_ACMPC:
            cf_nav_setpoints[index] = msg
            ready[index] = True
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1.e9
        else:
            ready[index] = False

    def _navigator_timeout_callback(self):
        for n in range(self.n_agents):
            if self.cf_nav_ready[n] and (rclpy.clock.Clock().now().to_msg().sec + rclpy.clock.Clock().now().to_msg().nanosec/1.e9 - self.last_navigator_msg_stamp > self.nav_timeout):
                self.cf_nav_ready[n] = False
    
    def _control_publisher_callback(self):
        if not any(self.cf_states_ready[:]) or not any(self.cf_nav_ready[:]):
            # self.get_logger().info('Waiting for all agents to be ready')
            return
        
        # Figure out which agents we need to compute the control for
        agents_to_control = np.where(self.cf_states_ready & self.cf_nav_ready)[0]
            
        obs = self.get_observations(agents_to_control)
        normalized_obs = self.normalize_observation(obs)
        # start_time = time()
        actions, _ = self.model.predict(self.np_to_torch(normalized_obs).to('cpu'), deterministic=True)
        # self.get_logger().info(f"observation: {obs}")
        # self.get_logger().info(f"normalized_obs: {normalized_obs}")
        # self.get_logger().info(f"Time to get control: {time() - start_time}")
        controls = actions * self.action_std + self.action_mean
        # self.get_logger().info(f"Controls: {controls}")
        for n in agents_to_control:
            if not self.initialized_control[n]:
                self.initialized_control[n] = True
                self.publish_control(np.zeros(4), n)
            else:
                self.publish_control(controls[n], n)

    def publish_control(self, control:np.ndarray, index:int):
        roll_rate, pitch_rate, yaw_rate, normalized_thrust = control.ravel()
        setpoint = AttitudeSetpoint()
        # rpy = euler_from_quaternion([self.agent_states[index].quaternion[-1], self.agent_states[index].quaternion[0], self.agent_states[index].quaternion[1], self.agent_states[index].quaternion[2]])
        rpy = Rotation.from_quat([self.agent_states[index].quaternion[1], self.agent_states[index].quaternion[2], self.agent_states[index].quaternion[3], self.agent_states[index].quaternion[0]]).as_euler('XYZ', degrees=True)
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds
        time = (self.get_clock().now().nanoseconds - self.start_time)/10**9
        setpoint.roll = np.clip(np.degrees(roll_rate), -30.0, 30.0) * np.tanh(time)
        setpoint.pitch = np.clip(np.degrees(pitch_rate), -30.0, 30.0) * np.tanh(time)
        # setpoint.roll = np.clip(np.degrees(roll_rate)*0.05 + rpy[0], -5.0, 5.0)
        # setpoint.pitch = np.clip(np.degrees(pitch_rate)*0.05 + rpy[1], -5.0, 5.0)
        # setpoint.yaw_rate = -np.clip(np.degrees(yaw_rate), -30.0, 30.0)
        setpoint.thrust = int(self.thrust_to_pwm(normalized_thrust * self.m))
        self.attitude_setpoint_publishers[index].publish(setpoint)

    def thrust_to_pwm(self, collective_thrust: float) -> int:
        # omega_per_rotor = 7460.8*np.sqrt((collective_thrust / 4.0))
        # pwm_per_rotor = 24.5307*(omega_per_rotor - 380.8359)
        if self.motors == self.Motors.MOTOR_CLASSIC:
            return max(min(24.5307*(7460.8*np.sqrt((collective_thrust / 4.0)) - 380.8359), 65535),0)
        elif self.motors == self.Motors.MOTOR_UPGRADE:
            return max(min(24.5307*(6462.1*np.sqrt((collective_thrust / 4.0)) - 380.8359), 65535),0)
        
    @staticmethod
    def _quaternion_to_dcm(q) -> np.ndarray:
        qw, qx, qy, qz = q
        return np.array([[qw**2 + qx**2 - qy**2 - qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                         [2*qx*qy + 2*qz*qw, qw**2 - qx**2 + qy**2 - qz**2, 2*qy*qz - 2*qx*qw],
                         [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, qw**2 - qx**2 - qy**2 + qz**2]])
    
    @staticmethod
    def point_to_np(point: Point) -> np.ndarray:
        return np.array([point.x, point.y, point.z])
    
    def get_observations(self, agents_to_control):
        for n in agents_to_control:
            cf_state = self.agent_states[n]
            cf_nav_setpoint = self.nav_setpoints[n]

            obs = np.zeros((agents_to_control.size, 21))

            if cf_nav_setpoint.current_waypoint < len(cf_nav_setpoint.path.poses) - 1:
                obs[n, 0:3] = cf_state.uam_state
                obs[n, 3:6] = cf_state.position - self.point_to_np(cf_nav_setpoint.path.poses[cf_nav_setpoint.current_waypoint].pose.position)
                obs[n, 6:9] = cf_state.position - self.point_to_np(cf_nav_setpoint.path.poses[cf_nav_setpoint.current_waypoint+1].pose.position)
                obs[n, 9:12] = cf_state.velocity
                obs[n, 12:21] = self._quaternion_to_dcm(cf_state.quaternion).flatten()
            else:
                obs[n, 0:3] = cf_state.uam_state
                obs[n, 3:6] = cf_state.position - self.point_to_np(cf_nav_setpoint.path.poses[cf_nav_setpoint.current_waypoint].pose.position)
                obs[n, 6:9] = cf_state.position - self.point_to_np(cf_nav_setpoint.path.poses[cf_nav_setpoint.current_waypoint].pose.position)
                obs[n, 9:12] = cf_state.velocity
                obs[n, 12:21] = self._quaternion_to_dcm(cf_state.quaternion).flatten()
        return obs
    
    def normalize_observation(self, obs):
        return self.env.normalize_obs(obs)
    
    @staticmethod
    def np_to_torch(x):
        return torch.FloatTensor(x)