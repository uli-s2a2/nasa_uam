import argparse
import numpy as np

from crazyflie_py import *

import rclpy

from rclpy import executors
from rclpy.qos import qos_profile_sensor_data
from actor_critic_mpc_controller.actor_critic_mpc_controller import ActorCriticMpc

def main():
    # parser = argparse.ArgumentParser()
    # args = parser.parse_args()

    rclpy.init()
    node = ActorCriticMpc(n_agents=1, frequency=100)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
