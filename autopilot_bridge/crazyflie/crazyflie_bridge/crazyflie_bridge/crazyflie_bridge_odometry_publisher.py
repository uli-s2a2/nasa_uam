import numpy as np
from numpy.linalg import norm
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

from crazyflie_interfaces.msg import LogDataGeneric

from functools import partial

class CrazyflieBridge(Node):
    def __init__(self, n_agents, publish_rate=100):
        super().__init__('odom_publisher')

        self.n_agents = n_agents

        self.cf_names = [f'cf_{agent_n}' for agent_n in range(1, self.n_agents+1)]

        self.cf_odom_ready = np.full((self.n_agents, 2), False, dtype=bool)
        self.cf_odom = [Odometry() for _ in range(n_agents)]


        self.cf_odom_publishers = [self.create_publisher(Odometry, f'/{cf_name}/odom', 10) for cf_name in self.cf_names]


        for n in range(n_agents):
            cf_name = self.cf_names[n]

            self.create_subscription(
                PoseStamped,
                f'/{cf_name}/pose',
                partial(self._pose_msg_callback, index=n, cf_odom=self.cf_odom, ready=self.cf_odom_ready),
                10)
            
            self.create_subscription(
                LogDataGeneric,
                f'/{cf_name}/velocity',
                partial(self._velocity_msg_callback, index=n, cf_odom=self.cf_odom, ready=self.cf_odom_ready),
                10)
        
        self.create_timer(1.0 / publish_rate, self._publish_odom_loop)

    def _pose_msg_callback(self, msg:PoseStamped, index:int, cf_odom:list[Odometry], ready):
        cf_odom[index].pose.pose = msg.pose
        ready[index][0] = True

    def _velocity_msg_callback(self, msg:LogDataGeneric, index:int, cf_odom:list[Odometry], ready):
        cf_odom[index].twist.twist.linear.x = msg.values[0]
        cf_odom[index].twist.twist.linear.y = msg.values[1]
        cf_odom[index].twist.twist.linear.z = msg.values[2]
        ready[index][1] = True

    def _publish_odom_loop(self):
        for i in range(self.n_agents):
            
            pose_ready, vel_ready = self.cf_odom_ready[i]
            # self.get_logger().info(f'pose_ready: {pose_ready}, vel_ready: {vel_ready}')
            odom = self.cf_odom[i]

            if pose_ready and vel_ready:
                self.cf_odom_publishers[i].publish(odom)
                # ready[i] = [False, False]


def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieBridge(n_agents=1, publish_rate=100)
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
