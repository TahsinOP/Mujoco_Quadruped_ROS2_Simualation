#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class GaitTrajectorySubscriber(Node):
    def __init__(self):
        super().__init__('gait_trajectory_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'LR_joint_trajectory',  # Change this to other legs if needed
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        try:
            data = np.array(msg.data)
            reshaped_data = data.reshape(-1, 3)  # shape into [timesteps x 3 joints]
            self.get_logger().info(f"Received {reshaped_data.shape[0]} timesteps of joint angles.")

            # Print first 5 entries for sanity check
            for i, (hip, thigh, knee) in enumerate(reshaped_data[:50]):
                self.get_logger().info(f"Step {i}: Hip={hip:.3f}, Thigh={thigh:.3f}, Knee={knee:.3f}")
        except Exception as e:
            self.get_logger().error(f"Error in callback: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = GaitTrajectorySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
