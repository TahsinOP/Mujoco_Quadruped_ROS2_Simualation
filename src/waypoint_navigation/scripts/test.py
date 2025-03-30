import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import numpy as np
from math import sin, pi
from kinematics import Kinematics


class GaitTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('gait_trajectory_publisher')

        # Individual publishers for each leg
        self.pub_LF = self.create_publisher(Float64MultiArray, 'LF_joint_trajectory', 10)
        self.pub_LB = self.create_publisher(Float64MultiArray, 'LB_joint_trajectory', 10)
        self.pub_RB = self.create_publisher(Float64MultiArray, 'RB_joint_trajectory', 10)
        self.pub_RF = self.create_publisher(Float64MultiArray, 'RF_joint_trajectory', 10)

        self.duration = 1.0
        self.dt = 0.01
        self.trajs = self.generate_gait_trajectories(self.duration, self.dt)

        self.timer = self.create_timer(0.1, self.publish_all_leg_trajectories)

    def foot_trajectory(self, phase, step_length=0.06, step_height=0.03):
        beta = 0.5
        if phase < (1 - beta):
            swing_phase = phase / (1 - beta)
            x = (swing_phase - 0.5) * step_length
            z = step_height * sin(pi * swing_phase)
        else:
            stance_phase = (phase - (1 - beta)) / beta
            x = (0.5 - stance_phase) * step_length
            z = 0.0
        return np.array([x, 0.0, z])

    def generate_gait_trajectories(self, duration=1.0, dt=0.01):
        kin = Kinematics()
        num_steps = int(duration / dt)
        home_positions = {
            0: np.array([ 0.3,  0.2, -0.51]),  # LF
            1: np.array([-0.3,  0.2, -0.51]),  # LB
            2: np.array([-0.3, -0.2, -0.51]),  # RB
            3: np.array([ 0.3, -0.2, -0.51])   # RF
        }
        trajectories = {leg: [] for leg in range(4)}
        phase_offsets = {0: 0.0, 1: 0.5, 2: 0.0, 3: 0.5}

        for step in range(num_steps):
            t = step * dt
            for leg in range(4):
                phase = (t + phase_offsets[leg]) % 1.0
                foot_offset = self.foot_trajectory(phase)
                foot_pos = home_positions[leg] + foot_offset
                joint_angles = kin.leg_IK(foot_pos, legID=leg)[:3]

                # Apply joint angle corrections
                hip, thigh, knee = joint_angles
                if leg in [1, 2]:
                    thigh = pi * 3 / 2 - thigh - 0.1
                else:
                    thigh = pi * 3 / 2 - thigh + 0.8

                hip -= 0.18 if leg in [0, 1] else 2.9
                knee = -(0.5 + knee)

                trajectories[leg].append([hip, thigh, knee])

        return trajectories

    def create_msg(self, traj):
        msg = Float64MultiArray()
        flat_data = [angle for triple in traj for angle in triple]
        msg.data = flat_data

        # Layout to help subscriber reshape
        dim_time = MultiArrayDimension()
        dim_time.label = "timesteps"
        dim_time.size = len(traj)
        dim_time.stride = len(flat_data)

        dim_joint = MultiArrayDimension()
        dim_joint.label = "joints"
        dim_joint.size = 3
        dim_joint.stride = 3

        msg.layout.dim = [dim_time, dim_joint]
        return msg

    def publish_all_leg_trajectories(self):
        self.pub_LF.publish(self.create_msg(self.trajs[0]))
        self.pub_LB.publish(self.create_msg(self.trajs[1]))
        self.pub_RB.publish(self.create_msg(self.trajs[2]))
        self.pub_RF.publish(self.create_msg(self.trajs[3]))

        self.get_logger().info("Published structured gait trajectories to all leg topics.")


def main(args=None):
    rclpy.init(args=args)
    node = GaitTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
