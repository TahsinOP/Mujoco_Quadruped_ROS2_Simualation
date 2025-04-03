import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String
import numpy as np
from math import pi
from kinematics import Kinematics

class GaitTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('gait_trajectory_publisher')
        
        # Define leg names
        self.legs = ['FL', 'RL', 'RR', 'FR']
        
        # Create publishers dynamically and store them in a dictionary
        self.trajectory_publishers = {
            leg: self.create_publisher(Float64MultiArray, f'{leg}_joint_trajectory_forward', 10)
            for leg in self.legs
        }
        
        # Direction publisher
        self.direction_pub = self.create_publisher(String, 'gait_direction', 10)
        
        # Home pose angles (hip, thigh, knee)
        self.home_angles = [0.02, -0.7, 1.31]
        
        # Generate 10-11 point trajectories
        self.forward_trajectories = self.generate_trot_trajectory()
        
        # Set up a timer to publish trajectories periodically
        self.publish_timer = self.create_timer(0.5, self.publish_all_trajectories)
        
        self.get_logger().info("Gait trajectory publisher initialized")
    
    def generate_trot_trajectory(self):
        """Generate 10-11 point trajectory for a trotting gait using FL as reference."""
        kin = Kinematics()
        
        num_points = 4 # Define number of trajectory points
        step_length = 0.05  # Forward step length
        step_height = 0.075# Swing height
        phase_shift = 0.5  # Stance and swing phase offset
        
        # Generate swing trajectory for FL (Forward Left) leg
        trajectory = []
        for i in range(num_points):
            phase = i / (num_points - 1)
            if phase < phase_shift:
                # Stance phase: Move leg backward (relative motion)
                x_offset = step_length * (phase / phase_shift)
                z_offset = 0.0
            else:
                # Swing phase: Move leg forward and lift
                swing_phase = (phase - phase_shift) / (1 - phase_shift)
                x_offset = step_length * swing_phase
                z_offset = step_height * np.sin(swing_phase * pi)
            
            foot_pos = np.array([x_offset, 0.0, z_offset]) + [0.29, 0.2, -0.51]
            print(foot_pos)
            joint_angles = kin.leg_IK(foot_pos, legID=0)[:3]
            hip, thigh, knee = joint_angles
            thigh = pi * 3 / 2 - thigh + 0.8
            hip -= 0.18
            knee = -(0.5 + knee)

            trajectory.append([hip,thigh,knee])
        
        # Mirror FL trajectory for other legs as needed
        trajectories = {
            'FL': trajectory,
            'RR': trajectory,  # Diagonal leg moves the same way
            'FR': trajectory[::-1],  # Opposite phase for trotting
            'RL': trajectory[::-1]   # Opposite phase for trotting
        }
        
        return trajectories
    
    def create_msg(self, traj):
        """Convert trajectory data to a Float64MultiArray message."""
        msg = Float64MultiArray()
        msg.data = [angle for triple in traj for angle in triple]
        
        msg.layout.dim = [
            MultiArrayDimension(label="timesteps", size=len(traj), stride=len(msg.data)),
            MultiArrayDimension(label="joints", size=3, stride=3)
        ]
        return msg
    
    def publish_all_trajectories(self):
        """Publish trajectories for all legs."""
        direction_msg = String()
        direction_msg.data = "forward"
        self.direction_pub.publish(direction_msg)
        
        for leg in self.legs:
            self.trajectory_publishers[leg].publish(self.create_msg(self.forward_trajectories[leg]))
        
        self.get_logger().info("Published forward trotting gait trajectory")


def main(args=None):
    rclpy.init(args=args)
    node = GaitTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
