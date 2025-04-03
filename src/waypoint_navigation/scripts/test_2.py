import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String
import numpy as np
from math import pi
from kinematics import Kinematics
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
        
        # Default foot position at home pose (only for FL, others will be mirrored)
        self.fl_default_position = [0.29, 0.2, -0.51]
        
        # Generate trajectories for trotting gait
        self.forward_trajectories, self.foot_trajectories = self.generate_trot_trajectory()
        
        # Visualize the trajectories
        self.visualize_trajectories()
        
        # Set up a timer to publish trajectories periodically
        self.publish_timer = self.create_timer(0.5, self.publish_all_trajectories)
        
        self.get_logger().info("Gait trajectory publisher initialized")
    
    def generate_trot_trajectory(self):
        """Generate trajectory points for a trotting gait using only FL leg IK."""
        kin = Kinematics()
        
        # Trot parameters
        num_points = 10  # Number of points for a complete cycle
        step_length = 0.1 # Forward step length
        step_height = 0.075  # Swing height
        
        # Initialize containers
        fl_joint_trajectory = []
        fl_foot_trajectory = []
        
        # Generate trajectory for FL leg
        for i in range(num_points):
            phase = i / num_points
            
            # For the first half of the cycle, FL is in swing phase
            if phase < 0.5:
                # Normalized phase within swing
                swing_phase = phase * 2
                
                # Forward movement and lifting during swing
                x_offset = step_length * (swing_phase - 0.5)
                z_offset = step_height * np.sin(swing_phase * pi)
            else:
                # Normalized phase within stance
                stance_phase = (phase - 0.5) * 2
                
                # Backward movement during stance
                x_offset = step_length * (0.5 - stance_phase)
                z_offset = 0  # No lift during stance
            
            # Calculate foot position
            foot_pos = np.array([
                self.fl_default_position[0] + x_offset,
                self.fl_default_position[1],
                self.fl_default_position[2] + z_offset
            ])
            
            fl_foot_trajectory.append(foot_pos)
            
            # Calculate joint angles using inverse kinematics (only for FL)
            joint_angles = kin.leg_IK(foot_pos, legID=0)[:3]
            
            # Apply angle corrections for FL
            hip, thigh, knee = joint_angles
            hip -= 0.18
            thigh = pi * 3 / 2 - thigh + 0.8
            knee = -(0.5 + knee)
            
            fl_joint_trajectory.append([hip, thigh, knee])
        
        # Create trajectories for all legs by replicating FL trajectory with phase shifts
        joint_trajectories = {}
        foot_trajectories = {}
        
        # FL and RR move together (in phase)
        joint_trajectories['FL'] = fl_joint_trajectory
        foot_trajectories['FL'] = fl_foot_trajectory
        
        # RR uses the same trajectory as FL (but with adjusted angles for right side)
        joint_trajectories['RR'] = []
        foot_trajectories['RR'] = []
        for i, angles in enumerate(fl_joint_trajectory):
            # Mirror the hip angle for right side
            hip, thigh, knee = angles
            hip_rr = -hip   # Mirror and adjust for right side
            joint_trajectories['RR'].append([hip_rr, thigh, knee])
            
            # Mirror the foot position for RR
            fl_pos = fl_foot_trajectory[i]
            rr_pos = [-fl_pos[0], -fl_pos[1], fl_pos[2]]  # Mirror X and Y for rear-right
            foot_trajectories['RR'].append(rr_pos)
        
        # FR and RL are in opposite phase (when FL/RR are in swing, FR/RL are in stance)
        half_cycle = num_points // 2
        
        # FR: Use FL trajectory but shift phase by 180 degrees
        joint_trajectories['FR'] = []
        foot_trajectories['FR'] = []
        for i in range(num_points):
            idx = (i + half_cycle) % num_points
            hip, thigh, knee = fl_joint_trajectory[idx]
            hip_fr = -hip   # Mirror and adjust for right side
            joint_trajectories['FR'].append([hip_fr, thigh, knee])
            
            fl_pos = fl_foot_trajectory[idx]
            fr_pos = [fl_pos[0], -fl_pos[1], fl_pos[2]]  # Mirror Y for front-right
            foot_trajectories['FR'].append(fr_pos)
        
        # RL: Use FL trajectory but shift phase by 180 degrees
        joint_trajectories['RL'] = []
        foot_trajectories['RL'] = []
        for i in range(num_points):
            idx = (i + half_cycle) % num_points
            joint_trajectories['RL'].append(fl_joint_trajectory[idx])
            
            fl_pos = fl_foot_trajectory[idx]
            rl_pos = [-fl_pos[0], fl_pos[1], fl_pos[2]]  # Mirror X for rear-left
            foot_trajectories['RL'].append(rl_pos)
        
        return joint_trajectories, foot_trajectories
    
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
    
    def visualize_trajectories(self):
        """Visualize the foot trajectories for all legs."""
        fig = plt.figure(figsize=(12, 10))
        
        # Create 3D plot for foot positions
        ax1 = fig.add_subplot(221, projection='3d')
        colors = {'FL': 'r', 'FR': 'g', 'RL': 'b', 'RR': 'y'}
        
        for leg, positions in self.foot_trajectories.items():
            positions = np.array(positions)
            ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                    marker='o', linestyle='-', color=colors[leg], label=leg)
        
        ax1.set_xlabel('X (forward)')
        ax1.set_ylabel('Y (lateral)')
        ax1.set_zlabel('Z (vertical)')
        ax1.set_title('Foot Trajectories in 3D')
        ax1.legend()
        
        # Create 2D plot for X-Z plane (side view)
        ax2 = fig.add_subplot(222)
        for leg, positions in self.foot_trajectories.items():
            positions = np.array(positions)
            ax2.plot(positions[:, 0], positions[:, 2], 
                    marker='o', linestyle='-', color=colors[leg], label=leg)
        
        ax2.set_xlabel('X (forward)')
        ax2.set_ylabel('Z (vertical)')
        ax2.set_title('Foot Trajectories (Side View)')
        ax2.legend()
        
        # Create 2D plot for X-Y plane (top view)
        ax3 = fig.add_subplot(223)
        for leg, positions in self.foot_trajectories.items():
            positions = np.array(positions)
            ax3.plot(positions[:, 0], positions[:, 1], 
                    marker='o', linestyle='-', color=colors[leg], label=leg)
        
        ax3.set_xlabel('X (forward)')
        ax3.set_ylabel('Y (lateral)')
        ax3.set_title('Foot Trajectories (Top View)')
        ax3.legend()
        
        # Create plot for joint angles over time
        ax4 = fig.add_subplot(224)
        joint_names = ['Hip', 'Thigh', 'Knee']
        line_styles = ['-', '--', ':']
        
        for leg in self.legs:
            angles = np.array(self.forward_trajectories[leg])
            time_steps = np.arange(len(angles))
            
            for j, joint_name in enumerate(joint_names):
                ax4.plot(time_steps, angles[:, j], 
                        linestyle=line_styles[j], color=colors[leg], 
                        label=f'{leg} {joint_name}')
        
        ax4.set_xlabel('Time Step')
        ax4.set_ylabel('Joint Angle (rad)')
        ax4.set_title('Joint Angles Over Time')
        ax4.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=4)
        
        plt.tight_layout()
        plt.savefig('trot_gait_visualization.png')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = GaitTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()