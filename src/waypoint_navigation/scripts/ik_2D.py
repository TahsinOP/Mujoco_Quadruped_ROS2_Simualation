import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray,String
import time

class LegIK:
    def __init__(self, l1, l2):
        """
        Initialize the leg inverse kinematics solver
        
        Args:
            l1 (float): Length of the first link (hip to knee)
            l2 (float): Length of the second link (knee to foot)
        """
        self.l1 = l1  # Length of first link
        self.l2 = l2  # Length of second link

    def solve_ik(self, x, y, direction='forward'):
        """
        Solve inverse kinematics for a 2D leg
        
        Args:
            x (float): Target x position of the foot
            y (float): Target y position of the foot
            direction (str): 'forward' or 'backward' to determine theta1 calculation
            
        Returns:
            tuple: (theta1, theta2) joint angles in radians
        """
        try:
            # Calculate the distance from origin to target point
            r = np.sqrt(x**2 + y**2)
            
            # Check if the target is reachable
            if r > (self.l1 + self.l2) or r < abs(self.l1 - self.l2):
                raise ValueError(f"Target position ({x:.3f}, {y:.3f}) is outside workspace. Distance: {r:.3f}")

            # Calculate knee angle using cosine law
            cos_theta2 = (x**2 + y**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
            # Negative because we want the knee to bend forward (clockwise)
            theta2 = -np.arccos(np.clip(cos_theta2, -1.0, 1.0))
            
            # Calculate angles for hip angle
            beta = np.arctan2(x, y)
            cos_alpha = (x**2 + y**2 + self.l1**2 - self.l2**2) / (2 * self.l1 * r)
            alpha = np.arccos(np.clip(cos_alpha, -1.0, 1.0))
            
            # Calculate theta1 based on direction
            if direction == 'forward':
                theta1 = alpha
                theta2 = theta2  # For forward motion
            else:
                theta1 = -alpha + np.pi/2 
                theta2 = theta2  # For backward motion
            
            # Ensure angles are within the specified ranges (-π/2 to π/2)
            theta1 = np.clip(theta1, -np.pi/2, np.pi/2)
            theta2 = np.clip(theta2, -np.pi/2, np.pi/2)

            return theta1, theta2
            
        except Exception as e:
            raise ValueError(f"IK calculation failed: {str(e)}")

    def forward_kinematics(self, theta1, theta2):
        """
        Compute forward kinematics for verification
        
        Args:
            theta1 (float): Hip angle in radians (from +Y axis)
            theta2 (float): Knee angle in radians (relative to first link)
            
        Returns:
            tuple: (x, y) coordinates of the foot position
        """
        # Since theta1 is measured from +Y axis, we use sin for x and cos for y
        x = self.l1 * np.sin(theta1) + self.l2 * np.sin(theta1 + theta2)
        y = self.l1 * np.cos(theta1) + self.l2 * np.cos(theta1 + theta2)
        return x, y

class TrotGaitGenerator(Node):
    def __init__(self):
        super().__init__('trot_gait_generator')
        
        # Create publishers for each leg (forward)
        self.fr_pub_forward = self.create_publisher(Float64MultiArray, 'FR_joint_trajectory_forward', 10)
        self.fl_pub_forward = self.create_publisher(Float64MultiArray, 'FL_joint_trajectory_forward', 10)
        self.rr_pub_forward = self.create_publisher(Float64MultiArray, 'RR_joint_trajectory_forward', 10)
        self.rl_pub_forward = self.create_publisher(Float64MultiArray, 'RL_joint_trajectory_forward', 10)
        
        # Create publishers for each leg (backward)
        self.fr_pub_backward = self.create_publisher(Float64MultiArray, 'FR_joint_trajectory_backward', 10)
        self.fl_pub_backward = self.create_publisher(Float64MultiArray, 'FL_joint_trajectory_backward', 10)
        self.rr_pub_backward = self.create_publisher(Float64MultiArray, 'RR_joint_trajectory_backward', 10)
        self.rl_pub_backward = self.create_publisher(Float64MultiArray, 'RL_joint_trajectory_backward', 10)

        self.gait_direction_pub = self.create_publisher(String, 'gait_direction', 10)
        
        # Initialize leg IK solver
        self.leg_ik = LegIK(l1=0.32, l2=0.35)
        
        # Gait parameters
        self.cycle_time = 1.0
        self.stance_phase = 0.5
        self.home_x = 0.02
        self.home_y = 0.53
        self.stride_length = 0.1
        self.stride_height = 0.1
        
        # Phase offsets for each leg (trot gait)
        self.phase_offsets = {
            'FR': 0.0,
            'FL': 0.5,
            'RR': 0.5,
            'RL': 0.0
        }
        
        # Generate and publish waypoints for both directions
        self.create_timer(0.1, self.generate_and_publish_trajectories)

    def generate_waypoints(self, direction='forward'):
        """Generate waypoints for given direction"""
        if direction == 'forward':
            # Forward motion: start from home, move in +x direction
            stance_x = np.linspace(self.stride_length/2, -self.stride_length/2, 3)
            swing_x = np.linspace(-self.stride_length/2, self.stride_length/2, 3)
        else:  # backward
            # Backward motion: start from home, move in -x direction
            stance_x = np.linspace(-self.stride_length/2, -self.stride_length*1.5, 3)
            swing_x = np.linspace(-self.stride_length*1.5, -self.stride_length/2, 3)
        
        # Y coordinates remain same for both directions
        stance_y = np.zeros_like(stance_x)
        swing_y = -self.stride_height * np.sin(np.linspace(0, np.pi, 3))
        
        # Combine waypoints
        waypoints_x = np.concatenate([stance_x, swing_x])
        waypoints_y = np.concatenate([stance_y, swing_y])
        
        # Add home position offset
        waypoints_x += self.home_x
        waypoints_y += self.home_y

        # Log waypoints for debugging
        if direction == 'backward':
            self.get_logger().info(f"Backward waypoints X: {waypoints_x}")
            self.get_logger().info(f"Backward waypoints Y: {waypoints_y}")
        
        return waypoints_x, waypoints_y

    def calculate_leg_trajectory(self, waypoints_x, waypoints_y, phase_offset=0.0, direction='forward'):
        """Calculate complete trajectory for a leg with given phase offset and direction"""
        trajectory = []
        num_points = len(waypoints_x)
        
        # Reorder waypoints based on phase offset
        offset_indices = [(i + int(phase_offset * num_points)) % num_points 
                         for i in range(num_points)]
        
        # Generate trajectory with phase offset
        for idx in offset_indices:
            x = waypoints_x[idx]
            y = waypoints_y[idx]
            
            try:
                thigh, knee = self.leg_ik.solve_ik(x, y, direction)
                trajectory.extend([0.0, thigh, knee])
            except ValueError as e:
                self.get_logger().error(f"IK failed for point {idx}: {str(e)}")
                thigh, knee = self.leg_ik.solve_ik(self.home_x, self.home_y, direction)
                trajectory.extend([0.0, thigh, knee])
        
        return trajectory

    def generate_and_publish_trajectories(self):
        try:
            # Generate waypoints for both directions
            forward_x, forward_y = self.generate_waypoints('forward')
            backward_x, backward_y = self.generate_waypoints('backward')
            
            # Calculate trajectories for forward motion
            fr_trajectory_forward = self.calculate_leg_trajectory(forward_x, forward_y, 
                                                               self.phase_offsets['FR'], 
                                                               'forward')
            fl_trajectory_forward = self.calculate_leg_trajectory(forward_x, forward_y, 
                                                               self.phase_offsets['FL'], 
                                                               'forward')
            rr_trajectory_forward = self.calculate_leg_trajectory(forward_x, forward_y, 
                                                               self.phase_offsets['RR'], 
                                                               'forward')
            rl_trajectory_forward = self.calculate_leg_trajectory(forward_x, forward_y, 
                                                               self.phase_offsets['RL'], 
                                                               'forward')
            
            # Calculate trajectories for backward motion
            fr_trajectory_backward = self.calculate_leg_trajectory(backward_x, backward_y, 
                                                                self.phase_offsets['FR'], 
                                                                'backward')
            fl_trajectory_backward = self.calculate_leg_trajectory(backward_x, backward_y, 
                                                                self.phase_offsets['FL'], 
                                                                'backward')
            rr_trajectory_backward = self.calculate_leg_trajectory(backward_x, backward_y, 
                                                                self.phase_offsets['RR'], 
                                                                'backward')
            rl_trajectory_backward = self.calculate_leg_trajectory(backward_x, backward_y, 
                                                                self.phase_offsets['RL'], 
                                                                'backward')

            # Log some angles for verification
            self.get_logger().info("\nSample angles (FR leg):")
            self.get_logger().info(f"Forward first point: {fr_trajectory_forward[0:3]}")
            self.get_logger().info(f"Backward first point: {fr_trajectory_backward[0:3]}")
            
            # Create and publish messages
            # Forward messages
            fr_msg_forward = Float64MultiArray(data=fr_trajectory_forward)
            fl_msg_forward = Float64MultiArray(data=fl_trajectory_forward)
            rr_msg_forward = Float64MultiArray(data=rr_trajectory_forward)
            rl_msg_forward = Float64MultiArray(data=rl_trajectory_forward)
            
            # Backward messages
            fr_msg_backward = Float64MultiArray(data=fl_trajectory_backward)
            fl_msg_backward = Float64MultiArray(data=fr_trajectory_backward)
            rr_msg_backward = Float64MultiArray(data=rl_trajectory_backward)
            rl_msg_backward = Float64MultiArray(data=rr_trajectory_backward)
            
            # Publish all messages
            self.fr_pub_forward.publish(fr_msg_forward)
            self.fl_pub_forward.publish(fl_msg_forward)
            self.rr_pub_forward.publish(rr_msg_forward)
            self.rl_pub_forward.publish(rl_msg_forward)
            
            self.fr_pub_backward.publish(fr_msg_backward)
            self.fl_pub_backward.publish(fl_msg_backward)
            self.rr_pub_backward.publish(rr_msg_backward)
            self.rl_pub_backward.publish(rl_msg_backward)

            self.gait_direction_pub.publish(String(data='backward'))
            
            self.get_logger().info("Successfully published all trajectories")
            
        except Exception as e:
            self.get_logger().error(f"Failed to generate and publish trajectories: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = TrotGaitGenerator()
    
    try:
        node.get_logger().info("Node is ready")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
