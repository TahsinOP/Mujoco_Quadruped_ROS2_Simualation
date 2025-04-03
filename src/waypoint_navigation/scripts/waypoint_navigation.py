#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, Header
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from tf2_ros import TransformException, Buffer, TransformListener
import numpy as np
from scipy.interpolate import CubicSpline
from nav_msgs.msg import Path
from tf_transformations import euler_from_quaternion
import math
import threading
import sys

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # TF Buffer for odometry
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Interactive marker server
        self.server = InteractiveMarkerServer(self, "waypoint_markers")
        
        # Publishers
        self.path_vis_pub = self.create_publisher(Marker, '/path_visualization', 10)
        
        # Publishers for leg trajectories
        self.leg_publishers = {
            'FL': self.create_publisher(Float64MultiArray, 'FL_joint_trajectory_forward', 10),
            'FR': self.create_publisher(Float64MultiArray, 'FR_joint_trajectory_forward', 10),
            'RL': self.create_publisher(Float64MultiArray, 'RL_joint_trajectory_forward', 10),
            'RR': self.create_publisher(Float64MultiArray, 'RR_joint_trajectory_forward', 10)
        }
        
        # Initialize state variables
        self.waypoint_counter = 0
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.robot_position = [0.0, 0.0]  # x, y position
        self.current_yaw = 0.0
        self.interpolated_path = []
        
        # Control parameters
        self.distance_threshold = 0.12 # meters
        self.angle_threshold = 0.12 # radians
        self.max_hip_angle = 0.3 # maximum hip joint angle (radians)
        
        # State flags
        self.waypoint_generation_complete = False
        self.navigation_active = False
        
        self.get_logger().info('Waypoint navigator initialized')
        
    def start_waypoint_generation(self):
        """Start the waypoint generation process"""
        self.get_logger().info('Starting waypoint generation...')
        self.create_initial_marker()
        # Create timer for visualization during waypoint generation
        self.vis_timer = self.create_timer(0.2, self.publish_visualization)

    def complete_waypoint_generation(self):
        """Complete the waypoint generation and prepare for navigation"""
        self.waypoint_generation_complete = True
        self.get_logger().info('Waypoint generation completed')
        
        # Interpolate final path
        self.interpolate_path()
        
        # Destroy visualization timer and create navigation timer
        self.vis_timer.cancel()
        self.create_timer(0.1, self.control_loop)  # 10 Hz control loop
        self.create_timer(0.2, self.publish_visualization)  # 5 Hz visualization
        
        self.navigation_active = True
        self.get_logger().info('Starting navigation...')

    def get_robot_pose(self):
        """Get robot pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                'base',
                rclpy.time.Time())
            
            # Extract position
            self.robot_position[0] = transform.transform.translation.x
            self.robot_position[1] = transform.transform.translation.y
            
            # Extract yaw from quaternion
            q = transform.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.current_yaw = yaw
            self.get_logger().info(f'Robot pose: {self.robot_position}, {self.current_yaw}')
            
            return True
        except TransformException as ex:
            self.get_logger().warning(f'Could not get robot pose: {ex}')
            return False

    def interpolate_path(self):
        """Create a smooth interpolated path from waypoints"""
        if len(self.waypoints) < 2:
            return None, None
            
        x_coords = [wp.pose.position.x for wp in self.waypoints]
        y_coords = [wp.pose.position.y for wp in self.waypoints]
        
        t = np.zeros(len(x_coords))
        for i in range(1, len(x_coords)):
            dx = x_coords[i] - x_coords[i-1]
            dy = y_coords[i] - y_coords[i-1]
            t[i] = t[i-1] + np.sqrt(dx*dx + dy*dy)
        
        if len(np.unique(t)) != len(t):
            eps = 1e-6
            for i in range(1, len(t)):
                if t[i] <= t[i-1]:
                    t[i] = t[i-1] + eps
        
        try:
            cs_x = CubicSpline(t, x_coords)
            cs_y = CubicSpline(t, y_coords)
            
            t_new = np.linspace(0, t[-1], num=100)
            x_new = cs_x(t_new)
            y_new = cs_y(t_new)
            
            self.interpolated_path = list(zip(x_new, y_new))
            return x_new, y_new
        except Exception as e:
            self.get_logger().warn(f"Path interpolation failed: {str(e)}")
            return None, None

    def create_path_visualization(self, x_coords, y_coords):
        """Create visualization marker for the path"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        for x, y in zip(x_coords, y_coords):
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.1
            marker.points.append(point)
            
        return marker

    def calculate_target_yaw(self):
        """Calculate target yaw angle to next waypoint"""
        if not self.interpolated_path or self.current_waypoint_idx >= len(self.interpolated_path):
            return None
            
        target = self.interpolated_path[self.current_waypoint_idx]
        self.get_logger().info(f'Target: {target}')
        dx = target[0] - self.robot_position[0]
        dy = target[1] - self.robot_position[1]
        
        return math.atan2(dy, dx)

    def calculate_hip_angles(self, yaw_error):
        """Calculate hip angles for turning based on yaw error"""
        turn_factor = np.clip(yaw_error / (math.pi/2), -1.0, 1.0)
        self.get_logger().info(f'Turn factor: {turn_factor}')
        base_hip_angle = self.max_hip_angle * turn_factor
        
        return {
            'FL':-base_hip_angle,
            'FR': -base_hip_angle,
            'RL': base_hip_angle,
            'RR': base_hip_angle
        }

    def generate_leg_trajectory(self, hip_angle, leg_name):
        """Generate trajectory for a leg including turning motion"""
        points = 4
        trajectory = []
        phase_offset = math.pi if leg_name in ['FR', 'RL'] else 0.0
        
        for i in range(points):
            phase = (2 * math.pi * i / points) + phase_offset
            hip = hip_angle
            thigh = 0.71+ 0.18 * math.sin(phase)
            knee = -1.33 + 0.14 * math.sin(phase)
            trajectory.append([hip, thigh, knee])
            
        return trajectory

    def publish_leg_trajectories(self, hip_angles):
        """Publish trajectories for all legs"""
        for leg_name, hip_angle in hip_angles.items():
            trajectory = self.generate_leg_trajectory(hip_angle, leg_name)
            msg = Float64MultiArray()
            msg.data = [val for point in trajectory for val in point]
            self.leg_publishers[leg_name].publish(msg)

    def create_initial_marker(self):
        """Create the first interactive marker"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.name = f"waypoint_{self.waypoint_counter}"
        int_marker.description = f"Waypoint {self.waypoint_counter}"
        int_marker.pose.position.z = 0.1
        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.orientation.w = 1.0
        
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        marker.color.r = 0.6
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.8
        
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)
        
        move_control = InteractiveMarkerControl()
        move_control.name = "move_xy"
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_control.orientation.w = 1.0
        move_control.orientation.y = 1.0
        int_marker.controls.append(move_control)
        
        self.server.insert(int_marker)

        
        # Set callback
        self.server.setCallback(int_marker.name, self.marker_feedback)
        self.server.applyChanges()

    def marker_feedback(self, feedback):
        """Handle marker feedback"""
        if feedback.event_type == feedback.POSE_UPDATE and not self.waypoint_generation_complete:
            waypoint = PoseStamped()
            waypoint.header = feedback.header
            waypoint.pose = feedback.pose
            
            self.waypoints.append(waypoint)
            self.waypoint_counter += 1
            
            self.create_next_marker(waypoint.pose)
            self.interpolate_path()

    def create_next_marker(self, last_pose):
        """Create a new marker at the last waypoint's position"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.name = f"waypoint_{self.waypoint_counter}"
        int_marker.description = f"Waypoint {self.waypoint_counter}"
        int_marker.pose = last_pose
        
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        marker.color.r = 0.6
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.8
        
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)
        
        move_control = InteractiveMarkerControl()
        move_control.name = "move_xy"
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_control.orientation.w = 1.0
        move_control.orientation.y = 1.0
        int_marker.controls.append(move_control)
        
        self.server.insert(int_marker)
        
        # Set callback
        self.server.setCallback(int_marker.name, self.marker_feedback)
        self.server.applyChanges()

    def publish_visualization(self):
        """Publish path visualization"""
        if self.interpolated_path:
            x_coords, y_coords = zip(*self.interpolated_path)
            vis_marker = self.create_path_visualization(x_coords, y_coords)
            self.path_vis_pub.publish(vis_marker)

    def control_loop(self):
        """Main control loop for waypoint navigation"""
        if not self.navigation_active or not self.interpolated_path:
            return
            
        # Get robot pose from TF
        if not self.get_robot_pose():
            return
            
        # Calculate target yaw
        target_yaw = self.calculate_target_yaw()
        self.get_logger().info(f'Target yaw: {target_yaw}')
        if target_yaw is None:
            return
            
        # Calculate yaw error
        yaw_error = target_yaw - self.current_yaw
        yaw_error = ((yaw_error + math.pi) % (2 * math.pi)) - math.pi
        
        # Calculate and publish leg trajectories
        hip_angles = self.calculate_hip_angles(yaw_error)
        self.publish_leg_trajectories(hip_angles)
        
        # Check if reached current waypoint
        current_target = self.interpolated_path[self.current_waypoint_idx]
        distance = math.sqrt((current_target[0] - self.robot_position[0])**2 +
                           (current_target[1] - self.robot_position[1])**2)
                           

        if distance < self.distance_threshold:
            self.current_waypoint_idx += 1
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx}')
            if self.current_waypoint_idx >= len(self.interpolated_path):
                self.get_logger().info('Reached final waypoint')
                self.navigation_active = False

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    
    try:
        # Start a separate thread for ROS spinning
        thread = threading.Thread(target=rclpy.spin, args=(navigator,), daemon=True)
        thread.start()
        
        # Wait for user input to start waypoint generation
        input("Press Enter to start waypoint generation...")
        navigator.start_waypoint_generation()
        
        # Wait for user input to complete waypoint generation
        input("Press Enter when waypoint generation is complete...")
        navigator.complete_waypoint_generation()
        
        # Keep the main thread running
        thread.join()
        
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
