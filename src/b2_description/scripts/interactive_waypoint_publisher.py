#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import numpy as np
from scipy.interpolate import CubicSpline
from nav_msgs.msg import Path

class WaypointMarkerServer(Node):
    def __init__(self):
        super().__init__('interactive_waypoint_publisher')
        
        # Create interactive marker server
        self.server = InteractiveMarkerServer(self, "waypoint_markers")
        
        # Publishers
        self.waypoint_pub = self.create_publisher(
            PoseStamped, '/waypoints', 10
        )
        self.path_pub = self.create_publisher(
            Path, '/smooth_path', 10
        )
        self.path_vis_pub = self.create_publisher(
            Marker, '/path_visualization', 10
        )
        
        # Waypoint counter and list
        self.waypoint_counter = 0
        self.waypoints = []
        
        # Create timer for publishing waypoints and path (5Hz)
        self.create_timer(0.2, self.publish_data)
        
        # Create initial marker
        self.create_initial_marker()

    def interpolate_path(self):
        """Create a smooth interpolated path from waypoints"""
        if len(self.waypoints) < 2:
            return None, None
            
        # Extract x and y coordinates from waypoints
        x_coords = [wp.pose.position.x for wp in self.waypoints]
        y_coords = [wp.pose.position.y for wp in self.waypoints]
        
        # Create parameter t (cumulative distance along path)
        t = np.zeros(len(x_coords))
        for i in range(1, len(x_coords)):
            dx = x_coords[i] - x_coords[i-1]
            dy = y_coords[i] - y_coords[i-1]
            t[i] = t[i-1] + np.sqrt(dx*dx + dy*dy)
        
        # Check if we have valid parameterization
        if len(np.unique(t)) != len(t):
            # If points are coincident, add small offsets to make t strictly increasing
            eps = 1e-6
            for i in range(1, len(t)):
                if t[i] <= t[i-1]:
                    t[i] = t[i-1] + eps
        
        try:
            # Create cubic spline interpolation
            cs_x = CubicSpline(t, x_coords)
            cs_y = CubicSpline(t, y_coords)
            
            # Generate points along the path
            t_new = np.linspace(0, t[-1], num=100)
            x_new = cs_x(t_new)
            y_new = cs_y(t_new)
            
            return x_new, y_new
        except Exception as e:
            self.get_logger().warn(f"Path interpolation failed: {str(e)}")
            return None, None

    def create_path_message(self, x_coords, y_coords):
        """Create Path message from interpolated coordinates"""
        path_msg = Path()
        path_msg.header.frame_id = "base"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in zip(x_coords, y_coords):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.1
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        return path_msg

    def create_path_visualization(self, x_coords, y_coords):
        """Create visualization marker for the path"""
        marker = Marker()
        marker.header.frame_id = "base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03  # Line width
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

    def publish_data(self):
        """Publish waypoints, interpolated path and visualization"""
        if not self.waypoints:
            return
            
        # Get current time for header
        current_time = self.get_clock().now().to_msg()
        
        # Publish stored waypoints
        for waypoint in self.waypoints:
            updated_waypoint = PoseStamped()
            updated_waypoint.header.frame_id = "base"
            updated_waypoint.header.stamp = current_time
            updated_waypoint.pose = waypoint.pose
            self.waypoint_pub.publish(updated_waypoint)
        
        # Create and publish interpolated path
        x_new, y_new = self.interpolate_path()
        if x_new is not None and y_new is not None:
            # Publish Path message
            path_msg = self.create_path_message(x_new, y_new)
            self.path_pub.publish(path_msg)
            
            # Publish visualization
            vis_marker = self.create_path_visualization(x_new, y_new)
            self.path_vis_pub.publish(vis_marker)
        
    def create_initial_marker(self):
        """Create the first interactive marker"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base"
        int_marker.name = f"waypoint_{self.waypoint_counter}"
        int_marker.description = f"Waypoint {self.waypoint_counter}"
        
        # Set initial pose with explicit float values
        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.1
        int_marker.pose.orientation.w = 1.0
        int_marker.pose.orientation.x = 0.0
        int_marker.pose.orientation.y = 0.0
        int_marker.pose.orientation.z = 0.0
        
        # Visual marker (sphere)
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.6
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.8
        
        # Controls
        controls = []
        
        # Sphere control (always visible)
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        controls.append(control)
        
        # Move XY Plane
        control = InteractiveMarkerControl()
        control.name = "move_xy"
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        controls.append(control)
        
        # Add controls to marker
        int_marker.controls = controls
        
        # Insert marker
        self.server.insert(int_marker)
        
        # Set callback
        self.server.setCallback(int_marker.name, self.marker_feedback)
        
        # Apply changes
        self.server.applyChanges()
        
    def marker_feedback(self, feedback):
        """Handle marker feedback"""
        # Check if marker is being moved
        if feedback.event_type == feedback.POSE_UPDATE:
            # Create waypoint
            waypoint = PoseStamped()
            waypoint.header = feedback.header
            waypoint.pose = feedback.pose
            
            # Ensure float values
            waypoint.pose.position.x = float(waypoint.pose.position.x)
            waypoint.pose.position.y = float(waypoint.pose.position.y)
            waypoint.pose.position.z = 0.1 # Keep z at 0
            
            waypoint.pose.orientation.w = 1.0
            waypoint.pose.orientation.x = 0.0
            waypoint.pose.orientation.y = 0.0
            waypoint.pose.orientation.z = 0.0
            
            # Store the waypoint
            self.waypoints.append(waypoint)
            
            self.get_logger().info(f"Stored Waypoint {self.waypoint_counter}: "
                                    f"x={waypoint.pose.position.x}, "
                                    f"y={waypoint.pose.position.y}")
            
            # Increment counter
            self.waypoint_counter += 1
            
            # Create a new marker at the last waypoint's position
            self.create_next_marker(waypoint.pose)
        
    def create_next_marker(self, last_pose):
        """Create a new marker at the last waypoint's position"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base"
        int_marker.name = f"waypoint_{self.waypoint_counter}"
        int_marker.description = f"Waypoint {self.waypoint_counter}"
        
        # Set initial pose to the last waypoint's position
        int_marker.pose = last_pose
        
        # Visual marker (sphere)
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.6
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.8
        
        # Controls
        controls = []
        
        # Sphere control (always visible)
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        controls.append(control)
        
        # Move XY Plane
        control = InteractiveMarkerControl()
        control.name = "move_xy"
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        controls.append(control)
        
        # Add controls to marker
        int_marker.controls = controls
        
        # Insert marker
        self.server.insert(int_marker)
        
        # Set callback
        self.server.setCallback(int_marker.name, self.marker_feedback)
        
        # Apply changes
        self.server.applyChanges()

def main():
    rclpy.init()
    try:
        marker_server = WaypointMarkerServer()
        rclpy.spin(marker_server)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()