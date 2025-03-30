#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

class WaypointMarkerServer(Node):
    def __init__(self):
        super().__init__('interactive_waypoint_publisher')
        
        # Create interactive marker server
        self.server = InteractiveMarkerServer(self, "waypoint_markers")
        
        # Waypoint publisher
        self.waypoint_pub = self.create_publisher(
            PoseStamped, '/waypoints', 10
        )
        
        # Waypoint counter and list
        self.waypoint_counter = 0
        self.waypoints = []
        
        # Create initial marker
        self.create_initial_marker()
        
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
        
        # 6-DOF controls
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
            # Create and publish waypoint
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
            
            # Publish waypoint
            self.waypoint_pub.publish(waypoint)
            self.get_logger().info(f"Published Waypoint {self.waypoint_counter}: "
                                    f"x={waypoint.pose.position.x}, "
                                    f"y={waypoint.pose.position.y}")
            
            # Store the waypoint
            self.waypoints.append(waypoint)
            
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