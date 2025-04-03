#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String, Float64MultiArray
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3
import mujoco
import mujoco.viewer
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory


class QuadrupedGaitController(Node):
    def __init__(self):
        super().__init__('mujoco_simulator')
        
        # Load MuJoCo model
        try:
            package_share_dir = get_package_share_directory('b2_description')
            model_path = os.path.join(package_share_dir, 'xml', 'scene.xml')
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            return
        
        # Get joint names
        self.joint_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt)
        ]
        
        # Define home position - standing pose
        self.home_positions = {
            "FL_hip_joint": 0.0, "FL_thigh_joint": 0.7, "FL_calf_joint": -1.3,
            "RL_hip_joint": 0.0, "RL_thigh_joint": 0.7, "RL_calf_joint": -1.3,
            "FR_hip_joint": 0.0, "FR_thigh_joint": 0.7, "FR_calf_joint": -1.3,
            "RR_hip_joint": 0.0, "RR_thigh_joint": 0.7, "RR_calf_joint": -1.3
        }

        # Direction control (forward/backward)
        self.direction = "forward"  # default direction
        self.create_subscription(String, 'gait_direction', self.direction_callback, 10)
        
        # Dictionary to store subscribers
        self.subscribers = {}
        
        # Initialize subscribers for forward direction
        self.setup_direction_subscribers(self.direction)
        
        # Set initial target positions to home
        self.target_positions = self.home_positions.copy()
        
        # Improved control parameters for faster movement
        self.kp = 600.0  # Position gain - increased for faster response
        self.kd = 50.0   # Damping gain - increased for stability
        self.error_threshold = 0.2 # Radians - slightly increased for faster transitions
           
        self.leg_trajectories = {
            'FL': [], 'FR': [], 'RL': [], 'RR': []
        }
        self.current_trajectory_index = 0
        self.trajectory_received = False
        
        # Gait parameters
        self.current_gait_index = 0
        self.is_stable = False
        self.stable_counter = 0
        self.steps_to_wait = int(0.05 / self.model.opt.timestep)  # Reduced wait time (0.1 seconds)
        
        # Initial robot position
        self.data.qpos[0:3] = [0, 0, 0.65]  # x, y, z position
        self.data.qvel[:] = 0.0
        
        # Forward kinematics to update positions
        mujoco.mj_forward(self.model, self.data)
        
        # Define proper gait pattern with leg lift
        # Each phase is defined as [hip, thigh, calf] for each leg
        # self.define_gait_patterns()
        self.points = 11
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'mujoco_joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Add IMU publisher
        self.imu_pub = self.create_publisher(Imu, 'trunk_imu', 10)
        
        # Launch viewer and simulation timer
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.create_timer(self.model.opt.timestep, self.simulation_step)
        
        # Stabilization phase
        self.stabilization_steps = 200
        self.step_count = 0
        
        self.get_logger().info(f"Quadruped gait controller initialized with 11 gait points")

    def setup_direction_subscribers(self, direction):
        """Setup subscribers based on direction"""
        # First, unsubscribe from existing subscribers
        for sub in self.subscribers.values():
            self.destroy_subscription(sub)
        self.subscribers.clear()
        
        # Create new subscribers based on direction
        direction_topic = f'_joint_trajectory_{direction}'
        self.subscribers['FL'] = self.create_subscription(
            Float64MultiArray, 
            f'FL{direction_topic}',
            lambda msg: self.trajectory_callback(msg, 'FL'), 
            10)
        self.subscribers['FR'] = self.create_subscription(
            Float64MultiArray, 
            f'FR{direction_topic}',
            lambda msg: self.trajectory_callback(msg, 'FR'), 
            10)
        self.subscribers['RL'] = self.create_subscription(
            Float64MultiArray, 
            f'RL{direction_topic}',
            lambda msg: self.trajectory_callback(msg, 'RL'), 
            10)
        self.subscribers['RR'] = self.create_subscription(
            Float64MultiArray, 
            f'RR{direction_topic}',
            lambda msg: self.trajectory_callback(msg, 'RR'), 
            10)
        
        self.get_logger().info(f"Subscribed to {direction} trajectory topics")
        
        # Clear existing trajectories when changing direction
        self.leg_trajectories = {
            'FL': [], 'FR': [], 'RL': [], 'RR': []
        }
        self.trajectory_received = False
        self.current_trajectory_index = 0

    def trajectory_callback(self, msg, leg):
        """Store received trajectory for a leg."""
        data = np.array(msg.data).reshape(-1, 3)  # Reshape to [N, 3]
        self.leg_trajectories[leg] = data.tolist()
        print(self.leg_trajectories[leg])
        
        if all(len(traj) > 0 for traj in self.leg_trajectories.values()):
            self.trajectory_received = True
            self.get_logger().info("All trajectories received!")
    
    def execute_trajectory(self):

        gait_idx = self.current_gait_index

        """Update targets from stored trajectory."""
        for leg in ['FL', 'FR', 'RL', 'RR']:
            traj = self.leg_trajectories[leg]
            self.points = len(traj)
            if len(traj) == 0:
                continue
            
            # Cycle through trajectory points
            point = traj[self.current_trajectory_index % len(traj)]
            # print(point)
            self.target_positions[f"{leg}_hip_joint"] = point[0]
            self.target_positions[f"{leg}_thigh_joint"] = point[1] 
            self.target_positions[f"{leg}_calf_joint"] = point[2] 
        
        self.current_gait_index += 1
        self.current_trajectory_index += 1

    
    def direction_callback(self, msg):
        """Handle direction changes"""
        new_direction = msg.data.lower()
        if new_direction in ["forward", "backward"]:
            if self.direction != new_direction:
                self.direction = new_direction
                self.get_logger().info(f"Switching direction to {new_direction}")
                # Setup new subscribers for the new direction
                self.setup_direction_subscribers(new_direction)
                # Reset gait parameters
                self.current_gait_index = 0
                self.is_stable = False
                self.stable_counter = 0
    
    def simulation_step(self):
        # Stabilization phase
        if self.step_count < self.stabilization_steps:
            # Use home position during stabilization
            self.target_positions = self.home_positions.copy()
            self.step_count += 1
            if self.step_count % 50 == 0:
                self.get_logger().info(f"Stabilizing: {self.step_count}/{self.stabilization_steps}")
        else:
            # Gait execution phasse

            self.execute_gait()
            
            # if self.trajectory_received:
            #     self.execute_trajectory()
        # Apply improved position control with velocity component
        self.apply_improved_control()
        
        # Step the simulation
        mujoco.mj_step(self.model, self.data)
        
        # Update viewer
        if self.viewer.is_running():
            self.viewer.sync()
        
        # Publish state
        self.publish_joint_states()
        self.publish_tf()
        
        # Get IMU sensor data
        # Get accelerometer data
        accel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "body_acc")
        if accel_id != -1:
            accel_data = self.data.sensordata[self.model.sensor_adr[accel_id]:self.model.sensor_adr[accel_id]+3]
        
        # Get gyroscope data
        gyro_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "body_gyro")
        if gyro_id != -1:
            gyro_data = self.data.sensordata[self.model.sensor_adr[gyro_id]:self.model.sensor_adr[gyro_id]+3]
        
        # Get orientation quaternion
        quat_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "body_quat")
        if quat_id != -1:
            quat_data = self.data.sensordata[self.model.sensor_adr[quat_id]:self.model.sensor_adr[quat_id]+4]
        
        # Create and publish IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        # Set orientation
        if 'quat_data' in locals():
            imu_msg.orientation.w = quat_data[0]
            imu_msg.orientation.x = quat_data[1]
            imu_msg.orientation.y = quat_data[2]
            imu_msg.orientation.z = quat_data[3]
        
        # Set angular velocity (gyroscope data)
        if 'gyro_data' in locals():
            imu_msg.angular_velocity.x = gyro_data[0]
            imu_msg.angular_velocity.y = gyro_data[1]
            imu_msg.angular_velocity.z = gyro_data[2]
        
        # Set linear acceleration (accelerometer data)
        if 'accel_data' in locals():
            imu_msg.linear_acceleration.x = accel_data[0]
            imu_msg.linear_acceleration.y = accel_data[1]
            imu_msg.linear_acceleration.z = accel_data[2]
        
        # Set covariance matrices (you can adjust these values based on your sensor's characteristics)
        imu_msg.orientation_covariance = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]
        imu_msg.angular_velocity_covariance = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]
        imu_msg.linear_acceleration_covariance = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]
        
        # Publish IMU message
        self.imu_pub.publish(imu_msg)
    
    def execute_gait(self):
        # If robot is stable at current point, consider moving to next point
        if self.is_stable:
            self.stable_counter += 1
            if self.stable_counter >= self.steps_to_wait:
                # Time to move to next gait point
                self.move_to_next_gait_point()
        else:
            # Check if we've reached the target position (within error threshold)
            current_error = self.calculate_max_joint_error()
            if current_error < self.error_threshold:
                self.is_stable = True
                self.stable_counter = 0
                self.get_logger().info(f"Stable at gait point {self.current_gait_index} (error: {current_error:.4f})")
    
    def move_to_next_gait_point(self):
        # Reset stability flags
        self.is_stable = False
        self.stable_counter = 0
        
        # Move to next point in sequence
        # num_points = self.points        
        # self.current_gait_index = (self.current_gait_index + 1) % num_points
        
        self.get_logger().info(f"Moving to gait point {self.current_gait_index}")

        self.execute_trajectory()

        
    
    def update_target_positions(self):
        # Get current gait point for each leg
        gait_idx = self.current_gait_index
        
        # Handle direction (forward/backward)
        if self.direction == "backward":
            # For backward movement, we invert hip angles and use reversed sequence
            num_points = len(self.gait_points["FL"])
            gait_idx = (num_points - 1) - self.current_gait_index
        
        # Update target for each leg
        for leg in ["FL", "FR", "RL", "RR"]:
            point = self.gait_points[leg][gait_idx].copy()
            
            # Invert hip angle for backward direction
            if self.direction == "backward":
                point[0] = -point[0]  # Invert hip angle
            
            # Set joint targets
            self.target_positions[f"{leg}_hip_joint"] = point[0]
            self.target_positions[f"{leg}_thigh_joint"] = point[1]
            self.target_positions[f"{leg}_calf_joint"] = point[2]
    
    def calculate_max_joint_error(self):
        max_error = 0.0
        
        for joint_name, target_pos in self.target_positions.items():
            # Find joint in MuJoCo model
            for i in range(self.model.njnt):
                if mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) == joint_name:
                    q_index = self.model.jnt_qposadr[i]
                    current_pos = self.data.qpos[q_index]
                    error = abs(target_pos - current_pos)
                    max_error = max(max_error, error)
                    break
        
        return max_error
    
    def apply_improved_control(self):
        # Enhanced PD control with feedforward term
        for i in range(self.model.nu):
            joint_id = self.model.actuator_trnid[i][0]
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            
            if joint_name in self.target_positions:
                q_index = self.model.jnt_qposadr[joint_id]
                v_index = self.model.jnt_dofadr[joint_id]
                
                target_angle = self.target_positions[joint_name]
                current_angle = self.data.qpos[q_index]
                current_vel = self.data.qvel[v_index]
                
                # PD control with limited torque
                error = target_angle - current_angle
                
                # Desired velocity component - faster response
                desired_vel = error   # Scale factor for desired velocity
                vel_error = desired_vel - current_vel
                
                # Combined control with position and velocity components
                torque = self.kp * error - self.kd * current_vel
                
                # Limit torque to prevent instability
                max_torque = 200.0
                torque = np.clip(torque, -max_torque, max_torque)
                
                # Apply torque
                self.data.ctrl[i] = torque
    
    def publish_joint_states(self):
        # Create and publish JointState message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.data.qpos[self.model.jnt_qposadr[i]] for i in range(self.model.njnt)]
        msg.velocity = [self.data.qvel[self.model.jnt_dofadr[i]] for i in range(self.model.njnt)]
        self.joint_pub.publish(msg)
    
    def publish_tf(self):
        # Publish transform from world to base
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'base'
        
        # Position
        tf_msg.transform.translation.x = self.data.qpos[0]
        tf_msg.transform.translation.y = self.data.qpos[1]
        tf_msg.transform.translation.z = self.data.qpos[2]
        
        # Orientation (quaternion)
        tf_msg.transform.rotation.w = self.data.qpos[3]
        tf_msg.transform.rotation.x = self.data.qpos[4]
        tf_msg.transform.rotation.y = self.data.qpos[5]
        tf_msg.transform.rotation.z = self.data.qpos[6]
        
        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    simulator = QuadrupedGaitController()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        simulator.get_logger().info("Simulation stopped by user")
    finally:
        # Cleanup
        if hasattr(simulator, 'viewer') and simulator.viewer is not None:
            simulator.viewer.close()
        simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


