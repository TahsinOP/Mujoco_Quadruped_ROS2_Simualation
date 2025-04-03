#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64MultiArray
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped
# import mujoco
# import mujoco.viewer
# import numpy as np
# import os
# from ament_index_python.packages import get_package_share_directory

# class MujocoSimulator(Node):
#     def __init__(self):
#         super().__init__('mujoco_simulator')
#         try:
#             package_share_dir = get_package_share_directory('b2_description')
#             model_path = os.path.join(package_share_dir, 'xml', 'scene.xml')
#         except Exception as e:
#             self.get_logger().error(f"Failed to locate package or XML: {e}")
#             return
        
#         self.model = mujoco.MjModel.from_xml_path(model_path)
#         self.data = mujoco.MjData(self.model)
        
#         # Initialize joint positions
#         self.joint_names = [
#             mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
#             for i in range(self.model.njnt)
#         ]
#         self.joint_positions = {name: 0.0 for name in self.joint_names}
        
#         # Define home position
#         self.home_positions = {
#             "FL_hip_joint": 0.0, "FL_thigh_joint": 0.7, "FL_calf_joint": -1.2,
#             "RL_hip_joint": 0.0, "RL_thigh_joint": 0.7, "RL_calf_joint": -1.2,
#             "FR_hip_joint": 0.0, "FR_thigh_joint": 0.7, "FR_calf_joint": -1.2,
#             "RR_hip_joint": 0.0, "RR_thigh_joint": 0.7, "RR_calf_joint": -1.2
#         }
        
#         # Trajectory buffer for each leg - initialized with None to track receipt status
#         self.trajectory_buffer = {"FL": None, "RL": None, "FR": None, "RR": None}
#         self.current_timestep = 0
#         self.total_timesteps = 5  # Match the number of points from publisher (5)
        
#         # Simulation phases
#         self.PHASE_STABILIZE = 0
#         self.PHASE_WAIT_FOR_DATA = 1
#         self.PHASE_EXECUTE_GAIT = 2
        
#         self.current_phase = self.PHASE_STABILIZE
#         self.stabilization_steps = 200
#         self.step_count = 0
#         self.cycle_count = 0
#         self.gait_cycle_time = 2.5  # seconds per gait cycle
#         self.steps_per_cycle = int(self.gait_cycle_time / self.model.opt.timestep)
#         self.steps_per_trajectory_point = self.steps_per_cycle // self.total_timesteps

#         # Initialize robot position
#         self.data.qpos[0:3] = [0, 0, 0.65]
#         self.data.qvel[:] = 0.0
#         # Forward kinematics to update body and joint positions
#         mujoco.mj_forward(self.model, self.data)
        
#         # Subscribers for each leg
#         self.create_subscription(
#             Float64MultiArray, 'FL_joint_trajectory', 
#             lambda msg: self.gait_callback(msg, "FL"), 10)
#         self.create_subscription(
#             Float64MultiArray, 'RL_joint_trajectory', 
#             lambda msg: self.gait_callback(msg, "RL"), 10)
#         self.create_subscription(
#             Float64MultiArray, 'RR_joint_trajectory', 
#             lambda msg: self.gait_callback(msg, "RR"), 10)
#         self.create_subscription(
#             Float64MultiArray, 'FR_joint_trajectory', 
#             lambda msg: self.gait_callback(msg, "FR"), 10)

#         # Publishers
#         self.joint_pub = self.create_publisher(JointState, 'mujoco_joint_states', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # PD Controller Parameters
#         self.kp = 400.0
#         self.kd = 20.0
#         self.sim_dt = self.model.opt.timestep
        
#         # Launch the viewer and start simulation timer
#         self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
#         self.create_timer(self.sim_dt, self.simulation_step)
        
#         self.get_logger().info(f"MuJoCo simulation initialized with model: {model_path}")
#         self.get_logger().info(f"Waiting for trajectories for all 4 legs...")

#     def gait_callback(self, msg, leg_name):
#         try:
#             # Reshape the flat array into (timesteps, 3 joints)
#             data = np.array(msg.data).reshape(-1, 3)
#             self.trajectory_buffer[leg_name] = data
#             self.get_logger().info(f"Received {data.shape[0]} timesteps for {leg_name}")
            
#             # Check if we've received data for all legs
#             if all(buffer is not None for buffer in self.trajectory_buffer.values()):
#                 if self.current_phase == self.PHASE_WAIT_FOR_DATA:
#                     self.current_phase = self.PHASE_EXECUTE_GAIT
#                     self.get_logger().info("All trajectories received. Starting gait execution.")
#         except Exception as e:
#             self.get_logger().error(f"Error in {leg_name} callback: {str(e)}")

#     def simulation_step(self):
#         if self.current_phase == self.PHASE_STABILIZE:
#             # Stabilization phase - hold the home pose
#             for joint_name, home_angle in self.home_positions.items():
#                 self.joint_positions[joint_name] = home_angle
            
#             self.step_count += 1
#             if self.step_count >= self.stabilization_steps:
#                 self.current_phase = self.PHASE_WAIT_FOR_DATA
#                 self.get_logger().info("Robot stabilized. Waiting for trajectory data...")
                
#         elif self.current_phase == self.PHASE_WAIT_FOR_DATA:
#             # Continue holding home position until we get data
#             for joint_name, home_angle in self.home_positions.items():
#                 self.joint_positions[joint_name] = home_angle
                
#             # Check if we have data for all legs
#             if all(buffer is not None for buffer in self.trajectory_buffer.values()):
#                 self.current_phase = self.PHASE_EXECUTE_GAIT
#                 self.get_logger().info("All trajectories received. Starting gait execution.")
                
#         elif self.current_phase == self.PHASE_EXECUTE_GAIT:
#             # Execute the gait trajectory
#             self.execute_gait()
        
#         # Apply PD control to all joints
#         self.apply_pd_control()
        
#         # Step the simulation
#         mujoco.mj_step(self.model, self.data)
#         self.viewer.sync()
        
#         # Publish joint states and TF
#         self.publish_joint_states()
#         self.publish_tf()

#     def execute_gait(self):
#         # Determine current trajectory point based on step count within the cycle
#         step_in_cycle = self.step_count % self.steps_per_cycle
#         trajectory_index = min(step_in_cycle // self.steps_per_trajectory_point, self.total_timesteps - 1)
        
#         # If we've completed a full cycle, increment the counter
#         if step_in_cycle == 0 and self.step_count > 0:
#             self.cycle_count += 1
#             self.get_logger().info(f"Completed gait cycle {self.cycle_count}")
        
#         # Apply the joint angles for the current trajectory point
#         for leg_name in ["FL", "RL", "FR", "RR"]:
#             # Get joint angles for current trajectory point
#             joint_angles = self.trajectory_buffer[leg_name][trajectory_index]
            
#             # Set joint positions for this leg
#             self.joint_positions[f"{leg_name}_hip_joint"] = joint_angles[0]
#             self.joint_positions[f"{leg_name}_thigh_joint"] = joint_angles[1]
#             self.joint_positions[f"{leg_name}_calf_joint"] = joint_angles[2]
        
#         self.step_count += 1

#     def apply_pd_control(self):
#         # Apply PD control to all actuated joints
#         for i in range(self.model.nu):
#             joint_id = self.model.actuator_trnid[i][0]
#             joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
#             q_index = self.model.jnt_qposadr[joint_id]
#             v_index = self.model.jnt_dofadr[joint_id]

#             target_angle = self.joint_positions.get(joint_name, 0.0)
#             current_angle = self.data.qpos[q_index]
#             current_vel = self.data.qvel[v_index]

#             error = target_angle - current_angle
#             torque = self.kp * error - self.kd * current_vel
#             self.data.ctrl[i] = 1.2*torque

#     def publish_joint_states(self):
#         joint_state_msg = JointState()
#         joint_state_msg.header.stamp = self.get_clock().now().to_msg()
#         joint_state_msg.name = self.joint_names
#         joint_state_msg.position = [self.data.qpos[self.model.jnt_qposadr[i]] for i in range(self.model.njnt)]
#         joint_state_msg.velocity = [self.data.qvel[self.model.jnt_dofadr[i]] for i in range(self.model.njnt)]
#         self.joint_pub.publish(joint_state_msg)

#     def publish_tf(self):
#         tf_msg = TransformStamped()
#         tf_msg.header.stamp = self.get_clock().now().to_msg()
#         tf_msg.header.frame_id = 'world'
#         tf_msg.child_frame_id = 'base'
#         tf_msg.transform.translation.x = self.data.qpos[0]
#         tf_msg.transform.translation.y = self.data.qpos[1]
#         tf_msg.transform.translation.z = self.data.qpos[2]
#         tf_msg.transform.rotation.w = self.data.qpos[3]
#         tf_msg.transform.rotation.x = self.data.qpos[4]
#         tf_msg.transform.rotation.y = self.data.qpos[5]
#         tf_msg.transform.rotation.z = self.data.qpos[6]
#         self.tf_broadcaster.sendTransform(tf_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = MujocoSimulator()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Simulation stopped by user')
#     finally:
#         node.viewer.close()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64MultiArray, String
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped
# import mujoco
# import mujoco.viewer
# import numpy as np
# import os
# from ament_index_python.packages import get_package_share_directory

# class MujocoSimulator(Node):
#     def __init__(self):
#         super().__init__('mujoco_simulator')
#         try:
#             package_share_dir = get_package_share_directory('b2_description')
#             model_path = os.path.join(package_share_dir, 'xml', 'scene.xml')
#         except Exception as e:
#             self.get_logger().error(f"Failed to locate package or XML: {e}")
#             return
        
#         self.model = mujoco.MjModel.from_xml_path(model_path)
#         self.data = mujoco.MjData(self.model)
        
#         # Initialize joint positions
#         self.joint_names = [
#             mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
#             for i in range(self.model.njnt)
#         ]
#         self.joint_positions = {name: 0.0 for name in self.joint_names}
        
#         # Define home position
#         self.home_positions = {
#             "FL_hip_joint": 0.0, "FL_thigh_joint": 0.7, "FL_calf_joint": -1.2,
#             "RL_hip_joint": 0.0, "RL_thigh_joint": 0.7, "RL_calf_joint": -1.2,
#             "FR_hip_joint": 0.0, "FR_thigh_joint": 0.7, "FR_calf_joint": -1.2,
#             "RR_hip_joint": 0.0, "RR_thigh_joint": 0.7, "RR_calf_joint": -1.2
#         }
        
#         # Store trajectories for each leg - initialized with empty arrays
#         self.trajectories = {
#             "forward": {
#                 "FL": None, "RL": None, "FR": None, "RR": None
#             },
#             "backward": {
#                 "FL": None, "RL": None, "FR": None, "RR": None
#             }
#         }
        
#         # Flag to know when we've received trajectories for all legs
#         self.has_complete_data = {
#             "forward": False,
#             "backward": False
#         }
        
#         # Current direction and point index
#         self.current_direction = "forward"
#         self.current_point_index = 0
        
#         # Initial robot position
#         self.data.qpos[0:3] = [0, 0, 0.65]
#         self.data.qvel[:] = 0.0
        
#         # Stabilization phase
#         self.stabilization_steps = 200
#         self.step_count = 0
#         self.is_stabilized = False
        
#         # Forward kinematics to update body and joint positions
#         mujoco.mj_forward(self.model, self.data)
        
#         # Subscribers for forward gait trajectories
#         self.create_subscription(
#             Float64MultiArray, 'FL_joint_trajectory_forward', 
#             lambda msg: self.trajectory_callback(msg, "FL", "forward"), 10)
#         self.create_subscription(
#             Float64MultiArray, 'RL_joint_trajectory_forward', 
#             lambda msg: self.trajectory_callback(msg, "RL", "forward"), 10)
#         self.create_subscription(
#             Float64MultiArray, 'FR_joint_trajectory_forward', 
#             lambda msg: self.trajectory_callback(msg, "FR", "forward"), 10)
#         self.create_subscription(
#             Float64MultiArray, 'RR_joint_trajectory_forward', 
#             lambda msg: self.trajectory_callback(msg, "RR", "forward"), 10)
        
#         # Subscribers for backward gait trajectories
#         self.create_subscription(
#             Float64MultiArray, 'FL_joint_trajectory_backward', 
#             lambda msg: self.trajectory_callback(msg, "FL", "backward"), 10)
#         self.create_subscription(
#             Float64MultiArray, 'RL_joint_trajectory_backward', 
#             lambda msg: self.trajectory_callback(msg, "RL", "backward"), 10)
#         self.create_subscription(
#             Float64MultiArray, 'FR_joint_trajectory_backward', 
#             lambda msg: self.trajectory_callback(msg, "FR", "backward"), 10)
#         self.create_subscription(
#             Float64MultiArray, 'RR_joint_trajectory_backward', 
#             lambda msg: self.trajectory_callback(msg, "RR", "backward"), 10)
        
#         # Subscriber for direction command
#         self.create_subscription(String, 'gait_direction', self.direction_callback, 10)

#         # Publishers
#         self.joint_pub = self.create_publisher(JointState, 'mujoco_joint_states', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Simple position control parameters
#         self.kp = 300.0 
#         self.kd = 40.0 # Position gain
        
#         # Start simulation timer
#         self.sim_dt = self.model.opt.timestep
#         self.create_timer(self.sim_dt, self.simulation_step)
        
#         # Launch viewer
#         self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
#         self.get_logger().info(f"MuJoCo simulation initialized with model: {model_path}")

#     def trajectory_callback(self, msg, leg_name, direction):
#         try:
#             # Reshape the flat array into (timesteps, 3 joints)
#             data = np.array(msg.data).reshape(-1, 3)
#             self.trajectories[direction][leg_name] = data
#             self.get_logger().info(f"Received {data.shape[0]} timesteps for {leg_name} ({direction})")
            
#             # Check if we have data for all legs in this direction
#             if all(self.trajectories[direction][leg] is not None for leg in ["FL", "FR", "RL", "RR"]):
#                 self.has_complete_data[direction] = True
#                 self.get_logger().info(f"Complete {direction} trajectory data received")
#         except Exception as e:
#             self.get_logger().error(f"Error in {leg_name} {direction} callback: {str(e)}")

#     def direction_callback(self, msg):
#         direction = msg.data.lower()
#         if direction in ["forward", "backward"]:
#             # Only change direction if we have data for that direction
#             if self.has_complete_data[direction]:
#                 if self.current_direction != direction:
#                     self.current_direction = direction
#                     self.current_point_index = 0
#                     self.get_logger().info(f"Switching to {direction} movement")
#             else:
#                 self.get_logger().warning(f"Cannot switch to {direction} - no complete trajectory data")

#     def simulation_step(self):
#         # Handle stabilization phase
#         if not self.is_stabilized:
#             if self.step_count < self.stabilization_steps:
#                 # Hold home pose during stabilization
#                 for joint_name, home_angle in self.home_positions.items():
#                     self.joint_positions[joint_name] = home_angle
#                 self.step_count += 1
#                 if self.step_count % 50 == 0:
#                     self.get_logger().info(f"Stabilizing... {self.step_count}/{self.stabilization_steps}")
#             else:
#                 self.is_stabilized = True
#                 self.get_logger().info("Stabilization complete")
#         else:
#             # Execute gait after stabilization if we have data
#             if self.has_complete_data[self.current_direction]:
#                 self.execute_gait()
#             else:
#                 # Hold home position until we get data
#                 for joint_name, home_angle in self.home_positions.items():
#                     self.joint_positions[joint_name] = home_angle
        
#         # Apply simple position control
#         self.apply_position_control()
        
#         # Step the simulation
#         mujoco.mj_step(self.model, self.data)
#         self.viewer.sync()
        
#         # Publish joint states and TF
#         self.publish_joint_states()
#         self.publish_tf()

#     def execute_gait(self):
#         # Get trajectory data for current direction
#         trajectory_data = self.trajectories[self.current_direction]
        
#         # Get number of points in trajectory (use FL leg as reference)
#         num_points = len(trajectory_data["FL"])
        
#         if num_points == 0:
#             self.get_logger().error(f"No trajectory points for {self.current_direction}")
#             return
        
#         # For each leg, set the target joint positions from the current point
#         for leg_name in ["FL", "RL", "FR", "RR"]:
#             leg_trajectory = trajectory_data[leg_name]
            
#             # Get joint angles for current trajectory point
#             if self.current_point_index < len(leg_trajectory):
#                 joint_angles = leg_trajectory[self.current_point_index]
                
#                 # Set joint positions for this leg
#                 self.joint_positions[f"{leg_name}_hip_joint"] = joint_angles[0]
#                 self.joint_positions[f"{leg_name}_thigh_joint"] = joint_angles[1]
#                 self.joint_positions[f"{leg_name}_calf_joint"] = joint_angles[2]
        
#         # Move to next point in trajectory
#         self.current_point_index += 1
#         if self.current_point_index >= num_points:
#             self.current_point_index = 0
#             self.get_logger().info(f"Completed {self.current_direction} gait cycle")

#     def apply_position_control(self):
#         # Simple position control for all actuated joints
#         for i in range(self.model.nu):
#             joint_id = self.model.actuator_trnid[i][0]
#             joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            
#             if joint_name in self.joint_positions:
#                 joint_id = self.model.actuator_trnid[i][0]
#                 joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
#                 q_index = self.model.jnt_qposadr[joint_id]
#                 v_index = self.model.jnt_dofadr[joint_id]

#                 target_angle = self.joint_positions.get(joint_name, 0.0)
#                 current_angle = self.data.qpos[q_index]
#                 current_vel = self.data.qvel[v_index]

#                 error = target_angle - current_angle
#                 torque = self.kp * error - self.kd * current_vel
#                 self.data.ctrl[i] = torque

#     def publish_joint_states(self):
#         joint_state_msg = JointState()
#         joint_state_msg.header.stamp = self.get_clock().now().to_msg()
#         joint_state_msg.name = self.joint_names
#         joint_state_msg.position = [self.data.qpos[self.model.jnt_qposadr[i]] for i in range(self.model.njnt)]
#         joint_state_msg.velocity = [self.data.qvel[self.model.jnt_dofadr[i]] for i in range(self.model.njnt)]
#         self.joint_pub.publish(joint_state_msg)

#     def publish_tf(self):
#         tf_msg = TransformStamped()
#         tf_msg.header.stamp = self.get_clock().now().to_msg()
#         tf_msg.header.frame_id = 'world'
#         tf_msg.child_frame_id = 'base'
#         tf_msg.transform.translation.x = self.data.qpos[0]
#         tf_msg.transform.translation.y = self.data.qpos[1]
#         tf_msg.transform.translation.z = self.data.qpos[2]
#         tf_msg.transform.rotation.w = self.data.qpos[3]
#         tf_msg.transform.rotation.x = self.data.qpos[4]
#         tf_msg.transform.rotation.y = self.data.qpos[5]
#         tf_msg.transform.rotation.z = self.data.qpos[6]
#         self.tf_broadcaster.sendTransform(tf_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = MujocoSimulator()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Simulation stopped by user')
#     finally:
#         node.viewer.close()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import mujoco
import mujoco.viewer
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class MujocoSimulator(Node):
    def __init__(self):
        super().__init__('mujoco_simulator')
        try:
            package_share_dir = get_package_share_directory('b2_description')
            model_path = os.path.join(package_share_dir, 'xml', 'scene.xml')
        except Exception as e:
            self.get_logger().error(f"Failed to locate package or XML: {e}")
            return
        
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # Initialize joint positions
        self.joint_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt)
        ]
        self.joint_positions = {name: 0.0 for name in self.joint_names}
        
        # Define home position
        self.home_positions = {
            "FL_hip_joint": 0.0, "FL_thigh_joint": 0.7, "FL_calf_joint": -1.2,
            "RL_hip_joint": 0.0, "RL_thigh_joint": 0.7, "RL_calf_joint": -1.2,
            "FR_hip_joint": 0.0, "FR_thigh_joint": 0.7, "FR_calf_joint": -1.2,
            "RR_hip_joint": 0.0, "RR_thigh_joint": 0.7, "RR_calf_joint": -1.2
        }
        
        # Store trajectories for each leg - initialized with empty arrays
        self.trajectories = {
            "forward": {
                "FL": None, "RL": None, "FR": None, "RR": None
            },
            "backward": {
                "FL": None, "RL": None, "FR": None, "RR": None
            }
        }
        
        # Flag to know when we've received trajectories for all legs
        self.has_complete_data = {
            "forward": False,
            "backward": False
        }
        
        # Current direction and point index
        self.current_direction = "forward"
        self.current_point_index = 0
        
        # Error control parameters
        self.error_threshold = 0.05 # Radians
        self.min_stable_time = 1.0  # Seconds to wait at each point
        self.stable_counter = 0
        self.steps_to_wait = int(self.min_stable_time / self.model.opt.timestep)
        self.target_reached = False
        
        # Initial robot position
        self.data.qpos[0:3] = [0, 0, 0.65]
        self.data.qvel[:] = 0.0
        
        # Stabilization phase
        self.stabilization_steps = 200
        self.step_count = 0
        self.is_stabilized = False
        
        # Forward kinematics to update body and joint positions
        mujoco.mj_forward(self.model, self.data)
        
        # Subscribers for forward gait trajectories
        self.create_subscription(
            Float64MultiArray, 'FL_joint_trajectory_forward', 
            lambda msg: self.trajectory_callback(msg, "FL", "forward"), 10)
        self.create_subscription(
            Float64MultiArray, 'RL_joint_trajectory_forward', 
            lambda msg: self.trajectory_callback(msg, "RL", "forward"), 10)
        self.create_subscription(
            Float64MultiArray, 'FR_joint_trajectory_forward', 
            lambda msg: self.trajectory_callback(msg, "FR", "forward"), 10)
        self.create_subscription(
            Float64MultiArray, 'RR_joint_trajectory_forward', 
            lambda msg: self.trajectory_callback(msg, "RR", "forward"), 10)
        
        # Subscribers for backward gait trajectories
        self.create_subscription(
            Float64MultiArray, 'FL_joint_trajectory_backward', 
            lambda msg: self.trajectory_callback(msg, "FL", "backward"), 10)
        self.create_subscription(
            Float64MultiArray, 'RL_joint_trajectory_backward', 
            lambda msg: self.trajectory_callback(msg, "RL", "backward"), 10)
        self.create_subscription(
            Float64MultiArray, 'FR_joint_trajectory_backward', 
            lambda msg: self.trajectory_callback(msg, "FR", "backward"), 10)
        self.create_subscription(
            Float64MultiArray, 'RR_joint_trajectory_backward', 
            lambda msg: self.trajectory_callback(msg, "RR", "backward"), 10)
        
        # Subscriber for direction command
        self.create_subscription(String, 'gait_direction', self.direction_callback, 10)

        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'mujoco_joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Simple position control parameters
        self.kp = 300.0
        self.kd = 30.0  # Position gain
        
        # Start simulation timer
        self.sim_dt = self.model.opt.timestep
        self.create_timer(self.sim_dt, self.simulation_step)
        
        # Launch viewer
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
        self.get_logger().info(f"MuJoCo simulation initialized with model: {model_path}")
        self.get_logger().info(f"Error threshold: {self.error_threshold}, Wait time per point: {self.min_stable_time}s")

    def trajectory_callback(self, msg, leg_name, direction):
        try:
            # Reshape the flat array into (timesteps, 3 joints)
            data = np.array(msg.data).reshape(-1, 3)
            self.trajectories[direction][leg_name] = data
            self.get_logger().info(f"Received {data.shape[0]} timesteps for {leg_name} ({direction})")
            
            # Check if we have data for all legs in this direction
            if all(self.trajectories[direction][leg] is not None for leg in ["FL", "FR", "RL", "RR"]):
                self.has_complete_data[direction] = True
                self.get_logger().info(f"Complete {direction} trajectory data received")
                
                # Reset point index when receiving new data if already running
                if self.is_stabilized and self.current_direction == direction:
                    self.current_point_index = 0
                    self.target_reached = False
                    self.stable_counter = 0
        except Exception as e:
            self.get_logger().error(f"Error in {leg_name} {direction} callback: {str(e)}")

    def direction_callback(self, msg):
        direction = msg.data.lower()
        if direction in ["forward", "backward"]:
            # Only change direction if we have data for that direction
            if self.has_complete_data[direction]:
                if self.current_direction != direction:
                    self.current_direction = direction
                    self.current_point_index = 0
                    self.target_reached = False
                    self.stable_counter = 0
                    self.get_logger().info(f"Switching to {direction} movement")
            else:
                self.get_logger().warning(f"Cannot switch to {direction} - no complete trajectory data")

    def simulation_step(self):
        # Handle stabilization phase
        if not self.is_stabilized:
            if self.step_count < self.stabilization_steps:
                # Hold home pose during stabilization
                for joint_name, home_angle in self.home_positions.items():
                    self.joint_positions[joint_name] = home_angle
                self.step_count += 1
                if self.step_count % 50 == 0:
                    self.get_logger().info(f"Stabilizing... {self.step_count}/{self.stabilization_steps}")
            else:
                self.is_stabilized = True
                self.get_logger().info("Stabilization complete")
        else:
            # Execute gait after stabilization if we have data
            if self.has_complete_data[self.current_direction]:
                self.execute_gait_with_error_control()
            else:
                # Hold home position until we get data
                for joint_name, home_angle in self.home_positions.items():
                    self.joint_positions[joint_name] = home_angle
        
        # Apply position control
        self.apply_position_control()
        
        # Step the simulation
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()
        
        # Publish joint states and TF
        self.publish_joint_states()
        self.publish_tf()

    def execute_gait_with_error_control(self):
        # Get trajectory data for current direction
        trajectory_data = self.trajectories[self.current_direction]
        
        # Get number of points in trajectory (use FL leg as reference)
        num_points = len(trajectory_data["FL"])
        
        if num_points == 0:
            self.get_logger().error(f"No trajectory points for {self.current_direction}")
            return
        
        # Check if we need to move to the next point
        if self.should_move_to_next_point():
            # Move to next point
            self.current_point_index = (self.current_point_index + 1) % num_points
            self.target_reached = False
            self.stable_counter = 0
            self.get_logger().info(f"Moving to point {self.current_point_index}/{num_points}")
            
            if self.current_point_index == 0:
                self.get_logger().info(f"Completed {self.current_direction} gait cycle")
        
        # For each leg, set the target joint positions from the current point
        for leg_name in ["FL", "RL", "FR", "RR"]:
            leg_trajectory = trajectory_data[leg_name]
            
            # Make sure we have valid data
            if leg_trajectory is not None and self.current_point_index < len(leg_trajectory):
                joint_angles = leg_trajectory[self.current_point_index]
                
                # Set joint positions for this leg
                self.joint_positions[f"{leg_name}_hip_joint"] = joint_angles[0]
                self.joint_positions[f"{leg_name}_thigh_joint"] = joint_angles[1]
                self.joint_positions[f"{leg_name}_calf_joint"] = joint_angles[2]

    def should_move_to_next_point(self):
        # If we already reached the target position, count stable time
        if self.target_reached:
            self.stable_counter += 1
            return self.stable_counter >= self.steps_to_wait
        
        # Check if we've reached the target position (error below threshold)
        max_error = self.calculate_max_joint_error()
        
        if max_error < self.error_threshold:
            self.target_reached = True
            self.get_logger().info(f"Point {self.current_point_index} reached with max error {max_error:.4f}")
            return False
        
        return False
    
    def calculate_max_joint_error(self):
        max_error = 0.0
        
        # Check error for all leg joints
        for leg_name in ["FL", "RL", "FR", "RR"]:
            for joint_type in ["hip", "thigh", "calf"]:
                joint_name = f"{leg_name}_{joint_type}_joint"
                
                # Get joint ID and position
                for i in range(self.model.njnt):
                    if mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) == joint_name:
                        q_index = self.model.jnt_qposadr[i]
                        target_angle = self.joint_positions.get(joint_name, 0.0)
                        current_angle = self.data.qpos[q_index]
                        
                        error = abs(target_angle - current_angle)
                        max_error = max(max_error, error)
        
        return max_error

    def apply_position_control(self):
        # Simple position control for all actuated joints
        for i in range(self.model.nu):
            joint_id = self.model.actuator_trnid[i][0]
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            
            if joint_name in self.joint_positions:
                # joint_id = self.model.actuator_trnid[i][0]
                # joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
                q_index = self.model.jnt_qposadr[joint_id]
                v_index = self.model.jnt_dofadr[joint_id]

                target_angle = self.joint_positions.get(joint_name, 0.0)
                current_angle = self.data.qpos[q_index]
                current_vel = self.data.qvel[v_index]

                error = target_angle - current_angle
                torque = self.kp * error - self.kd * current_vel
                self.data.ctrl[i] = torque

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = [self.data.qpos[self.model.jnt_qposadr[i]] for i in range(self.model.njnt)]
        joint_state_msg.velocity = [self.data.qvel[self.model.jnt_dofadr[i]] for i in range(self.model.njnt)]
        self.joint_pub.publish(joint_state_msg)

    def publish_tf(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'base'
        tf_msg.transform.translation.x = self.data.qpos[0]
        tf_msg.transform.translation.y = self.data.qpos[1]
        tf_msg.transform.translation.z = self.data.qpos[2]
        tf_msg.transform.rotation.w = self.data.qpos[3]
        tf_msg.transform.rotation.x = self.data.qpos[4]
        tf_msg.transform.rotation.y = self.data.qpos[5]
        tf_msg.transform.rotation.z = self.data.qpos[6]
        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Simulation stopped by user')
    finally:
        node.viewer.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()