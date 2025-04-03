#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import mujoco
import tf_transformations
import mujoco.viewer
import numpy as np
import os
from tf_transformations import quaternion_multiply
import threading

from ament_index_python.packages import get_package_share_directory


class MujocoSimulator(Node):
    def __init__(self):
        super().__init__('mujoco_simulator')

        try:
            package_share_dir = get_package_share_directory('b2_description')
            model_path = os.path.join(package_share_dir, 'xml', 'scene.xml')
        except Exception as e:
            self.get_logger().error(f"Failed to locate package or XM-L: {e}")
            return
        
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Set joint angles for each leg
        self.target_joint_angles = {
            "FL_hip_joint": 0.0,
            "FL_thigh_joint": 0.7,
            "FL_calf_joint": -1.2,
            "FR_hip_joint": 0.0,
            "FR_thigh_joint": 0.7,
            "FR_calf_joint": -1.2,
            "RL_hip_joint": 0.0,
            "RL_thigh_joint": 0.7,
            "RL_calf_joint": -1.2,
            "RR_hip_joint": 0.0,
            "RR_thigh_joint": 0.7,
            "RR_calf_joint": -1.2,
        }

        # Set initial joint configuration
        for name, angle in self.target_joint_angles.items():
            try:
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)

                q_index = self.model.jnt_qposadr[joint_id]
                self.data.qpos[q_index] = angle
            except Exception as e:
                self.get_logger().warn(f"Joint '{name}' not found: {e}")

        self.data.qpos[0:3] = [0, 0,0.65]
        self.data.qvel[:] = 0.0
        # Forward kinematics to update body and joint positions
        mujoco.mj_forward(self.model, self.data)

        # Launch viewer in a background thread
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # ROS publishers
        self.joint_pub = self.create_publisher(JointState, 'mujoco_joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Joint info
        self.joint_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt)
        ]
        self.joint_indices = [
            self.model.jnt_qposadr[i]
            for i in range(self.model.njnt)
        ]

        self.kp = 300.0
        self.kd = 20.0
        self.sim_dt = self.model.opt.timestep
        self.create_timer(self.sim_dt, self.simulation_step)

        self.get_logger().info(f"MuJoCo simulation initialized with model: {model_path}")

    def simulation_step(self):

        # for i in range(self.model.nu):  # Number of actuators
            
        #     joint_id = self.model.actuator_trnid[i][0]  # First element gives joint ID
        #     joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        #     q_index = self.model.jnt_qposadr[joint_id]
        #     v_index = self.model.jnt_dofadr[joint_id]

        #     target_angle = self.target_joint_angles.get(joint_name, 0.0)
        #     current_angle = self.data.qpos[q_index]
        #     current_vel = self.data.qvel[v_index]

        #     error = target_angle - current_angle
        #     torque = self.kp * error - self.kd * current_vel
        #     self.data.ctrl[i] = torque
        # # Step simulation
        mujoco.mj_step(self.model, self.data)

        # Update viewer
        self.viewer.sync()

        # Publish joint states
        joint_state_msg = JointState()
        now = self.get_clock().now().to_msg()
        joint_state_msg.header.stamp = now
        joint_state_msg.header.frame_id = 'base'
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = [self.data.qpos[self.model.jnt_qposadr[i]] for i in range(self.model.njnt)]
        joint_state_msg.velocity = [self.data.qvel[self.model.jnt_dofadr[i]] for i in range(self.model.njnt)]
        self.joint_pub.publish(joint_state_msg)

        # Publish TF from world to base
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'base'

        tf_msg.transform.translation.x = self.data.qpos[0]
        tf_msg.transform.translation.y = self.data.qpos[1]
        tf_msg.transform.translation.z = self.data.qpos[2]

     
        raw_quat = [self.data.qpos[3], self.data.qpos[4], self.data.qpos[5], self.data.qpos[6]]
        corrected_quat = raw_quat

        tf_msg.transform.rotation.w = corrected_quat[0]
        tf_msg.transform.rotation.x = corrected_quat[1]
        tf_msg.transform.rotation.y = corrected_quat[2]
        tf_msg.transform.rotation.z = corrected_quat[3]
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
