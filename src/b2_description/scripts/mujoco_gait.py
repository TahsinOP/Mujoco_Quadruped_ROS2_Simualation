#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
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
        
        # Trajectory buffer for each leg
        self.trajectory_buffer = {"FL": np.zeros((100, 3)), "RL": np.zeros((100, 3)), 
                                  "FR": np.zeros((100, 3)), "RR": np.zeros((100, 3))}
        self.current_timestep = 0
        
        self.stabilization_steps = 200
        self.step_count = 0
        self.gait_completed = False

        self.data.qpos[0:3] = [0,0,0.65]
        self.data.qvel[:] = 0.0
        # Forward kinematics to update body and joint positions
        mujoco.mj_forward(self.model, self.data)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        # Subscribers for each leg
        self.create_subscription(Float64MultiArray, 'LF_joint_trajectory', lambda msg: self.gait_callback(msg, "FL"), 10)
        self.create_subscription(Float64MultiArray, 'LB_joint_trajectory', lambda msg: self.gait_callback(msg, "RL"), 10)
        self.create_subscription(Float64MultiArray, 'RB_joint_trajectory', lambda msg: self.gait_callback(msg, "RR"), 10)
        self.create_subscription(Float64MultiArray, 'RF_joint_trajectory', lambda msg: self.gait_callback(msg, "FR"), 10)

        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'mujoco_joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # PD Controller Parameters
        self.kp = 300.0
        self.kd = 20.0
        self.sim_dt = self.model.opt.timestep
        self.create_timer(self.sim_dt, self.simulation_step)

        # # MuJoCo Viewer
        # self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.get_logger().info(f"MuJoCo simulation initialized with model: {model_path}")

    def gait_callback(self, msg, leg_name):
        try:
            data = np.array(msg.data).reshape(-1, 3)
            self.trajectory_buffer[leg_name] = data
            self.get_logger().info(f"Received {data.shape[0]} timesteps for {leg_name}")

            # for i, (hip, thigh, knee) in enumerate(data[:70]):
            #     self.get_logger().info(f"{leg_name} Step {i}: Hip={hip:.3f}, Thigh={thigh:.3f}, Knee={knee:.3f}")
        except Exception as e:
            self.get_logger().error(f"Error in {leg_name} callback: {str(e)}")

    def simulation_step(self):
        print("started simulation")
        if self.step_count < self.stabilization_steps :
            # Hold home pose before starting gait
            for joint_name, home_angle in self.home_positions.items():
                self.joint_positions[joint_name] = home_angle
                print("Setting home pose")
            self.step_count += 1
        else:
            # Execute gait after stabilization
            # self.execute_gait()
            print(self.step_count)
            print("Done setting up home pose")
            self.execute_gait()
        
        # Apply PD control
        for i in range(self.model.nu):
            joint_id = self.model.actuator_trnid[i][0]
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            q_index = self.model.jnt_qposadr[joint_id]
            v_index = self.model.jnt_dofadr[joint_id]

            target_angle = self.joint_positions.get(joint_name, 0.0)
            current_angle = self.data.qpos[q_index]
            current_vel = self.data.qvel[v_index]

            error = target_angle - current_angle
            torque = self.kp * error - self.kd * current_vel
            self.data.ctrl[i] = torque

        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()
        self.publish_joint_states()
        self.publish_tf()

    def execute_gait(self):
        if self.current_timestep >= 100:
            self.current_timestep = 0
            self.gait_completed = True
        
        # for leg_name, joint_order in zip(["FL", "RL", "FR", "RR"], [["hip", "thigh", "calf"]] * 4):
        for leg_name, joint_order in zip(["FL", "RL" , "FR" , "RR" ], [["hip", "thigh", "calf"]] * 4):
            print(self.current_timestep)
            joint_angles = self.trajectory_buffer[leg_name][self.current_timestep]
            print(joint_angles)
            for j, joint in enumerate(joint_order):
                joint_name = f"{leg_name}_{joint}_joint"
                self.joint_positions[joint_name] = joint_angles[j]
        
        self.current_timestep += 1

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
