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
        self.joint_positions = {}

        # Initialize joint state dictionary
        self.joint_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt)
        ]
        for name in self.joint_names:
            self.joint_positions[name] = 0.0
        
        # Separate Subscribers for Each Leg
        self.create_subscription(Float64MultiArray, 'LF_joint_trajectory', lambda msg: self.gait_callback(msg, "FL"), 10)
        self.create_subscription(Float64MultiArray, 'LB_joint_trajectory', lambda msg: self.gait_callback(msg, "RL"), 10)
        self.create_subscription(Float64MultiArray, 'RB_joint_trajectory', lambda msg: self.gait_callback(msg, "RR"), 10)
        self.create_subscription(Float64MultiArray, 'RF_joint_trajectory', lambda msg: self.gait_callback(msg, "FR"), 10)

        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'mujoco_joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.kp = 300.0
        self.kd = 20.0
        self.sim_dt = self.model.opt.timestep
        # self.create_timer(self.sim_dt, self.simulation_step)

        # self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.get_logger().info(f"MuJoCo simulation initialized with model: {model_path}")

    def gait_callback(self, msg, leg_name):
        if len(msg.data) != 3:
            self.get_logger().warn(f"Received incorrect trajectory data length for {leg_name}")
            return

        joint_order = ['hip', 'thigh', 'calf']
        for j in range(3):
            joint_name = f"{leg_name}_{joint_order[j]}_joint"
            self.joint_positions[joint_name] = msg.data[j]

    def simulation_step(self):
        for i in range(self.model.nu):  # Number of actuators
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

    def publish_joint_states(self):
        joint_state_msg = JointState()
        now = self.get_clock().now().to_msg()
        joint_state_msg.header.stamp = now
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = [self.data.qpos[self.model.jnt_qposadr[i]] for i in range(self.model.njnt)]
        joint_state_msg.velocity = [self.data.qvel[self.model.jnt_dofadr[i]] for i in range(self.model.njnt)]
        self.joint_pub.publish(joint_state_msg)

    def publish_tf(self):
        tf_msg = TransformStamped()
        now = self.get_clock().now().to_msg()
        tf_msg.header.stamp = now
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
