from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare Xacro file path

    urdf_file_arg = DeclareLaunchArgument(
        name='urdf_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('b2_description'),
            'xacro',
            'robot.xacro'
        ]),
        description='Absolute path to Xacro file'
    )

    robot_description = Command(['xacro ', LaunchConfiguration('urdf_file')])


    pkg_share = get_package_share_directory('b2_description')
    urdf_file_path = os.path.join(pkg_share, 'xml', 'b2_description.urdf')
    # Read URDF
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('b2_description'),
        'rviz',
        'mujoco_rviz_combined.rviz'
    ])



    return LaunchDescription([
        urdf_file_arg,

        # Launch MuJoCo simulator node (uncomment when simulate.py is installed correctly)
        Node(
            package='b2_description',
            executable='mujoco_simulation.py',
            name='mujoco_simulation',
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }],
            remappings=[
                ('/joint_states', '/mujoco_joint_states')
            ]
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            remappings=[
                ('/joint_states', '/mujoco_joint_states'),
                ('/tf', '/tf')
            ]
        )
    ])



