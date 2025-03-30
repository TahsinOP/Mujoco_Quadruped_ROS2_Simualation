from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('b2_description')
    urdf_file_path = os.path.join(pkg_share, 'xml', 'b2_description.urdf')
    # Read URDF
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

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

    return LaunchDescription([
        urdf_file_arg,

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
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
            }]
        ),

        # RViz
        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            get_package_share_directory('b2_description'),
            'rviz',
            'interactive_display_quadruped.rviz'
            ])]
        )

    ])
