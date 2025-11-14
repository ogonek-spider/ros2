import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory('spider-description'),  # Change to your package
        'robot.urdf')
    
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # If you are using Xacro, you can use this instead:
    # robot_desc = Command(['xacro ', urdf_path])

    rviz_config_file = os.path.join(get_package_share_directory("spider-description"), "spider.rviz")

    return LaunchDescription([
        # Node to publish the robot's state (positions of all joints and links)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        # Node to publish the state of non-fixed joints (opens a GUI for control)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # Node to start RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=["-d", rviz_config_file],
        ),
    ])