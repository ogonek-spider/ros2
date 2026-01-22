from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# we need this whole file to pass urdf content to inverse kinematics

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("spider_ros_control"), "urdf", "spider.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    ik_walker = Node(
        package="spider_walker",
        executable="ik",
        parameters=[
            robot_description
        ],
        output="both",
    )

    nodes = [
        ik_walker
    ]

    return LaunchDescription(nodes)