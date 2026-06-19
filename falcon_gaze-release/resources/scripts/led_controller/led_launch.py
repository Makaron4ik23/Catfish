#!/usr/bin/env python3
import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument(
        'id',
        default_value='0',
        description='Drone ID to target inside the swarm simulation grid (0, 1, 2, 3...)'
    )
    
    file_arg = DeclareLaunchArgument(
        'file',
        default_value='sequences.json',
        description='The name of the target JSON sequence file'
    )

    point_arg = DeclareLaunchArgument(
        'point',
        default_value='first_point',
        description='The target mission block key name inside the JSON structure'
    )
    
    target_id = LaunchConfiguration('id')
    target_file = LaunchConfiguration('file')
    target_point = LaunchConfiguration('point')

    # Unidirectional transport parameter bridge mapping
    bridge_cmd = [
        "ros2 run ros_gz_bridge parameter_bridge ",
        "'/model/x500_mono_cam_", target_id, "/led_cmd@std_msgs/msg/String]gz.msgs.StringMsg'"
    ]
    
    ros_gz_bridge = ExecuteProcess(
        cmd=[bridge_cmd],
        shell=True,
        output='screen'
    )

    # Launch the orchestrator passing ID, filename, and target mission point profile
    orchestrator = ExecuteProcess(
        cmd=["python3", "led_controll.py", target_id, target_file, target_point],
        output='screen'
    )

    return LaunchDescription([
        drone_id_arg,
        file_arg,
        point_arg,
        ros_gz_bridge,
        orchestrator
    ])