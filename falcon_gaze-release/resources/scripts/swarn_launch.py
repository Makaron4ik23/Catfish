from launch import LaunchDescription    
from launch.actions import ExecuteProcess, TimerAction

import os

WORLD_NAME = os.environ.get("WORLD_NAME", "baylands_custom")

def leader_instanse(x, y, z, yaw=3.7346):
        cmd = f"""
            cd ~/PX4-Autopilot/ &&
            PX4_SYS_AUTOSTART=4010 \
            PX4_SIM_MODEL=gz_x500_mono_cam \
            PX4_GZ_WORLD={WORLD_NAME} \
            PX4_GZ_MODEL_POSE="{x},{y},{z},0,0,3.7346" \
            ./build/px4_sitl_default/bin/px4 -i 0
            """
        return ExecuteProcess(
                cmd=["bash", "-c", cmd],
                output="screen"
        )

def follower_instanse(i, x, y, z, yaw=3.7346):
        cmd = f"""
            cd ~/PX4-Autopilot/ &&
            PX4_SYS_AUTOSTART=4010 \
            PX4_SIM_MODEL=gz_x500_mono_cam \
            PX4_GZ_WORLD={WORLD_NAME} \
            PX4_GZ_MODEL_POSE="{x},{y},{z},0,0,{yaw}" \
            ./build/px4_sitl_default/bin/px4 -i {i}
            """

        return ExecuteProcess(
                cmd=["bash", "-c", cmd],
                output="screen"
        )

def generate_launch_description():
        actions = []
        # Leader (Starts Gazebo)
        actions.append(
                leader_instanse(
                        127.0, 
                        52.67, 
                        1.4
                    )
        )
        # Followers
        followers = [
                (1, 129.92, 52.852, 1.4),
                (2, 129.08, 54.095, 1.4),
                (3, 128.24, 55.339, 1.4)
        ]

        #Delay followers so Gazebo is ready (staggered delay to prevent EKF/physics spikes on laptops)
        for n, (idx, x, y, z) in enumerate(followers):
                actions.append(
                        TimerAction(
                                period=20.0 + n * 10.0,  # generous delay for slow SITL
                                actions=[
                                        follower_instanse(
                                                idx, x, y, z
                                                )
                                        ]
                        )
                )
        return LaunchDescription(actions)