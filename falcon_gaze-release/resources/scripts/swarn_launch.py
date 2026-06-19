from launch import LaunchDescription    
from launch.actions import ExecuteProcess, TimerAction

import os
import math

WORLD_NAME = os.environ.get("WORLD_NAME", "baylands_custom")

# Heading: 0 rad = North (most stable for EKF magnetometer convergence)
# Drones will rotate to desired heading after takeoff via mission code
HEADING_RAD = 0.0
# Direction from leader to behind = heading + pi (south)
BEHIND_DX = -math.cos(HEADING_RAD)  # -1.0 (south along X)
BEHIND_DY = -math.sin(HEADING_RAD)  # 0.0

def leader_instanse(x, y, z, yaw=HEADING_RAD):
        cmd = f"""
            cd ~/PX4-Autopilot/ &&
            PX4_SYS_AUTOSTART=4010 \
            PX4_SIM_MODEL=gz_x500_mono_cam \
            PX4_GZ_WORLD={WORLD_NAME} \
            PX4_GZ_MODEL_POSE="{x},{y},{z},0,0,{yaw}" \
            ./build/px4_sitl_default/bin/px4 -i 0
            """
        return ExecuteProcess(
                cmd=["bash", "-c", cmd],
                output="screen"
        )

def follower_instanse(i, x, y, z, yaw=HEADING_RAD):
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

# Leader spawn point
LEADER_X, LEADER_Y = 125.0, 51.0  # moved downhill from purple elevation
SPAWN_Z = 1.0       # ground level - flat terrain here
SPACING = 2.0        # meters between drones (tight to stay on flat ground)

def generate_launch_description():
        actions = []
        # Leader (Starts Gazebo)
        actions.append(
                leader_instanse(LEADER_X, LEADER_Y, SPAWN_Z)
        )
        # Followers in single-file chain behind leader (2.5m spacing)
        followers = []
        for k in range(1, 4):
                fx = LEADER_X + BEHIND_DX * SPACING * k
                fy = LEADER_Y + BEHIND_DY * SPACING * k
                followers.append((k, round(fx, 3), round(fy, 3), SPAWN_Z))

        # Delay followers so Gazebo is ready (staggered 20s apart)
        for i, (idx, x, y, z) in enumerate(followers):
                actions.append(
                        TimerAction(
                                period=20.0 + i * 10.0,
                                actions=[
                                        follower_instanse(
                                                idx, x, y, z
                                                )
                                        ]
                        )
                )
        return LaunchDescription(actions)