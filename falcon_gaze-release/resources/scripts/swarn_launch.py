from launch import LaunchDescription    
from launch.actions import ExecuteProcess, TimerAction

import os
WORLD_NAME = os.environ.get("WORLD_NAME", "baylands_custom")
SETUP_SCRIPT = os.path.join(os.path.dirname(__file__), "px4_gz_setup.sh")

# Heading 0 rad matches PX4 North. In this Baylands world that points along +Y.
HEADING_RAD = 0.0

def leader_instanse(x, y, z, yaw=HEADING_RAD):
        cmd = f"""
            source "{SETUP_SCRIPT}" &&
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
            source "{SETUP_SCRIPT}" &&
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

# Start platform bbox from baylands_start.glb:
# center=(128.2, 53.5, 1.4), local x/y extent +/-3m.
PLATFORM_CENTER_X = 128.2
PLATFORM_CENTER_Y = 53.5
PLATFORM_TOP_Z = 1.4
SPAWN_Z = PLATFORM_TOP_Z + 0.35

# Leader is in front, followers spawn behind it in one row on the platform.
LEADER_X, LEADER_Y = PLATFORM_CENTER_X, PLATFORM_CENTER_Y + 2.0
FOLLOWER_ROW_Y = PLATFORM_CENTER_Y - 1.4
FOLLOWER_ROW_XS = {
        1: PLATFORM_CENTER_X - 2.0,
        2: PLATFORM_CENTER_X,
        3: PLATFORM_CENTER_X + 2.0,
}

def generate_launch_description():
        actions = []
        # Leader (Starts Gazebo)
        actions.append(
                leader_instanse(LEADER_X, LEADER_Y, SPAWN_Z)
        )
        followers = [
                (idx, round(x, 3), round(FOLLOWER_ROW_Y, 3), SPAWN_Z)
                for idx, x in FOLLOWER_ROW_XS.items()
        ]

        # Delay followers so Gazebo is ready.
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
