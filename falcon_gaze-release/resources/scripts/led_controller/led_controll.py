#!/usr/bin/env python3
import sys
import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Ros2LedConfigOrchestrator(Node):
    def __init__(self, drone_id=0, config_filename="sequences.json", mission_point="first_point"):
        super().__init__(f'led_orchestrator_{drone_id}')
        self.drone_id = drone_id
        self.config_filename = config_filename
        self.mission_point = mission_point
        
        # Output channel pointing directly to the Gazebo model string bridge
        topic_name = f'/model/x500_mono_cam_{drone_id}/led_cmd'
        self.publisher_ = self.create_publisher(String, topic_name, 10)
        
        self.sequence_step = 0
        self.delay_time = 0.05  # Safe fallback
        self.should_loop = True  # Safe fallback
        
        # Parse JSON configuration properties
        self.my_sequence = self.load_sequence_from_config()
        
        # Variable-Speed Tick Initialization
        self.timer = self.create_timer(self.delay_time, self.timer_callback)
        self.get_logger().info(f"Orchestrator online for drone {drone_id}. Speed tick rate set to: {self.delay_time}s")

    def load_sequence_from_config(self):
        """Reads JSON and dynamically builds a timeline array with runtime loop properties."""
        script_dir = os.path.dirname(os.path.realpath(__file__))
        config_path = os.path.join(script_dir, self.config_filename)
        
        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
            
            mission = config["swarm_missions"][self.mission_point]
            
            # Extract time configurations and tracking modes
            self.delay_time = float(mission["delay_time"])
            self.should_loop = bool(mission.get("loop", True)) # Defaults to True if omitted
            
            master_sequence = []
            for step in mission["timeline"]:
                mask = step["mask"]
                ticks = int(step["ticks"])
                master_sequence += [mask] * ticks
                
            self.get_logger().info(f"Loaded [{self.mission_point}] (Looping={self.should_loop}) with {len(master_sequence)} steps.")
            return master_sequence
            
        except Exception as e:
            self.get_logger().error(f"Failed parsing dynamic timeline target: {str(e)}")
            self.delay_time = 0.05
            self.should_loop = True
            return ["0000"]

    def send_cmd(self, command_string):
        msg = String()
        msg.data = command_string
        self.publisher_.publish(msg)

    def timer_callback(self):
        current_bitmask = self.my_sequence[self.sequence_step]
        self.send_cmd(current_bitmask)
        
        # --- NEW LOOP SAFETY FLOW ENGINE ---
        next_step = self.sequence_step + 1
        
        if next_step < len(self.my_sequence):
            # Advance normally if we haven't hit the end of the array
            self.sequence_step = next_step
        else:
            if self.should_loop:
                # Wrap back around to the beginning if looping is active
                self.sequence_step = 0
            else:
                # One-shot mode: Remain stationary on the last state block to avoid loop flickering
                pass

def main(args=None):
    rclpy.init(args=args)
    
    target_id = 0
    target_file = "sequences.json"
    target_point = "first_point"
    
    if len(sys.argv) > 1:
        try:
            target_id = int(str(sys.argv[1]).strip())
        except ValueError:
            target_id = 0
            
    if len(sys.argv) > 2:
        target_file = str(sys.argv[2]).strip()
        
    if len(sys.argv) > 3:
        target_point = str(sys.argv[3]).strip()

    node = Ros2LedConfigOrchestrator(drone_id=target_id, config_filename=target_file, mission_point=target_point)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.send_cmd("0000")
            except Exception:
                pass
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()