#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import traceback

class TestSub(Node):
    def __init__(self):
        super().__init__('test_sub_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/world/baylands_custom/model/x500_mono_cam_1/link/mono_cam/base_link/sensor/camera_sensor/image',
            self.callback,
            10
        )
        print("Subscriber created!")

    def callback(self, msg):
        print(f"Received message! Height: {msg.height}, Width: {msg.width}")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            print("Successfully converted to OpenCV image!")
            sys.exit(0)
        except Exception as e:
            print(f"Exception in callback: {e}")
            traceback.print_exc()

def main():
    rclpy.init()
    node = TestSub()
    try:
        rclpy.spin(node)
    except SystemExit:
        print("Exiting successfully after frame conversion.")
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
