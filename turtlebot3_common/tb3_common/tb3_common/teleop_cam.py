#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import traceback
import cv2
import os

def main():
    # ノードの初期化
    rclpy.init()
    node = Node("tb3_joy_teleop")

    try:
        if 'waffle' not in os.environ['TURTLEBOT3_MODEL']:
            rclpy.logging.get_logger('tb3_joy_teleop').logerr('Can\'t use camera this robot model')
            raise KeyboardInterrupt

        bridge = CvBridge()

        def joy_cb(img):
            twist = Twist()
            twist_control = False
            try:
                img = bridge.imgmsg_to_cv2(img, 'rgb8')
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            except CvBridgeError:
                node.get_logger().logerr("cv_bridge error")

            key = cv2.waitKey(1) & 0xFF
            if key == ord('w'):
                twist.linear.x = 0.2
                twist_control = True
            if key == ord('s'):
                twist.linear.x = -0.2
                twist_control = True
            if key == ord('d'):
                twist.angular.z = -1.0
                twist_control = True
            if key == ord('a'):
                twist.angular.z = 1.0
                twist_control = True
            if key == ord('x'):
                twist.linear.x = 0.
                twist.angular.z = 0.
                twist_control = True

            cv2.imshow("TB3 fromt camera", img)
            if twist_control:
                cmd_vel_pub.publish(twist)

        # cmd_vel へ publisher
        cmd_vel_pub = node.create_publisher(Twist, "cmd_vel", 10)

        # joy を subscribe
        joy_sub = node.create_subscription(Image, "/camera/image_raw", joy_cb, 10)

        # ノードをスピン
        rclpy.spin(node)

    except KeyboardInterrupt:
        rclpy.logging.get_logger("tb3_joy_teleop").info("Done")
        
if __name__ == "__main__":
    main()
