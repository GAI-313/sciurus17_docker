#!/usr/bin/env python
from rclpy.node import Node
import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import traceback

def main():
    # ノードの初期化
    rclpy.init()
    node = Node("tb3_joy_teleop")

    try:
        twist = Twist()

        def joy_cb(joy):
            if joy.buttons[5]:
                twist.linear.x = joy.axes[1] / 2
                twist.angular.z = joy.axes[0] / 2

                if joy.buttons[7]:
                    twist.linear.x = joy.axes[1]
                    twist.angular.z = joy.axes[0]
                    
                cmd_vel_pub.publish(twist)

        # cmd_vel へ publisher
        cmd_vel_pub = node.create_publisher(Twist, "cmd_vel", 10)

        # joy を subscribe
        joy_sub = node.create_subscription(Joy, "joy", joy_cb, 10)

        # ノードをスピン
        rclpy.spin(node)

    except KeyboardInterrupt:
        rclpy.logging.get_logger("tb3_joy_teleop").info("Done")
        
if __name__ == "__main__":
    main()
