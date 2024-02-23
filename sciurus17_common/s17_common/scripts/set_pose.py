#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from sensor_msgs.msg import JointState

from s17_common.srv import SetPose

import traceback

class SetPose:
    def __init__(self, node):
        self.node = node

        self.js_sub = self.node.create_subscription(JointState, '/joint_states', 10, self.js_callback)

    def js_callback(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            print(name, pose)

def main():
    rclpy.init()
    node = Node('set_pose')
    sp = SetPose(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
