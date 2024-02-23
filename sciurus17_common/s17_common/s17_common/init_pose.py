#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

class InitPose:
    def __init__(self node):
        self.node = node
        self.node.get_logger().info('Go to init pose ...')

    def get_current_pose():

def main():
    rclpy.init()
    node = Node('s17_init_pose')

    ip = InitPose(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
