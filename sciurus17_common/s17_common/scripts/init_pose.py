#!/usr/bin/env python3
from s17_common.robot_control import RobotControl
from rclpy.node import Node
import rclpy

def main():
    rclpy.init()
    node = Node('s17_init_pose')
    rc = RobotControl(node)

    rc.init_pose(duration=1.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
