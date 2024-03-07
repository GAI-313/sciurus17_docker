#!/usr/bin/env python3
from s17_common.robot_control import RobotControl

from rclpy.node import Node
import rclpy

import math

def main():
    rclpy.init()
    node = Node('gaze_table')

    rc = RobotControl(node)
    rc.init_pose()
    rc.neck_joints(pitch=math.radians(-80))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
