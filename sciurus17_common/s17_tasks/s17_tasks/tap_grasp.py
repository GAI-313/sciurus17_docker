#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from sensor_msgs.msg import PointCloud2

from s17_common.robot_control import RobotControl

import math
import time

def main():
    rclpy.init()
    node = Node('grasp')

    rc = RobotControl(node)

    while True:
        rc.init_pose(duration=0.1)
        rc.neck(pitch=math.radians(-80),duration=0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
