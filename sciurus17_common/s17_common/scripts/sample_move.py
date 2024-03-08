#!/usr/bin/env python3
from s17_common.robot_control import RobotControl
from rclpy.node import Node
import rclpy
import math

def main():
    rclpy.init()
    node = Node('sample_mvoe')

    rc = RobotControl(node)

    # init pose
    rc.init_pose()

    # head tilt
    rc.neck_joints(pitch=math.radians(40)) # tilt up for 40 deg
    rc.neck_joints(pitch=math.radians(-40)) # tilt down for 40 deg
    # head pan
    rc.neck_joints(yaw=math.radians(90), duration=2.0) # rotate ccw for 90 deg to 2 sec
    rc.neck_joints(yaw=math.radians(-90), duration=5.0) # rotate ccw for 90 deg to 5 sec
    # sync
    rc.neck_joints(yaw=math.radians(0), wait=False) # if false for wait. this action sync to next robot action
    rc.neck_joints(pitch=math.radians(0)) # rotate ccw for 90 deg

    # waist
    rc.waist_yaw_joints(math.radians(90), duration=3.0)
    rc.waist_yaw_joints(math.radians(-90), duration=3.0)

    # init
    rc.init_pose()

    # arm control
    rc.left_gripper(True)
    rc.left_arm_position(0.3, -0.1, 0.3, roll=math.radians(-90))
    rc.left_arm_position(0.3, -0.1, 0.1, roll=math.radians(-90))
    rc.left_gripper(False)
    rc.left_arm_position(0.3, 0.1, 0.3, roll=math.radians(-90))

    #rc.init_pose()
    
    rc.right_gripper(True)
    rc.right_arm_position(0.3, 0.1, 0.3, roll=math.radians(90))
    rc.right_arm_position(0.3, 0.1, 0.1, roll=math.radians(90))
    rc.right_gripper(False)
    rc.right_arm_position(0.3, -0.1, 0.3, roll=math.radians(90))

    rc.init_pose()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
