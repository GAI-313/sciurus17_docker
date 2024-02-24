#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from s17_msgs.srv import SetPose

import time
import math

class RobotControl:
    def __init__ (self, node):
        self.node = node

        self.cli = self.node.create_client(SetPose, 's17_common/set_pose')
        while not self.cli.wait_for_service(timeout_sec=10):
            self.node.get_logger().warn('set_pose node is not running ...')

    def _send_req(self, joint, position, duration):
        req = SetPose.Request()
        req.joint_name = joint
        req.position = position
        req.duration = float(duration)
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def left_arm(self, duration=5, wait=True, **kwargs):
        joints = {'l_arm_' + name: value for name, value in kwargs.items() if value is not None}
        result = self._send_req(list(joints.keys()), list(joints.values()), duration)
        if wait:
            time.sleep(duration)
        if not result.success:
            self.node.get_logger().warn(result.result)

    def right_arm(self, duration=5, wait=True, **kwargs):
        joints = {'r_arm_' + name: value for name, value in kwargs.items() if value is not None}
        result = self._send_req(list(joints.keys()), list(joints.values()), duration)
        if wait:
            time.sleep(duration)
        if not result.success:
            self.node.get_logger().warn(result.result)

    def neck(self, duration=5, wait=True, **kwargs):
        joints = {'neck_' + name + '_joint': value for name, value in kwargs.items() if value is not None}
        result = self._send_req(list(joints.keys()), list(joints.values()), duration)
        if wait:
            time.sleep(duration)
        if not result.success:
            self.node.get_logger().warn(result.result)

    def waist_yaw(self, yaw, duration=5, wait=True):
        joints = {'waist_yaw_joint': yaw}
        result = self._send_req(list(joints.keys()), list(joints.values()), duration)
        if wait:
            time.sleep(duration)
        if not result.success:
            self.node.get_logger().warn(result.result)

    def init_pose(self, duration=5, wait=True):
        self.left_arm( duration, wait=False, joint1 = 1.57,
                        joint2 = 1.57,
                        joint3 = 0.0,
                        joint4 = -2.36,
                        joint5 = 0.0,
                        joint6 = 0.75,
                        joint7 = 0.0)

        self.right_arm( duration, wait=False, joint1 = -1.57,
                        joint2 = -1.57,
                        joint3 = 0.0,
                        joint4 = 2.36,
                        joint5 = 0.0,
                        joint6 = -0.75,
                        joint7 = 0.0)

        self.neck(duration, wait=False, pitch = 0.0, yaw=0.0)
        self.waist_yaw(0.0, duration, wait)

    def t_pose(self, duration=5, wait=True):
        self.left_arm(duration, wait=False, joint1 = 1.57,
                        joint2 = 0.0,
                        joint3 = 0.0,
                        joint4 = 0.0,
                        joint5 = 0.0,
                        joint6 = 0.,
                        joint7 = 0.0)

        self.right_arm(duration, wait=False, joint1 = -1.57,
                        joint2 = 0.0,
                        joint3 = 0.0,
                        joint4 = 0.0,
                        joint5 = 0.0,
                        joint6 = 0.0,
                        joint7 = 0.0)

if __name__ == '__main__':
    rclpy.init()
    node = Node('sample_robot_control')
    rc = RobotControl(node)
    rc.init_pose(duration=0.5)

    node.destroy_node()
    rclpy.shutdown()
