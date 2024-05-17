#!/usr/bin/env python3
from rclpy.node import Node
from rclpy.action import ActionClient
import rclpy

from s17_msgs.srv import SetPose, SetPosition
from control_msgs.action import GripperCommand

from tf_transformations import quaternion_from_euler

import time
import math

class RobotControl:
    def __init__ (self, node):
        self.node = node

        self.pose_cli = self.node.create_client(SetPose, 's17_common/set_pose')
        self.l_position_cli = self.node.create_client(SetPosition, '/s17_common/l_arm/set_position')
        self.r_position_cli = self.node.create_client(SetPosition, '/s17_common/r_arm/set_position')
        while not self.pose_cli.wait_for_service(timeout_sec=10) or not self.l_position_cli.wait_for_service(timeout_sec=10) or not self.r_position_cli.wait_for_service(timeout_sec=10):
            self.node.get_logger().warn('set_pose node is not running ...')
        self.l_gripper_action = ActionClient(self.node, GripperCommand, '/left_gripper_controller/gripper_cmd')
        self.r_gripper_action = ActionClient(self.node, GripperCommand, '/right_gripper_controller/gripper_cmd')

    def _send_pose_req(self, joint, position, duration):
        req = SetPose.Request()
        req.joint_name = joint
        req.position = position
        req.duration = float(duration)
        future = self.pose_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def left_arm_joints(self, duration=5, wait=True, **kwargs):
        joints = {'l_arm_' + name: value for name, value in kwargs.items() if value is not None}
        result = self._send_pose_req(list(joints.keys()), list(joints.values()), duration)
        if wait:
            time.sleep(duration)
        if not result.success:
            self.node.get_logger().warn(result.result)

    def right_arm_joints(self, duration=5, wait=True, **kwargs):
        joints = {'r_arm_' + name: value for name, value in kwargs.items() if value is not None}
        result = self._send_pose_req(list(joints.keys()), list(joints.values()), duration)
        if wait:
            time.sleep(duration)
        if not result.success:
            self.node.get_logger().warn(result.result)

    def neck_joints(self, duration=5, wait=True, **kwargs):
        joints = {'neck_' + name + '_joint': value for name, value in kwargs.items() if value is not None}
        result = self._send_pose_req(list(joints.keys()), list(joints.values()), duration)
        if wait:
            time.sleep(duration)
        if not result.success:
            self.node.get_logger().warn(result.result)

    def waist_yaw_joints(self, yaw, duration=5, wait=True):
        joints = {'waist_yaw_joint': yaw}
        result = self._send_pose_req(list(joints.keys()), list(joints.values()), duration)
        if wait:
            time.sleep(duration)
        if not result.success:
            self.node.get_logger().warn(result.result)

    def init_pose(self, duration=5, wait=True):
        self.left_gripper(False)
        self.right_gripper(False)
        
        self.left_arm_joints( duration, wait=False, joint1 = 1.57,
                        joint2 = 1.57,
                        joint3 = 0.0,
                        joint4 = -2.36,
                        joint5 = 0.0,
                        joint6 = 0.75,
                        joint7 = 0.0)

        self.right_arm_joints( duration, wait=False, joint1 = -1.57,
                        joint2 = -1.57,
                        joint3 = 0.0,
                        joint4 = 2.36,
                        joint5 = 0.0,
                        joint6 = -0.75,
                        joint7 = 0.0)

        self.neck_joints(duration, wait=False, pitch = 0.0, yaw=0.0)
        self.waist_yaw_joints(0.0, duration, wait)

    def t_pose(self, duration=5, wait=True):
        self.left_arm_joints(duration, wait=False, joint1 = 1.57,
                        joint2 = 0.0,
                        joint3 = 0.0,
                        joint4 = 0.0,
                        joint5 = 0.0,
                        joint6 = 0.,
                        joint7 = 0.0)

        self.right_arm_joints(duration, wait=False, joint1 = -1.57,
                        joint2 = 0.0,
                        joint3 = 0.0,
                        joint4 = 0.0,
                        joint5 = 0.0,
                        joint6 = 0.0,
                        joint7 = 0.0)

    def _g_callback(self, feedback):
        self.node.get_logger().info(feedback.position)

    def _g_goal_resp(self, future):
        #self.node.get_logger().info(str(future.reached_goal))
        pass
        
    def left_gripper(self, open=True, space=1.0):
        goal = GripperCommand.Goal()
        goal.command.position = 0.0 if not open else -space
        goal.command.max_effort = 1.0

        if not self.l_gripper_action.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("Gripper action server is not available.")
            return

        future = self.l_gripper_action.send_goal_async(goal, self._g_callback)
        future.add_done_callback(self._g_goal_resp)

        time.sleep(1.0)

    def right_gripper(self, open=True, space=1.0):
        goal = GripperCommand.Goal()
        goal.command.position = 0.0 if not open else space
        goal.command.max_effort = 1.0

        if not self.l_gripper_action.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("Gripper action server is not available.")
            return

        future = self.r_gripper_action.send_goal_async(goal, self._g_callback)
        future.add_done_callback(self._g_goal_resp)

        time.sleep(1.0)

    def left_arm_position(self, x,y,z, roll=0., pitch=0., yaw=0.):
        q = quaternion_from_euler(roll, pitch, yaw)
        req = SetPosition.Request()

        req.x = x
        req.y = y
        req.z = z

        req.ox = q[0]
        req.oy = q[1]
        req.oz = q[2]
        req.ow = q[3]

        future = self.l_position_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        print(future)

    def right_arm_position(self, x,y,z, roll=0., pitch=0., yaw=0.):
        q = quaternion_from_euler(roll, pitch, yaw)
        req = SetPosition.Request()

        req.x = x
        req.y = y
        req.z = z

        req.ox = q[0]
        req.oy = q[1]
        req.oz = q[2]
        req.ow = q[3]

        future = self.r_position_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        print(future)

if __name__ == '__main__':
    rclpy.init()
    node = Node('sample_robot_control')
    rc = RobotControl(node)
    rc.right_gripper(True)
    rc.right_gripper(False)
    
    node.destroy_node()
    rclpy.shutdown()
