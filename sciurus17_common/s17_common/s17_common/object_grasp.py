#!/usr/bin/env python3
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy
from tf2_ros.transform_listener import TransformListener
import tf2_ros

import smach

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

from s17_common.robot_control import RobotControl

import math
import traceback

class ObjectGrasp(smach.State):
    def __init__(self, node, rc):
        smach.State.__init__(self, outcomes=['success', 'continue', 'timeout', 'failure'],
                                   input_keys=['object_pose', 'hand_status'],
                                   output_keys=['hand_status'])
        self.node = node
        self.rc = rc

    def execute(self, userdata):
        try:
            pose = userdata.object_pose
            pose_point = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]

            if not userdata.hand_status[0]:
                self.rc.right_gripper()
                self.rc.right_arm_position(pose_point[0],
                                            pose_point[1],
                                            0.3, roll=math.radians(90))
                self.rc.right_arm_position(pose_point[0],
                                            pose_point[1],
                                            pose_point[2]+0.07, roll=math.radians(90))
                self.rc.right_gripper(False)
                self.rc.right_arm_position(pose_point[0],
                                            pose_point[1],
                                            0.3, roll=math.radians(90))
                userdata.hand_status[0] = True

                self.rc.init_pose(duration=3.0, wait=False)
                self.rc.neck_joints(pitch=math.radians(-80), duration=3.0)

                return 'continue'
            else:
                self.rc.left_arm_position(pose_point[0],
                                            pose_point[1],
                                            0.3, roll=math.radians(-90))
                self.rc.left_gripper()
                self.rc.left_arm_position(pose_point[0],
                                            pose_point[1],
                                            pose_point[2]+0.07, roll=math.radians(-90))
                self.rc.left_gripper(False)
                self.rc.left_arm_position(pose_point[0],
                                            pose_point[1],
                                            0.3, roll=math.radians(-90))
                self.rc.waist_yaw_joints(math.radians(0), duration=3.0)
                self.rc.left_arm_position(0.1, 0.1, 0.1,
                                            roll=math.radians(-90))
                userdata.hand_status[1] = True
            return 'success'
        except:
            self.node.get_logger().error('Error is occured in ObjectGrasp \n%s'%traceback.format_exc())
            return 'failure'


class _ObjectGrasp:
    def __init__(self, node):
        self.node = node

        self.serch_object = False
        self.pose_list = []

        self.sf_cli = self.node.create_client(SetBool, '/s17_vision/space_finder')
        while not self.sf_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
            
        self.rc = RobotControl(self.node)
        self.rc.init_pose(wait=False)
        self.rc.neck_joints(pitch=math.radians(-80))

        self.obj_sub = self.node.create_subscription(PoseStamped, '/s17_vision/object_info', self.object_cb, 10)
        self.grasp()

        self.rc.init_pose(wait=False)

    def send_req(self, data):
        req = SetBool.Request()
        req.data = data
        future = self.sf_cli.call_async(req)
        self.node.get_logger().info(str(future))

    def grasp(self):
        self.node.get_logger().info('serching object ...')
        self.send_req(True)
        
        while len(self.pose_list) < 3:
            rclpy.spin_once(self.node)
            continue
        self.node.get_logger().info('grasping...')
        print(self.pose_list)

        if self.pose_list[1] > 0:
            self.rc.waist_yaw_joints(math.radians(45))

        self.rc.right_arm_position(self.pose_list[0],
                                    self.pose_list[1],
                                    0.3, roll=math.radians(90))
        self.rc.right_gripper()
        
        self.rc.right_arm_position(self.pose_list[0],
                                    self.pose_list[1],
                                    self.pose_list[2]+0.07, roll=math.radians(90))
        self.rc.right_gripper(False)

        self.rc.right_arm_position(self.pose_list[0],
                                    self.pose_list[1],
                                    0.3, roll=math.radians(90))

        self.rc.right_arm_position(self.pose_list[0],
                                    self.pose_list[1],
                                    0.3, roll=math.radians(90))
        self.rc.right_gripper()
        
    def object_cb(self, msg):
        if len(self.pose_list) < 3:
            print(msg.pose.position)
            self.send_req(False)

            self.pose_list = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ]

def main():
    rclpy.init()
    node = Node('object_grasp')
    og = _ObjectGrasp(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
