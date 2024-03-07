#!/usr/bin/env python3
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy
from tf2_ros.transform_listener import TransformListener
import tf2_ros

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

from s17_common.robot_control import RobotControl

import math

class ObjectGrasp:
    def __init__(self, node):
        self.node = node

        self.serch_object = False
        self.pose_list = []

        self.sf_cli = self.node.create_client(SetBool, '/s17_vision/space_finder')
        while not self.sf_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
            
        self.rc = RobotControl(self.node)
        self.rc.init_pose()
        self.rc.neck_joints(pitch=math.radians(-80))

        self.obj_sub = self.node.create_subscription(PoseStamped, '/s17_vision/object_info', self.object_cb, 10)
        self.grasp()

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
    og = ObjectGrasp(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
