#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
from rclpy.node import Node
import rclpy
import smach
import time
import traceback
import math

class SerchingObject(smach.State):
    def __init__ (self, node, obj_detect_client, rc):
        smach.State.__init__(self, outcomes=['found','not_found','clean','failure'],
                                   input_keys=['timeout_sec', 'hand_status'],
                                   output_keys=['object_pose'])
        self.node = node
        self.rc = rc
        self.obj_detect_client = obj_detect_client
        self.pose = None # target object pose
        self.obj_sub = self.node.create_subscription(PoseStamped, 's17_vision/object_info', self.object_cb, 10)

    def send_req(self, data):
        req = SetBool.Request()
        req.data = data
        future = self.obj_detect_client.call_async(req)
        self.node.get_logger().info(str(future))

    def object_cb(self, data):
        self.pose = data
        
    def execute(self, userdata):
        self.rc.neck_joints(pitch=math.radians(-80), duration=3.0)
        self.node.get_logger().info('Serching object ... wait time is %f sec'%userdata.timeout_sec)
        self.pose = None # delete data
        try:
            time.sleep(1.0)
            print('start space_finder')
            self.send_req(True) # object serching start
            init_time = time.time()
            while time.time() - init_time < userdata.timeout_sec:
            #while self.pose is None:
                rclpy.spin_once(self.node)
                if time.time() - init_time > userdata.timeout_sec / 2 and self.pose is not None:
                    print('Found !')
                    break
            print('stop space_finder')
            self.send_req(False)
            if self.pose is None:
                if any(userdata.hand_status):
                    self.node.get_logger().info('Put other object ...')
                    return 'clean'
                return 'not_found'
            self.node.get_logger().info('Found object in \n%s'%str(self.pose.pose.position))
            userdata.object_pose = self.pose
            return 'found'
        except:
            self.node.get_logger().error('Error is occured in SerchingObject \n%s'%traceback.format_exc())
            return 'failure'
