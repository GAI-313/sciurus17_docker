#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from s17_msgs.srv import SetPose

import traceback

class PoseControl:
    def __init__(self, node):
        self.node = node

        self.node.get_logger().info('pose controller sart !')
        
        self.js_position = {}
        self.l_traj_pub = self.node.create_publisher(JointTrajectory, 'left_arm_controller/joint_trajectory', 10)
        self.r_traj_pub = self.node.create_publisher(JointTrajectory, 'right_arm_controller/joint_trajectory', 10)
        self.n_traj_pub = self.node.create_publisher(JointTrajectory, 'neck_controller/joint_trajectory', 10)
        self.w_traj_pub = self.node.create_publisher(JointTrajectory, 'waist_yaw_controller/joint_trajectory', 10)
        self.js_sub = self.node.create_subscription(JointState, '/joint_states', self._js_callback, 10)
        self.sp_srv = self.node.create_service(SetPose, 's17_common/set_pose', self._srv_cb)

    def _js_callback(self, msg):
        self.js_position.update(zip(msg.name, msg.position))
        #print(self.js_position)

    def _srv_cb(self, req, res):
        if len(self.js_position) == 0:
            self.node.get_logger().info('Could not found joint_status')

            res.result = 'Counld not fount joint_status'
            res.success = False
        else:
            result = 'success'
            for joint_name, position in  zip(req.joint_name, req.position):
                if joint_name in self.js_position:
                    self.js_position[joint_name] = position
                else:
                    self.node.get_logger().warn('Joint \'%s\' is not exist !'%joint_name)
                    result = 'Joint \'%s\' is not exist !'%joint_name

            self._joint_publish()

            res.result = result
            res.success = True

        return res

    def _joint_publish(self):
        l_traj = JointTrajectory()
        r_traj = JointTrajectory()
        n_traj = JointTrajectory()
        w_traj = JointTrajectory()

        l_traj_dict= {}
        r_traj_dict= {}
        w_traj_dict= {}
        n_traj_dict= {}

        for j in self.js_position:
            if j.startswith('l_a'):
                l_traj_dict.update([(j, self.js_position[j])])
            if j.startswith('r_a'):
                r_traj_dict.update([(j, self.js_position[j])])
            if j.startswith('neck'):
                n_traj_dict.update([(j, self.js_position[j])])
            if j.startswith('waist'):
                w_traj_dict.update([(j, self.js_position[j])])

        point = JointTrajectoryPoint()
        l_traj.joint_names = list(l_traj_dict.keys())
        point.positions = list(l_traj_dict.values())
        point.time_from_start = rclpy.duration.Duration(seconds=2).to_msg()
        l_traj.points.append(point)

        point = JointTrajectoryPoint()
        r_traj.joint_names = list(r_traj_dict.keys())
        point.positions = list(r_traj_dict.values())
        point.time_from_start = rclpy.duration.Duration(seconds=2).to_msg()
        r_traj.points.append(point)

        point = JointTrajectoryPoint()
        n_traj.joint_names = list(n_traj_dict.keys())
        point.positions = list(n_traj_dict.values())
        point.time_from_start = rclpy.duration.Duration(seconds=2).to_msg()
        n_traj.points.append(point)

        point = JointTrajectoryPoint()
        w_traj.joint_names = list(w_traj_dict.keys())
        point.positions = list(w_traj_dict.values())
        point.time_from_start = rclpy.duration.Duration(seconds=2).to_msg()
        w_traj.points.append(point)

        self.l_traj_pub.publish(l_traj)
        self.r_traj_pub.publish(r_traj)
        self.w_traj_pub.publish(w_traj)
        self.n_traj_pub.publish(n_traj)

def main():
    rclpy.init()
    node = Node('set_pose')
    pc = PoseControl(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
