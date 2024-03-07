#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

def GraspObject():

def main():
    rclpy.init()
    node = Node('grasp_object')
    rc = GraspObject(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
