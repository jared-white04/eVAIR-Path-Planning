#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import RRT_Algorithm as rrt
import B_Spline as bsp

class FindPathNode(Node):
    def __init__(self):
        super().__init__("INSERTNAME")
        self.path_publisher_ = self.create_publisher(
            # type
            # topic
            10)
        self.map_subscriber_ = self.create_subscription(
            # type
            # topic
            self.map_callback,
            10
        )
        self.get_logger().info("")
    
    def map_callback(self, msg):
        cnct = True
        count = 0
        # ShowMap(goal=goal, bounds=obstacle_bounds, graph=graph)
        if cnct:
            self.get_logger().info(f"\033[92mPath found in {count} iterations.\033[00m")
        else:
            self.get_logger().info(f"\033[91mNo path found in {count} iterations.\033[00m")