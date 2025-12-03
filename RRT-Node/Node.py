#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import RRT_Algorithm as rrt
import B_Spline as bsp
import BinaryOccupancyMap

class FindPathNode(Node):
    def __init__(self):
        super().__init__("INSERTNAME")
        # self.path_publisher_ = self.create_publisher(
        #     # type
        #     # topic
        #     10)
        self.get_logger().info("")
    
    def FindPath(self, start, goal, binaryMap):
        area = [ [0, len(binaryMap[0])-1], [0, len(binaryMap)-1] ]
        print(area[0][0], area[0][1], area[1][0], area[1][1])
        path, cnct, vertices = rrt.GetPath(binaryMap, area, start, goal)
        
        if not cnct:
            self.get_logger().info("\033[91mNo path found.\033[00m")
            rrt.ShowMap([], goal, binaryMap, vertices)
            return None
        
        # Smooth path with B-Spline
        res = 50
        smoothPath, tPoints = bsp.GetBSpline(path, res, binaryMap)
        
        self.get_logger().info("\033[92mPath to goal found.\033[00m")
        rrt.ShowMap(path, goal, binaryMap, vertices)
        bsp.ShowBSpline(smoothPath, tPoints, goal, binaryMap, vertices)
        
        return smoothPath

def FindStart(binaryMap):
    for r in range(len(binaryMap)):
        for c in range(len(binaryMap[r])):
            if binaryMap[r][c] == 0:
                start = (c, r)
                return start
    return None

def main(args=None):
    rclpy.init(args=args)
    node = FindPathNode()
    map = BinaryOccupancyMap.Get2DBinaryMapSlice('voxel_grid', 50)
    start = FindStart(map)
    goal = [[len(map[0])-20, len(map)-20],[len(map[0])-15, len(map)-15]]
    # start = (0,0)
    node.FindPath(start, goal, map)
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()