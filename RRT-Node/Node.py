#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import RRT_Algorithm as rrt
import B_Spline as bsp
import BinaryOccupancyMap
from std_msgs.msg import Float64MultiArray

class FindPathNode(Node):
    def __init__(self):
        super().__init__("get_path")
        self.path_publisher_ = self.create_publisher(
            Float64MultiArray,
            '/Xpath',
            10)
        self.time_publisher_ = self.create_publisher(
            Float64MultiArray,
            '/tpath',
            10)
        self.timer = self.create_timer(1.0, self.FindPath)
        self.get_logger().info("FindPathNode has started.")
    
    def FindPath(self, start, goal, binaryMap):
        area = [ [0, len(binaryMap[0])-1], [0, len(binaryMap)-1] ]
        print(area[0][0], area[0][1], area[1][0], area[1][1])
        path, cnct, vertices = rrt.GetPath(binaryMap, area, start, goal)

        
        # Smooth path with B-Spline
        res = 50
        smoothPath, tPoints = bsp.GetBSpline(path, res, binaryMap)
        
        xpointmsg = PathToMsg(smoothPath)
        tpointmsg = TimeToMsg(tPoints)
        
        self.path_publisher_.publish(xpointmsg)
        self.time_publisher_.publish(tpointmsg)

def FindStart(binaryMap):
    for r in range(len(binaryMap)):
        for c in range(len(binaryMap[r])):
            if binaryMap[r][c] == 0:
                start = (c, r)
                return start
    return None

def PathToMsg(path):
    msg = Float64MultiArray()
    flat_path = []
    for point in path:
        flat_path.append(point[0])
        flat_path.append(point[1])
    msg.data = flat_path
    return msg

def TimeToMsg(tPoints):
    msg = Float64MultiArray()
    msg.data = tPoints
    return msg

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