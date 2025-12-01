#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import random as rand
import matplotlib.patches as patches
import math
from heapq import heappush, heappop
from basic_map import create_map
import RRTlearning 

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
        map_area, start, goal, obstacle_bounds = create_map()
        graph, vpath, cnct, count = RRT(map_area=map_area, start=start, goal=goal, obstacle_bounds=obstacle_bounds)
        # ShowMap(goal=goal, bounds=obstacle_bounds, graph=graph)
        if cnct:
            self.get_logger().info(f"\033[92mPath found in {count} iterations.\033[00m")
        else:
            self.get_logger().info(f"\033[91mNo path found in {count} iterations.\033[00m")

def Euclidean(p1, p2):
    return (math.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 ))

def ShowMap(goal, bounds, graph):
    edges = graph[0]
    vertices = graph[1]
    #plotting goals and shapes
    fig, ax = plt.subplots()
    goalsq = patches.Rectangle((goal[0][0], goal[1][0]), (goal[0][1] - goal[0][0]), (goal[1][1] - goal[1][0]), edgecolor=(0, 1, 0), facecolor=(0, 1, 0))
    ax.add_patch(goalsq)

    for v in vertices:
        plt.plot(v[0], v[1],'.', color=(0, .5, .5))
        ax.add_patch(goalsq)

    for bound in bounds:
        boundbox = patches.Rectangle((bound[0][0], bound[1][0]), (bound[0][1] - bound[0][0]), (bound[1][1] - bound[1][0]), edgecolor=(0, .75, .75), facecolor=(0, .75, .75))
        ax.add_patch(boundbox)
        
    for e in edges:
        p1 = [ e[0][0], e[1][0] ]
        p2 = [ e[0][1], e[1][1]]
        edgeline = patches.FancyArrowPatch(
            p1, p2,
            arrowstyle='-',
            linewidth=2,
            color='black'
        )
        ax.add_patch(edgeline)
    ### map ###

    plt.plot()

    plt.show()


def GetRandomPosition(area):
    min_x, max_x = area[0][0], area[0][1]
    min_y, max_y = area[1][0], area[1][1]
    randx = rand.randint(min_x, max_x)
    randy = rand.randint(min_y, max_y)
    return [randx, randy]


def WithinObstacle(point, bounds):
    for obs in bounds:
        if point[0] >= obs[0][0] and point[0] <= obs[0][1]:
            if point[1] >= obs[1][0] and point[1] <= obs[1][1]:
                return True
    return False


def NearestVertex(vertices, point):
    nearestVIndex = -1
    shortestDist = 10**10 #very long dist
    for i in range(0,len(vertices)):
        v = vertices[i]
        dist2 = Euclidean(point, v)
        if (dist2 < shortestDist):
            shortestDist = dist2
            nearestVIndex = i
    return vertices[nearestVIndex]


def MoveToClosest(point, nearest):
    delta_q = 50
    vector = [point[0] - nearest[0], point[1] - nearest[1]]
    magnitude = Euclidean(point, nearest)
    additionVector = [ (math.sqrt(delta_q)*vector[0])/magnitude, (math.sqrt(delta_q)*vector[1])/magnitude ]
    newPoint = [nearest[0] + additionVector[0], nearest[1] + additionVector[1]]
    return newPoint


def RRT(map_area, start, goal, obstacle_bounds):
    edges = [] #ARRAY of PAIRS of BOUNDS
    vertices = []
    limit = 10000 #max iterations 
    path_exists = False
    vstart = [start[0][0], start[1][0]]# start = [[0],[0]]
    vertices.append(vstart) 
    
    for i in range(limit):
        
        randPoint = GetRandomPosition(area=map_area)
        nearestPoint = NearestVertex(vertices=vertices, point=randPoint)
        newPoint = MoveToClosest(point=randPoint, nearest=nearestPoint)
        
        
        if (WithinObstacle(point=newPoint,bounds=obstacle_bounds)):
            continue
        vertices.append(newPoint)
        
        nearestPoint = NearestVertex(vertices=vertices, point=newPoint)
        
        newEdge = [ [nearestPoint[0], newPoint[0]], [nearestPoint[1], newPoint[1]] ]
        edges.append(newEdge)
        
        if newPoint[0] >= goal[0][0] and newPoint[0] <= goal[0][1]:
            if newPoint[1] >= goal[1][0] and newPoint[1] <= goal[1][1]:
                v_path = RRTlearning.findpath(edges, vstart)
                path_exists = True
                return (edges, vertices), v_path, path_exists, i
        
    return (edges, vertices), v_path, path_exists, limit
    
def main(args=None):
    rclpy.init(args=args)
    node = FindPathNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()