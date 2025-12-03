import matplotlib.pyplot as plt
import numpy as np
import random as rand
import matplotlib.patches as patches
import math
# import B_Spline as bsp
import RRTlearning 

def Euclidean(p1, p2):
    return (math.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 ))

def ShowMap(path, goal, map, vertices):
    
    fig, ax = plt.subplots()
    
    goalsq = patches.Rectangle(
        (goal[0][0], goal[0][1]),             # x_min, y_min
        goal[1][0] - goal[0][0],              # width  = x_max - x_min
        goal[1][1] - goal[0][1],              # height = y_max - y_min
        edgecolor=(0, 1, 0),
        facecolor=(0, 1, 0)
    )
    ax.add_patch(goalsq)
    
    # Draw obstacles as dots in the binary occupancy map
    for row in range(len(map)):
        for column in range(len(map[row])):
            if map[row][column] == 0:
                continue
            plt.plot(column, row,'.', color=(0, .5, .5))    
            
    # Draw vertices
    for v in vertices:
        plt.plot(v[0], v[1],'.', color=(0, .75, .75))    
    
    # Draw path
    for i in range(len(path)-1):
        p1 = [ path[i][0], path[i][1] ]
        p2 = [ path[i+1][0], path[i+1][1] ]
        pathline = patches.FancyArrowPatch(
            p1, p2,
            arrowstyle='-',
            linewidth=2,
            color='blue'
        )
        ax.add_patch(pathline)

    plt.plot()

    plt.show()


def GetRandomPosition(area):
    min_x, max_x = area[0][0], area[0][1]
    min_y, max_y = area[1][0], area[1][1]
    randx = rand.randint(min_x, max_x)
    randy = rand.randint(min_y, max_y)
    return (randx, randy)


def InvalidPoint(point, binaryMap, vertices):
    x, y = point[0], point[1]
    for v in vertices:
        if v[0] == x and v[1] == y:
            return True
    # print(f'{x},{y}')
    return (binaryMap[y][x] == 1) 
    """
    !!! double check if its map[x][y] or map[y][x]!!!
    """


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


def MoveToClosest(point, nearest, xMax, yMax):
    delta_q = 2
    vector = (point[0] - nearest[0], point[1] - nearest[1])
    magnitude = Euclidean(point, nearest)
    if (magnitude == 0):
        return nearest
    additionVector = [ (math.sqrt(delta_q)*vector[0])/magnitude, (math.sqrt(delta_q)*vector[1])/magnitude ]
    newX, newY = round(nearest[0] + additionVector[0]), round(nearest[1] + additionVector[1])
    if newX < 0:
        newX = 0
    elif newX >= xMax:
        newX = xMax - 1
    if newY < 0:
        newY = 0
    elif newY >= yMax:
        newY = yMax - 1
    newPoint = (newX, newY)
    return newPoint
 

def GetPath(binaryMap, area, start, goal):
    edges = []
    vertices = []
    limit = 10000 # max iterations
    path_exists = False
    vertices.append(start)
    path = []
    
    for i in range(limit):
        
        # Generate random point 
        randPoint = GetRandomPosition(area)
        nearestPoint = NearestVertex(vertices, randPoint)
        newPoint = MoveToClosest(randPoint, nearestPoint, len(binaryMap[0]), len(binaryMap))
        
        # Only append if not in obstacle and not repeat point
        if (InvalidPoint(newPoint, binaryMap, vertices)):
            continue
        vertices.append(newPoint)
        
        # Create new edge between new point and path
        newEdge = [ [nearestPoint[0], newPoint[0]], [nearestPoint[1], newPoint[1]] ]
        edges.append(newEdge)
        
        # Check if goal has been reached
        if newPoint[0] >= goal[0][0] and newPoint[0] <= goal[1][0]:
            if newPoint[1] >= goal[0][1] and newPoint[1] <= goal[1][1]:
                vpath = RRTlearning.findpath(edges, start)
                for i in range(len(vpath[0])):
                    path.append( (vpath[0][i], vpath[1][i]) )
                path_exists = True
                return path, path_exists, vertices
    return path, path_exists, vertices
                

def main(args=None):
    map1 = [
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [0,0,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        ]
    area = [[0,len(map1[0])-1],[0,len(map1)-1]]
    start = (0,0)
    goal = [(17,8),(19,9)]
    
    path, cnct, vert = GetPath(map1, area, start, goal)
    if cnct:
        print(f'\033[92mPath found.\033[00m') 
        print(path)
        ShowMap(path, goal, map1, vert)
    else:
        print(f'\033[91mNo path found.\033[00m')
        ShowMap(path, goal, map1, vert)
    
    
if __name__ == '__main__':
    main()