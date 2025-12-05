import matplotlib.patches as patches
import numpy as np
import random as rand
import math
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import time
# import requests
# import argparse

# Calculate distance
def Euclidean(p1, p2):
    return (math.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 ))

# Prints map after RRT algorithm
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

# Generates random position within area
def GetRandomPosition(area):
    min_x, max_x = area[0][0], area[0][1]
    min_y, max_y = area[1][0], area[1][1]
    
    randx = rand.randint(min_x, max_x)
    randy = rand.randint(min_y, max_y)
    return (randx, randy)

# Checks if point is already tracked or if it exists in an obstacle
def InvalidPoint(point, binaryMap, vertices):
    x, y = point[0], point[1]
    for v in vertices:
        if v[0] == x and v[1] == y:
            return True
    # print(f'{x},{y}')
    return (binaryMap[y][x] == 1) 

# Finds nearest vertex to generated point
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

# Moves point closer to random point by constant distance
def MoveToClosest(point, nearest, xMax, yMax):
    delta_q = 4 # step size
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
 
# Generates an RRT path
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
                vpath = FindPathFromRRT(edges, start)
                for i in range(len(vpath[0])):
                    path.append( (vpath[0][i], vpath[1][i]) )
                path_exists = True
                return path, path_exists, vertices
    return path, path_exists, vertices

# Turns path into bspline path
def GetBSpline(vpath, res, binaryMap):
    # Find all points where obstacle exists
    obstaclePoints = []
    for row in range(len(binaryMap)):
        for column in range(len(binaryMap[row])):
            if binaryMap[row][column] == 0:
                continue
            pObs = (column, row)
            obstaclePoints.append(pObs)
    
    # Intialize
    bezLength = len(vpath)
    tPoints = np.linspace(0,1,res)
    cPoints = []
    weights = []
    
    # Add weights for each point in path
    if len(obstaclePoints) != 0:
        for v in vpath:
            shortestDist = 10*10 # really big number
            for obstacle in obstaclePoints:
                currentDist = Euclidean(v, obstacle)
                if currentDist < shortestDist:
                    shortestDist = currentDist
            w = (1/shortestDist)**2
            weights.append(w)
    else:
        weights = [1 for _ in range(bezLength)]
    
    # Px constants
    pascal = Pascal(bezLength)
    
    for t in tPoints:
        cx_num, cx_den = 0, 0
        cy_num, cy_den = 0, 0
        n = bezLength - 1
        
        for i in range(bezLength):
            coeff = pascal[i]
            w = weights[i]
            v = vpath[i]
            
            bezWeight = (  coeff * ((1-t)**(n-i)) * (t**i) * w )
            cx_num += bezWeight * v[0]
            cx_den += bezWeight
            
            cy_num += bezWeight * v[1]
            cy_den += bezWeight
            # print(bezWeight, pascal)
            # print(cx_num, cx_den)
        c = (cx_num / cx_den, cy_num / cy_den)
        cPoints.append(c)
    
    return cPoints, tPoints
                
def Pascal(n):
    row = [1]
    for k in range(1, n+1):
        # Compute next value using binomial coefficient relation
        row.append(row[-1] * (n - k + 1) // k)
    return row

def ShowBSpline(cpoints, tpoints, goal, map_data, vertices, degree=3):
    fig, ax = plt.subplots()
    
    # --- Draw goal ---
    goalsq = patches.Rectangle(
        (goal[0][0], goal[0][1]),
        goal[1][0] - goal[0][0],
        goal[1][1] - goal[0][1],
        edgecolor=(0, 1, 0),
        facecolor=(0, 1, 0)
    )
    ax.add_patch(goalsq)

    # --- Draw obstacles ---
    for r in range(len(map_data)):
        for c in range(len(map_data[r])):
            if map_data[r][c] == 1:
                ax.plot(c, r, '.', color=(0, .5, .5))

    # --- Evaluate B-spline at tPoints ---
    xs = [p[0] for p in cpoints]
    ys = [p[1] for p in cpoints]
    
    tck, _ = splprep([xs, ys], k=degree, s=0)
    x_spline, y_spline = splev(tpoints, tck)
    
    # Convert to list of [x,y] points for plotting
    spline_points = list(zip(x_spline, y_spline))

    # --- Draw control points ---
    for cp in cpoints:
        ax.plot(cp[0], cp[1], 'o', color='red')

    # --- Draw control polygon (optional) ---
    for i in range(len(cpoints)-1):
        ax.plot(
            [cpoints[i][0], cpoints[i+1][0]],
            [cpoints[i][1], cpoints[i+1][1]],
            '--', color='red'
        )

    # --- Draw B-spline path ---
    for i in range(len(spline_points) - 1):
        p1 = spline_points[i]
        p2 = spline_points[i + 1]
        ax.add_patch(
            patches.FancyArrowPatch(
                p1, p2,
                arrowstyle='-',
                linewidth=2,
                color='purple'
            )
        )

    plt.show()
    
# Loads binary map from .npy file
def LoadBinaryMapFromFile(filename):
    file = filename + '.npy'
    binaryMap = np.load(file)
    return binaryMap

# Gets 2D slice from 3D binary map
def Get2DBinaryMapSlice(filename, z_index):
    binaryMap3D = LoadBinaryMapFromFile(filename)
    if z_index < 0 or z_index >= binaryMap3D.shape[2]:
        raise ValueError("z_index is out of bounds for the 3D binary map.")
    binaryMap2D = binaryMap3D[:, :, z_index]
    return binaryMap2D

# Extracts path from RRT edges
def FindPathFromRRT(E, start):
    icntr = 0
    #converts E one for x11 and y11, another for x12, y12
    E = np.array(E)
    VE1 = (E[:,:,0])
    VE2 = (E[:,:,1])

    ipath = -1
    Vpath = [[VE2[ipath,0]], [VE2[ipath,1]]]

    start2 = [start[0], start[1]]

    #matches the x11 and y11 with a x12 and y12
    while 0 != ipath and 0 != ipath and icntr < 1000:
        iplen = range(len(VE2))
        for j in iplen:
            if VE1[ipath][0] == VE2[j][0] and VE1[ipath][1] == VE2[j][1]:
                ipath = j
        Vpath[0].append(int(VE2[ipath,0]))
        Vpath[1].append(int(VE2[ipath,1]))
        icntr += 1
    Vpath[0].append(start2[0])
    Vpath[1].append(start2[0])

    return Vpath

def StackMap(layers):
    binaryMap = Get2DBinaryMapSlice('voxel_grid', 0)
    for i in range(1,layers-1):
        mapLayer = Get2DBinaryMapSlice('voxel_grid', i)
        for r in range(len(binaryMap)):
            for c in range(len(binaryMap[r])):
                if mapLayer[r][c] == 1:
                    binaryMap[r][c] = 1
    return binaryMap     

def VerifyMapSize(binaryMap, length):
    binaryMap = np.array(binaryMap)

    rows, cols = binaryMap.shape

    if cols < length:
        binaryMap = np.pad(binaryMap, ((0, 0), (0, length - cols)))
    elif cols > length:
        binaryMap = binaryMap[:, :length]

    if rows < length:
        binaryMap = np.pad(binaryMap, ((0, length - rows), (0, 0)))
    elif rows > length:
        binaryMap = binaryMap[:length, :]

    return binaryMap

def NormalizeAngles(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


def SplineToDistances(bpoints):
    distances = []
    angles = []

    for i in range(len(bpoints) - 1):
        x1, y1 = bpoints[i]
        x2, y2 = bpoints[i+1]

        dx = x2 - x1
        dy = y2 - y1

        dist = math.sqrt((dx*dx) + (dy*dy))
        distances.append(dist)

        theta = math.degrees(math.atan2(dy,dx))
        angles.append(theta)

    angle_change = []
    angle_change.append(angles[0])  # First angle
    for i in range(len(angles) - 1):
        diff = angles[i+1] - angles[i]
        diff = NormalizeAngles(diff)
        angle_change.append(abs(diff))

    return distances, angle_change

def MoveForward(distance, v, ip_addr):
    t = (0.05 * distance) / v
    command = ""
    
    start = time.time()
    while (time.time() - start) < t:
        pass
        # url = "http://" + ip_addr + "/js?json=" + command
        # response = requests.get(url)
        # content = response.text
        # print(content)
    # command = ""
    # url = "http://" + ip_addr + "/js?json=" + command
    # response = requests.get(url)

def Turn(angle, v, ip_addr):
    t = angle / v
    command = ""
    
    start = time.time()
    while (time.time() - start) < t:
        pass
        # url = "http://" + ip_addr + "/js?json=" + command
        # response = requests.get(url)
        # content = response.text
        # print(content)
    # command = ""
    # url = "http://" + ip_addr + "/js?json=" + command
    # response = requests.get(url)

def FollowSpline(path):
    print(f"\033[93mConnecting to Rover...\033[0m")
    # parser = argparse.ArgumentParser(description='Http JSON Communication')
    # parser.add_argument('ip', type=str, help='IP address: 192.168.10.104')

    # args = parser.parse_args()

    # ip_addr = args.ip
    ip_addr = 0
    print(f"\033[92mSuccessfully connected!\033[0m")
    
    distances, angles = SplineToDistances(path)
    v_lin = 0.5  # Linear velocity in m/s
    v_ang = 30   # Angular velocity in degrees/s
    print(f"\033[93mFollowing path...\033[0m")
    
    
    print(f"\033[94mCurrently at point {path[len(path)-1]}\033[0m")
    for i in range(len(distances)):
        Turn(angles[i], v_ang, ip_addr)
        MoveForward(distances[i], v_lin, ip_addr)
        print(f"\033[94mCurrently at point {path[len(path)-i-2]}\033[0m")
        
    print(f"\033[92mPath complete!\033[0m")

def CreatePath():
    # Initialize path info (hard coded)
    area = [ [0, 40], [0, 40] ]
    start = (0, 0)
    goal = [[30, 30], [32, 32]]
    
    # Get binary map from voxel
    print(f"\033[93mGenerating Map...\033[0m")
    stackedMap = StackMap(15)
    print(f"\033[93mVerifying Map Size...\033[0m")
    binaryMap = VerifyMapSize(stackedMap, 40)
    print(f"\033[92mMap created of length {len(binaryMap[0])} and width {len(binaryMap)}!\033[0m")
    
    # Create paths
    print(f"\033[93mAttempting to find path...\033[0m")
    path, cnct, vertices = GetPath(binaryMap, area, start, goal)
    if not cnct:
        print(f"\033[91mNo path found\033[0m")
        return
    print(f"\033[92mPath found!\033[0m")
    
    print(f"\033[93mGetting B-Spline Path\033[0m")
    bpoints, tpoints = GetBSpline(path, 100, binaryMap)
    print(f"\033[92mB-Spline Path Done.\033[0m")
    
    #Follow the spline path
    FollowSpline(bpoints)
    ShowBSpline(bpoints, tpoints, goal, binaryMap, vertices)

def main(args=None):
    CreatePath()
    
if __name__ == '__main__':
    main()