import numpy as np
import matplotlib.pyplot as plt
import RRT_Algorithm as rrt
from matplotlib import patches
from scipy.interpolate import splprep, splev

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
                currentDist = rrt.Euclidean(v, obstacle)
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

    # --- Draw vertices ---
    # for v in vertices:
    #     ax.plot(v[0], v[1], '.', color=(0, .75, .75))

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
    
    path, cnct, vert = rrt.GetPath(map1, area, start, goal)
    if cnct:
        print(f'\033[92mPath found.\033[00m') 
        rrt.ShowMap(path, goal, map1, vert)
        cpoints, tpoints = GetBSpline(path, 100, map1)
        ShowBSpline(cpoints, tpoints, goal, map1, vert)
    else:
        print(f'\033[91mNo path found.\033[00m')
        rrt.ShowMap(path, goal, map1, vert)
    
    
if __name__ == '__main__':
    main()