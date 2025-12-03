import numpy as np
import matplotlib.pyplot as plt
import random as rand
import time

#cornerfinder for binary occupancy maps
def cornerfinderBOM(map1):
    cnrs = [[],[]]
    #checks every point to find a 1
    for i in range(1, len(map1)-1):
        for j in range(1, len(map1[0])-1):
            cnrcheck = False

            if map1[i][j] == 1:
                #checks the 4 corners and edges of the "1" to find any 0's
                fcnrs = [[1,1],[-1,1],[-1,-1],[1,-1]]

                filfcnrs = []
                fcnrctr = 0

                #checks the 4 corners
                for fcnr in fcnrs:
                    if map1[i+fcnr[0]][j+fcnr[1]] == 0:
                        fcnrctr += 1
                        filfcnrs.append(fcnr)

                #checks if more than 3 of the 4 corners is a 0
                if fcnrctr > 2:
                    if map1[i][j+1] != map1[i][j-1] and (map1[i][j+1] == 0 or map1[i][j-1] == 0) :
                        cnrcheck = True
                    elif map1[i+1][j] != map1[i-1][j] and (map1[i+1][j] == 0 or map1[i-1][j] == 0):
                        cnrcheck = True

                if cnrcheck == True:
                    cnrs[0].append(j)
                    cnrs[1].append(i)

    return cnrs

def weightedbspline(Vpath, gapsize, res, obpts):
    #slicing the edges evenly

    newVpath = [[], []]
    #Vpathx's and y's
    xn = Vpath[0]
    yn = Vpath[1]

    for i in range(len(xn) - 1):
        length2 = ((xn[i] - xn[i+1])**2 + (yn[i] - yn[i+1])**2)**(1/2)
        if length2 >= gapsize * 2:
            xpt, ypt = xn[i], yn[i]
            newVpath[1].append(xpt)
            newVpath[0].append(ypt)
            n = int(length2/gapsize)
            dx = (xn[i] - xn[i+1])/(n + 1)
            dy = (yn[i] - yn[i+1])/(n + 1)
            for j in range(n):
                xpt, ypt = xpt-dx, ypt-dy
                newVpath[1].append(xpt)
                newVpath[0].append(ypt)
        else:
            xpt, ypt = xn[i], yn[i]
            newVpath[1].append(xpt)
            newVpath[0].append(ypt)

    xpt, ypt = xn[-1], yn[-1]
    newVpath[0].append(xpt)
    newVpath[1].append(xpt)

    n =  np.array(newVpath)
    newVpath = np.flip(newVpath)
    Vpath = newVpath

    #Belezier curve
    blen = len(Vpath[0])

    #path points
    tpts = np.linspace(0, 1, res)
    Cxpts = []
    Cypts = []

    #weights
    Ws = []

    #weights based on point length
    for i in range(blen):
        Wlen = ((Vpath[0][i] - obpts[0][0])**2 + (Vpath[1][i] - obpts[1][0])**2)
        for j in range(1, len(obpts[0])):
            Wlen1 = ((Vpath[0][i] - obpts[0][j])**2 + (Vpath[1][i] - obpts[1][j])**2)
            if Wlen1 < Wlen:
                Wlen = Wlen1

        W = (1/Wlen)**2
        Ws.append(W)

    #pascal's traingle (for Px constants)
    pas1 = [1,1]
    for i in range(blen - 2):
        pas2 = [1]
        for i in range(len(pas1)-1):
            pas2.append(pas1[i]+pas1[i+1])
        pas2.append(1)
        pas1 = pas2

    #Px and Py uses point P_i and an equation to create the Bezier curve, C
    for t in tpts:
        Cxn = 0
        Cyn = 0
        Cxd = 0
        Cyd = 0
        for i in range(blen):
            n = blen - 1

            bxW = (pas1[i]*((1-t)**(n - i))*(t**i))*Ws[i]
            Cxn += bxW*Vpath[0][i]
            Cxd += bxW

            byW = (pas1[i]*((1-t)**(n - i))*(t**i))*Ws[i]
            Cyn += byW*Vpath[1][i]
            Cyd += byW

        Cx = Cxn/Cxd
        Cy = Cyn/Cyd
        Cxpts.append(Cx)
        Cypts.append(Cy)

    return Cxpts, Cypts, Vpath

#collision check
def collision_freeBOM(p1, p2, map_matrix):

    x0, y0 = p1
    x1, y1 = p2

    steps = int(max(abs(x1 - x0), abs(y1 - y0)))
    for i in range(steps + 1):
        t = i / steps
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)


        xi = int(round(x))
        yi = int(round(y))


        # Check for obstacle
        if map_matrix[xi, yi] == 1:
            return False

    return True


#finding path
def findpath(E, start):
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


#RRT 1.3
def RRT13(map, start, goal):
    ### RRT ###
    #starting variables (counter and connect check)
    icntr = 0
    cnct = False
    #separate because it makes it easier to modify the map and RRT
    Qgoal = goal

    #G(V, E)
    #E = [[[x11,x12],[y11,y12]], [[x11,x12],[y11,y12]]]
    E = []
    #V = [[x1, x2...], [y1, y2...]]
    V = [[start[0]], [start[1]]]

    #While counter < lim and path isn't connected
    while icntr < 10000 and cnct == False:
        #Xnew  = RandomPosition()
        #[x1, y1]
        X = (rand.randrange(0, len(map), 1), rand.randrange(0, len(map[0]), 1))

        #if IsInObstacle(Xnew) == True:
            #continue
        Xvalid = True
        shapes1 = []

        xi = int(X[0])
        yi = int(X[1])

        #checks if there's already a point there
        for i in range(len(V[0])):
            if xi == V[0][i]:
                if yi == V[1][i]:
                    Xvalid = False
        #checks if point is on 1
        if map[xi, yi] == 1:
            Xvalid = False

        if Xvalid == True:
            #Xnearest = Nearest(G(V,E),Xnew) //find nearest vertex
            #setting the nearest vertex as vertex 1
            Vnrst = (V[0][0], V[1][0])
            Vdist1 = ((V[0][0] - X[0])**2 + (V[1][0] - X[1])**2)**(1/2)
            #comparing the vertex distances to determine the closest vertex
            for i in range(len(V[0])):
                Vdist2 = ((V[0][i] - X[0])**2 + (V[1][i] - X[1])**2)**(1/2)
                if Vdist2 < Vdist1:
                    Vnrst = (V[0][i], V[1][i])
                    Vdist1 = Vdist2

            #Link = Chain(Xnew,Xnearest)
            #G.append(Link)
            #[[[x11,x12], [y11,y12]]
            #E.append([[Vnrst[0],X[0]], [Vnrst[1],X[1]]])
            #V[0].append(X[0])
            #V[1].append(X[1])
            if Xvalid and collision_freeBOM(Vnrst, X, map):
                E.append([[Vnrst[1], X[1]], [Vnrst[0], X[0]]])
                V[1].append(X[1])
                V[0].append(X[0])

                #if Xnew in Qgoal:
                    #Return G
                if X[1] >= Qgoal[0][0] and X[0] >= Qgoal[1][0]:
                    if X[1] <= Qgoal[0][1] and X[0] <= Qgoal[1][1]:
                        cnct = True

        #iteration
        icntr += 1

    ### RRT ###


    ### tracing the path ###
    Vpath = findpath(E, start)
    ### tracing the path ###

    return E, V, Vpath, icntr, cnct


#multiple iteration RRT
def RRT_multi_iter_BOM(map, start, goal, iter):
    Es = []
    Vpaths = []
    slengths = []
    cncts = []
    for i in range(iter):
        E, V, Vpath, icntr, cnct = RRT13(map, start, goal)
        Vpaths.append(Vpath)
        cncts.append(cnct)

        slength = 0
        if cnct == True:
            for i in range(len(Vpath[0])-1):
                ds = ((Vpath[0][i] - Vpath[0][i+1])**2 + (Vpath[1][i] - Vpath[1][i+1])**2)**(1/2)
                slength += ds
        slengths.append(slength)
        Es.append(E)

    n = i

    cnct = False
    if len(slengths) > 1:
        minslength = min(slengths)
        idx = slengths.index(minslength)

        cnct = cncts[idx]
        Vpath = Vpaths[idx]
        E = Es[idx]

        return E, Vpath, cnct

    return 0, 0, cnct


'''map1 = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,1,1,1,1,0,0,1,1,0,1,1,1],
        [0,0,0,0,0,0,0,0,1,0,0,1,0,0,1,0,0,0,0,0],
        [1,1,1,1,0,1,1,1,1,0,0,1,1,1,1,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]])'''

map1 = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [0,0,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]])

start = [0, 0]
goal = [[18, 20], [8, 10]]

for i in range(len(map1)):
    for j in range(len(map1[i])):
        if map1[i][j] == 1:
            plt.plot(j, i, '.', color=(0,1,0))

cnrs = cornerfinderBOM(map1)

'''t1 = time.time()

E, Vpath1, cnct = RRTBOM.RRT_multi_iter_BOM(map1, start, goal, 5)

t2 = time.time()

Cxpts, Cypts, Vpath1 = spline.weightedbspline(Vpath1, 2, 100, cnrs)
Vpath1 = np.array([Cxpts, Cypts])

length = 0
for i in range(len(Vpath1[0]) - 1):
    length += ((Vpath1[0][i] - Vpath1[0][i+1])**2 + (Vpath1[1][i] - Vpath1[1][i+1])**2)

plt.plot(Cxpts, Cypts, color=(1,0,0))

print("cnct: ", cnct)
print(length)
print(t2-t1)'''

t1 = time.time()

E, Vpath2, cnct = RRT_multi_iter_BOM(map1, start, goal, 20)

t2 = time.time()

plt.plot(Vpath2[0], Vpath2[1], color=(1,0,0))

Cxpts, Cypts, Vpath2 = weightedbspline(Vpath2, 2, 100, cnrs)
Vpath2 = np.array([Cxpts, Cypts])

length = 0
for i in range(len(Vpath2[0]) - 1):
    length += ((Vpath2[0][i] - Vpath2[0][i+1])**2 + (Vpath2[1][i] - Vpath2[1][i+1])**2)

#for Es in E:
    #plt.plot(Es[0], Es[1], color=(1,0,0))

plt.plot(Vpath2[0], Vpath2[1], color=(0,0,1))

print("cnct: ", cnct)
print(length)
print(t2-t1)

plt.show()