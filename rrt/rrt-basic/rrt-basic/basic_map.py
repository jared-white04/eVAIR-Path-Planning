import matplotlib.pyplot as plt
import numpy as np
import random as rand
import matplotlib.patches as patches

#shapes replaced w/ obpts
#shapes1 replaced w/ bounds

def create_map():

    ### map ###
    #map 3 (4 squares)
    maplim = [[0, 500], [0, 500]]
    #goal and start
    goal = [[450, 500], [450, 500]]
    start = [[0], [0]]


    #obstacle points
    obpts = [[],[]]

    i = 0
    while i < 50:
        randx = rand.randint(0, 1000)
        randy = rand.randint(0, 1000)

        if randx < 450 and randy < 450:
            obpts[0].append(randx)
            obpts[1].append(randy)
            i += 1


    bounds = []


    #plotting goals and shapes
    # fig, ax = plt.subplots()
    # goalsq = patches.Rectangle((goal[0][0], goal[1][0]), (goal[0][1] - goal[0][0]), (goal[1][1] - goal[1][0]), edgecolor=(0, 1, 0), facecolor=(0, 1, 0))
    # ax.add_patch(goalsq)

    for i in range(len(obpts[0])):
        bounds.append([[obpts[0][i] - 20, obpts[0][i] + 20], [obpts[1][i] - 20, obpts[1][i] + 20]])

    # for i in range(len(obpts)):
    #     plt.plot(obpts[0], obpts[1],'.', color=(0, .5, .5))
    #     ax.add_patch(goalsq)

    # for bound in bounds:
    #     goalsq = patches.Rectangle((bound[0][0], bound[1][0]), (bound[0][1] - bound[0][0]), (bound[1][1] - bound[1][0]), edgecolor=(0, .75, .75), facecolor=(0, .75, .75))
    #     ax.add_patch(goalsq)
    ### map ###

    # plt.plot()

    # plt.show()
    
    # maplim = [ [x1, x2], [y1, y2] ], length & width of map
    # start = [x, y], coordinate of start
    # goal = [ [x1, x2], [y1, y2] ], x and y bounds of goal
    # bounds = [  [ [x1_1,x1_2], [y1_1, y1_2] ], [ [x2_1,x2_2], [y2_1, y2_2] ], ... [ [xi_1,xi_2], [yi_1, yi_2] ]  ]
    # bounds is an ARRAY of PAIRS of BOUNDS
    return maplim, start, goal, bounds