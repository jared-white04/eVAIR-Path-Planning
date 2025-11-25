import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random as rand


def ptb(obpts, bounds, obrad=[]):
    b = 5
    de = 0.5

    for indx in range(len(obpts[0])):
        x = obpts[0][indx]
        y = obpts[1][indx]

        B = b + de
        xn1 = x - B
        xn2 = x + B
        yn1 = y - B
        yn2 = y + B
        bounds.append([[xn1, xn2], [yn1, yn2]])



obpts = [[],[],[]]

i = 0
while i < 300:
    randx = rand.randint(0, 1000)
    randy = rand.randint(0, 1000)
    randz = rand.randint(0, 20)

    if randx < 450 and randy < 450:
        obpts[0].append(randx)
        obpts[1].append(randy)
        obpts[2].append(randz)
        i += 1

rad = [1,0.5,1]

plt.xlim([0, 500])
plt.ylim([0, 500])

bounds = []
ptb(obpts, bounds, rad)
ptb(obpts, bounds, rad)

fig, ax = plt.subplots()

for k in range(len(obpts[0])):
    bound = bounds[k]
    plt.plot(obpts[0][k], obpts[1][k], 'o', color=(0.3, 0.4, 0.3))
    length = bound[0][1] - bound[0][0]
    ob_sq = patches.Rectangle((obpts[0][k] - length/2, obpts[1][k] - length//2), length, length, edgecolor=(0, .75, .75), facecolor=(0, .75, .75))
    ax.add_patch(ob_sq)

plt.show()