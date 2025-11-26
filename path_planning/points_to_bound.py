import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random as rand

def points_to_bound(obpts, obrad, bounds, b, de):

    #ignore points that lie inside a preexisiting bound
    for i in range(len(obpts[0])):
        obs_inside_bound = False
        xobs = obpts[0][i]
        yobs = obpts[1][i]
        zobs = obpts[2][i]

        if zobs >= 10:
            continue

        for j in range(len(bounds)):
            xmin = bounds[j][0][0]
            xmax = bounds[j][0][1]
            ymin = bounds[j][1][0]
            ymax = bounds[j][1][1]

            if xobs >= xmin and xobs <= xmax and yobs >= ymin and yobs <= ymax:
                obs_inside_bound = True
                break
        if obs_inside_bound:
            continue
        B = b + obrad[i]*de
        new_bound = [[xobs - B, xobs + B], [yobs - B, yobs + B]]
        bounds.append(new_bound)
    return bounds
    
def main():
    #cloud point filter provided by eugene
    #obstacle points
    obpts = [[],[],[]]
    num_points = len(obpts[0])
    obrad = [1.0] * num_points

    i = 0
    while i < 300:
        randx = rand.randint(0, 500)
        randy = rand.randint(0, 500)
        randz = rand.randint(0, 20) #the rover should only consider points within z = 0 ~ 10

        if randx < 450 and randy < 450:
            obpts[0].append(randx)
            obpts[1].append(randy)
            obpts[2].append(randz)
            i += 1

    bounds = []

    num_points = len(obpts[0])
    obrad = [1.0] * num_points

    start = [[500], [500]]

    points_to_bound(obpts, obrad, bounds, b=10, de=1)

    fig, ax = plt.subplots()

    plt.xlim(0, start[0][0])
    plt.ylim(0, start[1][0])
    ax.plot(obpts[0], obpts[1], 'o', color='red', markersize=1, zorder=3)

    # plot each bounding box
    for b in bounds:
        xmin = b[0][0]
        xmax = b[0][1]
        ymin = b[1][0]
        ymax = b[1][1]

        width = xmax - xmin
        height = ymax - ymin

        rect = patches.Rectangle(
            (xmin, ymin),
            width,
            height,
            edgecolor=(0, 0.75, 0.75),
            facecolor='none',
            linewidth=1
        )
        ax.add_patch(rect)

    plt.show()

if __name__ == "__main__":
    main()



