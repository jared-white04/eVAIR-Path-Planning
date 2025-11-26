import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import random as rand
import random
import RRTlearning

# --------------------------------------------------------
# RRT ALGORITHM
# --------------------------------------------------------
def RRT(maplim, start, goal, bounds, max_iter=5000, step=25):   # ⬅ bigger step = cleaner tree
    def sample_free():
        x = random.uniform(maplim[0][0], maplim[0][1])
        y = random.uniform(maplim[1][0], maplim[1][1])
        return np.array([x, y])

    def nearest(V, q_rand):
        dists = np.sum((V - q_rand.reshape(1, 2))**2, axis=1)
        return np.argmin(dists)

    def steer(q_near, q_rand, step):
        direction = q_rand - q_near
        dist = np.linalg.norm(direction)
        if dist < step:
            return q_rand
        return q_near + direction / dist * step

    def in_goal(q):
        return (goal[0][0] <= q[0] <= goal[0][1] and
                goal[1][0] <= q[1] <= goal[1][1])

    def collision(q1, q2):
        return RRTlearning.collision_free(
            [float(q1[0]), float(q1[1])],
            [float(q2[0]), float(q2[1])],
            bounds
        )

    # Initialize RRT
    V = np.array([start])
    E = []
    counter = 0
    reached = False

    for _ in range(max_iter):
        counter += 1
        q_rand = sample_free()
        idx = nearest(V, q_rand)
        q_near = V[idx]
        q_new = steer(q_near, q_rand, step)

        if collision(q_near, q_new):
            V = np.vstack([V, q_new])
            E.append([[q_near[0], q_new[0]], [q_near[1], q_new[1]]])

            if in_goal(q_new):
                reached = True
                break

    if reached:
        Vpath = RRTlearning.findpath(E, start.tolist())
        Vpath = np.array(Vpath)
    else:
        Vpath = np.array([[], []])

    return np.array(E), counter, Vpath, reached, V


# --------------------------------------------------------
# CLEAN MAP GENERATION
# --------------------------------------------------------

# --------------------------------------------------------
# MAP GENERATION
# --------------------------------------------------------

maplim = [[0, 500], [0, 500]]
goal = [[450, 500], [450, 500]]
start = [0, 0]

num_obstacles = 30  # match your friend's density

obpts = [[], []]

i = 0
while i < num_obstacles:
    randx = rand.randint(0, 500)
    randy = rand.randint(0, 500)

    # keep obstacles out of goal area
    if not (randx > 430 and randy > 430):
        obpts[0].append(randx)
        obpts[1].append(randy)
        i += 1

# Make each obstacle 30×30
bounds = []
for i in range(num_obstacles):
    bounds.append([
        [obpts[0][i] - 15, obpts[0][i] + 15],
        [obpts[1][i] - 15, obpts[1][i] + 15]
    ])



# --------------------------------------------------------
# RUN RRT
# --------------------------------------------------------
E, counter, Vpath, hcnct, V = RRT(
    maplim=np.array(maplim),
    start=np.array(start),
    goal=np.array(goal),
    bounds=bounds
)

print("\nConnected?:", hcnct)
print("Iterations:", counter)
print("Edges shape:", E.shape)
print("Path:", Vpath)


# --------------------------------------------------------
# PLOTTING — cleaner output
# --------------------------------------------------------

fig, ax = plt.subplots()

goalsq = patches.Rectangle((goal[0][0], goal[1][0]),
                           goal[0][1] - goal[0][0],
                           goal[1][1] - goal[1][0],
                           edgecolor='green',
                           facecolor='green',
                           alpha=0.7)
ax.add_patch(goalsq)

for b in bounds:
    obs = patches.Rectangle((b[0][0], b[1][0]),
                            b[0][1] - b[0][0],
                            b[1][1] - b[1][0],
                            facecolor='cyan',
                            edgecolor='cyan',
                            alpha=0.5)
    ax.add_patch(obs)

# Nodes (smaller)
for v in V:
    plt.plot(v[0], v[1], 'go', markersize=3)

# Edges (thinner)
for e in E:
    x = [e[0][0], e[0][1]]
    y = [e[1][0], e[1][1]]
    plt.plot(x, y, '-', color='gray', linewidth=0.3)

# Path
if hcnct:
    plt.plot(Vpath[0], Vpath[1], 'ro-', linewidth=2, markersize=5)

plt.plot(start[0], start[1], 'bo', markersize=7)

plt.grid(True, color='lightgray', linewidth=0.5)
plt.xlim(maplim[0])
plt.ylim(maplim[1])
plt.gca().set_aspect('equal')
plt.show()
