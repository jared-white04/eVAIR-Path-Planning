import numpy as np

def compute_obstacle_velocities(Mobptss, t, rover_pose=None, Xpath=None):
    """
    Computes mean velocity of obstacles from position history.
    """

    Mobptss = [np.array(M) for M in Mobptss]
    t = np.array(t)

    velocities = []
    rel_velocities = []
    projected_velocities = []

    for k in range(len(t)-1):
        dt = t[k+1] - t[k]
        pos1 = Mobptss[k]
        pos2 = Mobptss[k+1]
        vel = (pos2 - pos1) / dt
        velocities.append(vel)

        # Relative velocity to rover
        if rover_pose is not None:
            rover_pose = np.array(rover_pose)
            rel_pos1 = pos1[:, :2] - rover_pose
            rel_pos2 = pos2[:, :2] - rover_pose
            rel_vel = (rel_pos2 - rel_pos1) / dt
            rel_velocities.append(rel_vel)

        # Projection onto path direction
        if Xpath is not None:
            # Approximate path tangent using finite difference
            px, py = Xpath[0], Xpath[1]
            path_dirs = np.diff(Xpath, axis=1)
            path_dirs = np.vstack([
                np.append(path_dirs[0], path_dirs[0][-1]),
                np.append(path_dirs[1], path_dirs[1][-1])
            ])
            norms = np.linalg.norm(path_dirs, axis=0)
            dirs = path_dirs / norms

            # Project obstacle velocities onto nearest path direction
            proj = []
            for p in pos1[:, :2]:
                # Find nearest path point
                dists = np.linalg.norm(Xpath.T - p, axis=1)
                idx = np.argmin(dists)
                path_dir = dirs[:, idx]
                proj.append(np.dot(vel[:, :2][0], path_dir))
            projected_velocities.append(proj)

    return {
        "velocities": velocities,
        "relative_velocities": rel_velocities if rover_pose is not None else None,
        "projected_velocities": projected_velocities if Xpath is not None else None
    }
