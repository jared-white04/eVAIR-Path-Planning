import math

def normalize_angle(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a

def angle_and_distance_computation(bpoints):
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
    for i in range(len(angles) - 1):
        diff = angles[i+1] - angles[i]
        diff = normalize_angle(diff)
        angle_change.append(abs(diff))

    return distances, angles, angle_change

if __name__ == "__main__":
    bpoints = [
        (0,0),
        (3,4),
        (3,9)
    ]

    distances, angles, angle_change = angle_and_distance_computation(bpoints)
    print("Distance:", distances)
    print("Angle:", angles)
    print("Angle Change:", angle_change)