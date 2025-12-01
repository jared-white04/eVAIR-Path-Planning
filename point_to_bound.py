import matplotlib.pyplot as plt
import random as rand
import matplotlib.patches as patches
import math

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

def Euclidean(p1, p2):
    return (math.sqrt( (p1.x - p2.x)**2 + (p1.y - p2.y)**2 ))

def PointsToBounds(objectPoints, bounds):
    boundExtent = 30
    
    for point in objectPoints:
        pointInRange = False
        
        for box in bounds:
            center = Point((box[0][0] + box[0][1])/2, (box[1][0] + box[1][1])/2, 0)
            distance = Euclidean(point, center)
            # print(f'\033[94mDistance from ({point.x}, {point.y}) to ({center.x}, {center.y}) is {distance}\033[00m')
            if distance <= boundExtent:
                
                # print(f'Point ({center.x}, {center.y}) within {distance} of ({point.x}, {point.y})')
                pointInRange = True
                break
            
        if not pointInRange:
            print(f'\033[92mPoint ({point.x}, {point.y}) has no neighbors within 100\033[00m')
            x1 = point.x - boundExtent
            x2 = point.x + boundExtent
            y1 = point.y - boundExtent
            y2 = point.y + boundExtent
            bounds.append(((x1, x2),(y1, y2)))
        
def GeneratePoints(objectPoints, num):
    for i in range(num):
        randx = rand.randint(0,1000)
        randy = rand.randint(0,1000)
        randz = rand.randint(0,10)
        
        newPoint = Point(randx, randy, randz)
        objectPoints.append(newPoint)


def DrawGraph(bounds):
    fig, ax = plt.subplots()
    
    for box in bounds:
        point = Point((box[0][0] + box[0][1])/2, (box[1][0] + box[1][1])/2, 0)
        sLength = abs(box[0][1] - box[0][0])
        
        ax.plot(point.x, point.y, 'o', color=(0, 0, 0))
        
        boundBox = patches.Rectangle((point.x - sLength/2, point.y - sLength/2), sLength, sLength, edgecolor='red', facecolor='none')
        ax.add_patch(boundBox)
        
    ax.relim()
    ax.autoscale_view()
    
    plt.show()

def main(args=None):
    objectPoints = []
    GeneratePoints(objectPoints, 100)
    
    bounds = []
    PointsToBounds(objectPoints, bounds)
    
    DrawGraph(bounds)
    
if __name__ == '__main__':
    main()