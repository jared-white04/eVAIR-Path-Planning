import matplotlib.pyplot as plt
import random as rand
import matplotlib.patches as patches

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

def PointsToBounds(objectPoints, bounds):
    boundExtent = 30
    
    for point in objectPoints:
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


def DrawGraph(objectPoints, bounds):
    fig, ax = plt.subplots()
    
    for i in range(len(bounds)):
        bound = bounds[i]
        point = objectPoints[i]
        
        ax.plot(point.x, point.y, 'o', color=(0, 0, 0))
        
        sLength = abs(bound[0][1] - bound[0][0])
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
    
    DrawGraph(objectPoints, bounds)
    
if __name__ == '__main__':
    main()