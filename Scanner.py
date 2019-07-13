import RethinkHelper as db
import SLAM
import numpy as np
import math
import random
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def randPoint(endpoints, noise):
    pt1 = endpoints[0]
    pt1x, pt1y = pt1
    pt2 = endpoints[1]
    pt2x, pt2y = pt2
    if pt1x == pt2x:
        y = random.uniform(pt1y, pt2y)
        x = pt1x + random.uniform(-noise, noise)

    elif pt1y == pt2y:
        x = random.uniform(pt1x, pt2x)
        y = pt1y + random.uniform(-noise, noise)
    else:
        x = random.uniform(pt1x, pt2x)
        y = x + random.uniform(-noise, noise) - 28
        x = x + random.uniform(-noise, noise)
    return x, y

def scan(points,position,maxR,):
    recScan = []
    x = []
    y = []
    scan = []
    for point in points:
        diff = np.subtract(point,position)
        dist = math.hypot(diff[0],diff[1])
        if dist < maxR:
            recScan.append(np.array(point))
            x.append(point[0])
            y.append(point[1])
    np.array(recScan)
    recScan.append(random.choice(points))
    # plt.scatter(x, y)
    # plt.xlim([position[0]-maxR, position[0]+maxR])
    # plt.ylim([position[1]-maxR, position[1]+maxR])
    # plt.axes().set_aspect('equal')
    # plt.show()
    for point in recScan:
        r = math.sqrt((point[0]-position[0])**2+(point[1]-position[1])**2)
        theta = math.atan2((point[1]-position[1]), (point[0]-position[0]))
        #theta += np.pi
        scan.append([theta, r])
    return scan


walls = [[[12, 48], [48, 48]], [[48, 20], [28, 0]], [[28, 0], [0, 0]],
         [[0, 36], [12, 36]], [[48, 48], [48, 20]], [[0, 0], [0, 36]], [[12, 36], [12, 48]],[[24,48],[24,24]],[[12,0],[12,25]],[[36,36],[36,8]]]

points = []
X = []
Y = []
pointsPerWall = 300
Range = db.getRange()
print(Range)

for wall in walls:
    for i in range(0, pointsPerWall):
        x, y = randPoint(wall, 0.1)
        X.append(x)
        Y.append(y)
        points.append([x, y])
# gs = gridspec.GridSpec(2, 2)
# fig1 = plt.figure()
# ax2 = fig1.add_subplot(gs[:,0])
# ax3 = fig1.add_subplot(gs[:,1],projection='polar')
# ax3.set_rticks([5,10,15,20,25])
# ax3.set_ylim(0,25)
# ax3.set_title('Axis [0,0]',y=1.16)

while 1:
        print("hey")
        db.getScanRequest()
        print("Scan Requested")
        db.disScanRequest()
        nextPos = db.getWaypoints()
        print(nextPos)
        time.sleep(0.2)
        pointCloud = scan(points,[nextPos[0],nextPos[1]],Range)
        theta = []
        r = []
        rectPoints = []
        x = []
        y = []
        for point in pointCloud:
            theta.append(point[1])
            r.append(point[0])
            pointpos = SLAM.polar2cords(nextPos, point[0], point[1])
            x.append(pointpos[0])
            y.append(pointpos[1])
            rectPoints.append(pointpos)
        # ax2.cla()
        # ax2.scatter(x,y)
        # ax3.cla()
        # ax3.scatter(r,theta)
        print("Point Cloud Generated")
        db.putPoints(pointCloud)
        print("Scan Complete")
        db.setScanComplete()
    # plt.pause(0.001)
