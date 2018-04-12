import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib import animation
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Button
import itertools
import SLAM
import math

def graph(w, x_range):
    x = np.array(x_range)
    y = x * w[1] + w[0]
    plt.plot(x, y)

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
        y = x + random.uniform(-noise/2, noise/2) - 28
        x = x + random.uniform(-noise/2, noise/2)
    return x, y

def scan(points,position,maxR):
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

def EStop(self):
    global Estop
    Estop = not Estop
    global stopButton
    print(Estop)
    if Estop:
        stopButton.label.set_text('Stop')
        return
    stopButton.label.set_text('Start')




walls = [[[12, 48], [48, 48]], [[48, 20], [28, 0]], [[28, 0], [0, 0]],
         [[0, 36], [12, 36]], [[48, 48], [48, 20]], [[0, 0], [0, 36]], [[12, 36], [12, 48]],[[24,48],[24,24]]]

points = []
X = []
Y = []
pointsPerWall = 300

roverRange = 10
scanResolution = 1
roverStart = [4,4]



roverPos = roverStart

# for wall in walls:
#     wall[0][0] -= 4
#     wall[0][1] -= 4
#     wall[1][0] -= 4
#     wall[1][1] -= 4

print(walls)
for wall in walls:
    for i in range(0, pointsPerWall):
        x, y = randPoint(wall, 0.05)
        X.append(x)
        Y.append(y)
        points.append([x, y])

Map = np.zeros([3,3])
Density = np.zeros([3,3])
Map.resize()
mins = [0,0,0,0]

keepgoing = True
Estop = False
path = []
minPoints = []
pathtest = []
travel = []
scanNumber = 1
tempr = []
tempt = []



gs = gridspec.GridSpec(2, 2)

fig1 = plt.figure()
ax1 = fig1.add_subplot(gs[0, 0])
ax1.imshow(Map,cmap=plt.cm.gray, interpolation='nearest', extent=[0,25,0,25])
ax2 = fig1.add_subplot(gs[0, 1])
ax2.imshow(Map,cmap=plt.cm.plasma, interpolation='nearest', extent=[0,25,0,25])
ax3 = fig1.add_subplot(gs[1, :],projection='polar')
ax3.set_rticks([1, 2, 3, 4, 5])
ax3.set_ylim(0,10)

buttonLoc = plt.axes([0.71, 0.01, 0.25, 0.25])
stopButton = Button(buttonLoc, "Stop")
stopButton.on_clicked(EStop)

plt.pause(5)

while keepgoing:
    if not Estop:
        print("Scan Number")
        print(scanNumber)
        pointCloud = scan(points, [roverPos[0]/scanResolution, roverPos[1]/scanResolution], roverRange)
        rectPoints = []
        theta = []
        r = []

        for point in pointCloud:
            theta.append(point[0])
            r.append(point[1])
            rectPoints.append(SLAM.polar2cords(roverPos, point[0], point[1]))
        ax3.cla()
        ax3.scatter(theta,r)
        ax3.scatter(0,0,marker="s")
        ax3.set_rticks([2, 4, 6, 8, 10])
        ax3.set_ylim(0, 10)
        Map,Density,mins,denseEnough = SLAM.gridassociation(rectPoints, scanResolution, Map, Density, mins)
        print("Dense Enough?")
        print(denseEnough)
        print("Map Enclosed?")
        enclosed = SLAM.enclosed(Map)
        print(enclosed)
        print("Keep Scanning?")
        print(not (enclosed  and denseEnough))
        keepgoing = not (enclosed and denseEnough)
        fig1.suptitle("Scan Number {}".format(scanNumber), fontsize=16)
        ax1.cla()
        ax1.set_title("Current Position: {}".format(roverPos))
        ax1.imshow(np.rot90(np.flip(Map,0),k=3),cmap=plt.cm.gray,origin='lower')
        ax1.scatter(roverPos[0], roverPos[1], marker='s',s=5,color='r')
        ax2.cla()
        if pathtest:
            ax2.set_title("Current Heading: {}".format(pathtest[0]))
        ax2.imshow(np.rot90(np.flip(Density,0),k=3),cmap=plt.cm.plasma,origin='lower')
        ax2.scatter(roverPos[0], roverPos[1], marker='s', s=5,color='r')
        # Note that using time.sleep does *not* work here!
        plt.pause(0.0001)
        repath = True
        for node in pathtest:
            if Map[node[0],node[1]] == 1:
                repath = True
                break
            repath = False
        if repath:
            print("Finding New Path")
            print("Current Position")
            print(roverPos)
            if path != []:
                Density = SLAM.griddensity(np.array(path), Density, [0, 0])
            for node in path:
                travel.append(node)
            path = []
            Heading = SLAM.newHeading(Map,Density,path)
            # print(roverPos)
            pathtest = SLAM.astardense(Map, Density, roverPos, Heading)
            # print(pathtest)
            minPoints.append(Heading)
            roverPos = pathtest.pop()
        roverPos = pathtest.pop()
        path.append(roverPos)
        print("Current Position")
        print(roverPos)

        Map,Density = SLAM.travelDetect(Map,Density, roverStart)
        # print(Map)
        scanNumber += 1



# Map,Density = SLAM.travelDetect(Map,Density, roverStart)
# path = SLAM.astardense(Map,Density,roverPos,roverStart)
# print(path)
print("left loop")

xpath = []
ypath = []


for i in travel:
    xpath.append(i[0])
    ypath.append(i[1])

xpath = np.array(xpath)
ypath = np.array(ypath)

plt.show()

plt.figure(2)

plt.imshow(np.rot90(Map),cmap=plt.cm.gray, interpolation='nearest',extent=[mins[1],mins[0],mins[3],mins[2]])
plt.scatter(roverStart[0],roverStart[1],marker='s')
plt.plot(xpath,ypath)
plt.show()

plt.figure(3)
plt.imshow(np.rot90(Density),cmap=plt.cm.plasma, interpolation='nearest',extent=[mins[1],mins[0],mins[3],mins[2]])
plt.scatter(minPoints[:][0],minPoints[:][1])
plt.plot(xpath,ypath)
plt.show()

#
# plt.scatter(X, Y)
# plt.plot(xpath*scanResolution,ypath*scanResolution)
# plt.scatter(roverStart[0], roverStart[1])
# plt.axis([-2, 14, -2, 14])
# plt.axes().set_aspect('equal')
# plt.show()