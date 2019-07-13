import math
import random
import time

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, CheckButtons, TextBox

import RethinkHelper as db

import SLAM



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
    #recScan.append(random.choice(points))
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
    print('Estop')
    if Estop:
        stopButton.label.set_text('Start')
        return
    stopButton.label.set_text('Stop')




walls = [[[12, 48], [48, 48]], [[48, 20], [28, 0]], [[28, 0], [0, 0]],
         [[0, 36], [12, 36]], [[48, 48], [48, 20]], [[0, 0], [0, 36]], [[12, 36], [12, 48]],[[24,48],[24,24]]]

points = []
X = []
Y = []
pointsPerWall = 300

roverRange = 10
scanResolution = 1
buffer = 4
completionPercent = 95
numConsentors = 15
roverStart = [5,5]



roverPos = roverStart
lastPos = [0,0]

# for wall in walls:
#     wall[0][0] -= 4
#     wall[0][1] -= 4
#     wall[1][0] -= 4
#     wall[1][1] -= 4

for wall in walls:
    for i in range(0, pointsPerWall):
        x, y = randPoint(wall, 0.05)
        X.append(x)
        Y.append(y)
        points.append([x, y])

consentMap = np.zeros([3,3])
Map = np.zeros([3,3])
Density = np.zeros([3,3])
mins = [0,0,0,0]

keepgoing = True
denseEnough = False
enclosed = False
userDenseEnough = False
userEnclosed = False
Estop = True
userPath = False
nearestFirst = True
initalize = False
Heading = [1,1]
headingTwoAgo = [0,0]
headingLast = [2,2]
path = []
minPoints = []
pathtest = []
travel = []
scanNumber = 1
travelDirection = 0
tempr = []
tempt = []

db.disScanRequest()

def addwalls(event):
    global Map
    global consentMap
    global Density
    global mins
    global denseEnough
    global pathtest
    global userPath
    global roverPos
    if event.inaxes == ax1:
        print("clicked wall plot")
        destination = [SLAM.myround(event.xdata, scanResolution) - 1, SLAM.myround(event.xdata, scanResolution) - 1]
        Map[destination]=1
        #consentMap, Map, Density, mins, denseEnough, percentScanned = SLAM.gridassociation([[event.xdata,event.ydata]], scanResolution, 10, consentMap, Density, mins)
    if event.inaxes == ax2:
        print("clicked density plot")
        destination = [SLAM.myround(event.xdata,scanResolution)-1,SLAM.myround(event.xdata,scanResolution)-1]
        if SLAM.nodeValid(destination,Map,5):
            pathtest = SLAM.astardense(Map, Density, roverPos, destination)
            print("user Path")
            print(pathtest)
            userPath = True



gs = gridspec.GridSpec(2, 2)

fig1 = plt.figure()
ax1 = fig1.add_subplot(gs[0, 0])
ax1.imshow(Map,cmap=plt.cm.gray, interpolation='nearest', extent=[0,25,0,25])
ax1.axis('off')
ax2 = fig1.add_subplot(gs[0, 1])
ax2.imshow(Map,cmap=plt.cm.plasma, interpolation='nearest', extent=[0,25,0,25])
ax2.axis('off')
ax3 = fig1.add_subplot(gs[1, :],projection='polar')
ax3.set_rticks([5,10,15,20,25])
ax3.set_ylim(0,25)
ax3.set_title('Axis [0,0]',y=1.16)

fig1.canvas.mpl_connect('button_press_event', addwalls)

def Initialize(self):
    global initalize,roverRange,buffer,completionPercent,numConsentors,scanResolution,roverPos

    roverRange = int(RangeInput.text)
    buffer = int(BufferInput.text)
    completionPercent = float(CompletionInput.text)
    numConsentors = int(ConsentorsInput.text)
    scanResolution = float(ResolutionInput.text)
    pointCloud = scan(points, roverPos, roverRange)
    rectPoints = []
    theta = []
    r = []
    for point in pointCloud:
        theta.append(point[0])
        r.append(point[1])
        rectPoints.append(SLAM.polar2cords([0, 0], point[0], point[1]))
    rectPoints = np.array(rectPoints)
    roverPos = SLAM.initialize(rectPoints, 1)
    initalize = True

    db.setSettings(buffer,completionPercent,numConsentors,roverRange,scanResolution,roverPos)

buttonLoc = plt.axes([0.71, 0.01, 0.22, 0.15])
stopButton = Button(buttonLoc, "Initialize")
stopButton.on_clicked(Initialize)

rax = plt.axes([0.71, 0.25, 0.22, 0.15])
check = CheckButtons(rax, ('Dense Enough', 'Enclosed'), (denseEnough, enclosed))

def checkbox(label):
    global denseEnough
    global enclosed
    global userDenseEnough
    global userEnclosed
    # if label == 'Dense Enough':
    #     denseEnough = check.get_status()[0]
    #     userDenseEnough = check.get_status()[0]
    #     print(denseEnough)
    # elif label == 'Enclosed':
    #     enclosed = check.get_status()[1]
    #     userEnclosed = check.get_status()[1]
    #     print(enclosed)

# def submitRange(text):
#     roverRange = int(text)
# def submitBuffer(text):
#     buffer = int(text)
# def submitPercent(text):
#     completionPercent = float(text)
# def submitConsent(text):
#     numConsentors = int(text)
# def submitResolution(text):
#     scanResolution = float(text)
check.on_clicked(checkbox)

RangeAx = plt.axes([0.18, 0.40, 0.12, 0.05])
RangeInput = TextBox(RangeAx,'Rover Range',initial='{}'.format(roverRange))
# RangeInput.on_submit(submitRange)
BufferAx = plt.axes([0.18, 0.32, 0.12, 0.05])
BufferInput = TextBox(BufferAx,'Obsticle Buffer',initial='{}'.format(buffer))
# BufferInput.on_submit(submitBuffer)
CompeltionAx = plt.axes([0.18, 0.24, 0.12, 0.05])
CompletionInput = TextBox(CompeltionAx,'% Completion',initial='{}'.format(completionPercent))
# CompletionInput.on_submit(submitPercent)
ConsentorsAx = plt.axes([0.18, 0.16, 0.12, 0.05])
ConsentorsInput = TextBox(ConsentorsAx,'Consentors/Grid',initial='{}'.format(numConsentors))
# ConsentorsInput.on_submit(submitConsent)
ResolutionAx = plt.axes([0.18, 0.08, 0.12, 0.05])
ResolutionInput = TextBox(ResolutionAx,'Scan Resolution',initial='{}'.format(scanResolution))
# ResolutionInput.on_submit(submitResolution)


while 1:
    if initalize:
        db.clearPoints()
        break
    plt.pause(0.1)
    continue
stopButton.label.set_text('Start')
stopButton.on_clicked(EStop)
notHome = True
while notHome:
    while keepgoing:
        if not Estop:
            print(roverPos)
            db.putWaypoints([roverPos[0], roverPos[1]])
            db.watchWaypoints()
            print("Sending Next Node")
            db.setScanRequest()
            print("Requesting Scan")
            db.disScanComplete()
            db.getScanComplete()
            print("Scan Complete")
            R,Theta = db.getPoints()
            db.clearPoints()
            #pointCloud = scan(points, [roverPos[0]/scanResolution, roverPos[1]/scanResolution], roverRange)
            #print(type(pointCloud))
            rectPoints = []
            theta = []
            r = []

            for index,rval in enumerate(R):
                rectPoints.append(SLAM.polar2cords(roverPos, rval, Theta[index]))
                r.append(rval)
                theta.append(Theta[index])
            ax3.cla()
            ax3.scatter(r,theta)
            ax3.scatter(0,0,marker="s")
            ax3.arrow(travelDirection, 0.5, 0, 5, alpha=0.5, width=0.1,
                             edgecolor='black', facecolor='black', lw=2, zorder=5)
            ax3.set_rticks([5,10,15,20,25])
            ax3.set_ylim(0, 10)
            Density = SLAM.pathdensity([roverPos],Density,buffer)
            percentScanned = 0
            if len(rectPoints) != 0:
                consentMap,Map,Density,mins,percentScanned = SLAM.gridassociation(rectPoints, scanResolution,buffer, consentMap, Density, mins)

            ax3.set_title("Percent\n Complete: \n {0:.2f}%".format(100-percentScanned), y=1.16)
            print("Dense Enough?")

            denseTemp = (100-percentScanned) > completionPercent
            denseEnough = denseTemp or check.get_status()[0]
            if denseEnough and not check.get_status()[0]:
                check.set_active(0)
            print(denseEnough)
            print("Map Enclosed?")
            Map, Density = SLAM.travelDetect(Map, Density, roverPos)
            enclosed = SLAM.enclosed(Map) or check.get_status()[1]
            if enclosed and not check.get_status()[1]:
                check.set_active(1)
            print(enclosed)
            print("Keep Scanning?")
            print(not (enclosed  and denseEnough))
            keepgoing = not (enclosed and denseEnough)
            fig1.suptitle("Scan Number {}".format(scanNumber), fontsize=16)
            ax1.cla()
            ax1.axis('off')
            ax1.set_title("Current Position: {}".format(roverPos))
            ax1.imshow(np.rot90(np.flip(Map,0),k=3),cmap=plt.cm.gray,origin='lower')
            ax1.scatter(roverPos[0], roverPos[1], marker='s',s=5,color='r')
            ax2.cla()
            ax2.axis('off')
            if pathtest:
                ax2.set_title("Current Heading: {}".format(pathtest[0]))
            ax2.imshow(np.rot90(np.flip(Density,0),k=3),cmap=plt.cm.plasma,origin='lower')
            ax2.scatter(roverPos[0], roverPos[1], marker='s', s=5,color='r')
            # Note that using time.sleep does *not* work here!
            plt.pause(0.001)
            pathsize = pathtest.__len__()
            for i in range(5):

                repath = True
                if not keepgoing:
                    break
                for index,node in enumerate(pathtest):
                    if node == Heading:
                        repath = True
                        break
                    if Map[node[0],node[1]] >= 1:
                        repath = True
                        break
                    # if SLAM.nodeValid(node,Map,5):
                    #     print("invalid node in path")
                    #     repath = True
                    #     break
                    if Density[node[0],node[1]]>20 and index == pathsize-1:
                        repath = True
                        break
                    repath = False
                if repath:
                    print("Finding New Path")

                    #if path != []:
                        #Density = SLAM.pathdensity(np.array(path), Density, 2)
                    for node in path:
                        travel.append(node)
                    path = []
                    headingTwoAgo = headingLast
                    headingLast = Heading
                    Heading = SLAM.newHeading(Map,Density,path,100,roverPos,True)
                    if headingTwoAgo == Heading:
                        nearestFirst = False
                    #print(Heading)a
                    # print(roverPos)
                    pathtest = SLAM.astardense(Map, Density, roverPos, Heading)
                    #print(pathtest)
                    minPoints.append(Heading)
                    roverPos = pathtest.pop()
                lastPos = roverPos
                roverPos = pathtest.pop()
                travelDirection = math.atan2(roverPos[1] - lastPos[1] , roverPos[0] - lastPos[0] )

                path.append(roverPos)
                fig1.suptitle("Scan Number {}".format(scanNumber), fontsize=16)
                ax1.cla()
                ax1.axis('off')
                ax1.set_title("Current Position: {}".format(roverPos))
                ax1.imshow(np.rot90(np.flip(Map, 0), k=3), cmap=plt.cm.gray, origin='lower')
                ax1.scatter(roverPos[0], roverPos[1], marker='s', s=5, color='r')
                ax2.cla()
                ax2.axis('off')
                if pathtest:
                    ax2.set_title("Current Heading: {}".format(pathtest[0]))
                ax2.imshow(np.rot90(np.flip(Density, 0), k=3), cmap=plt.cm.plasma, origin='lower')
                ax2.scatter(roverPos[0], roverPos[1], marker='s', s=5, color='r')
                # Note that using time.sleep does *not* work here!
                plt.pause(0.001)


                Density = SLAM.pathdensity([roverPos],Density,roverRange-3)


                # print(Map)
            scanNumber += 1
        plt.pause(0.001)
    print("Heading Home")
    pathHome = SLAM.astardense(Map,Density,roverPos,roverStart)
    print(pathHome)
    while 1:
        roverPos = pathHome.pop()
        db.putWaypoints([roverPos[0], roverPos[1]])
        ax2.cla()
        ax2.imshow(np.rot90(np.flip(Density, 0), k=3), cmap=plt.cm.plasma, origin='lower')
        ax2.scatter(roverPos[0], roverPos[1], marker='s', s=5, color='r')
        plt.pause(0.001)
        time.sleep(1)
        if roverPos == roverStart:
            notHome = False
            break



for node in path:
    travel.append(node)

# Map,Density = SLAM.travelDetect(Map,Density, roverStart)
# path = SLAM.astardense(Map,Density,roverPos,roverStart)
# print(path)
print("left loop")

denseEnough = SLAM.denseEnough(Density,.5)

xpath = []
ypath = []


for i in travel:
    xpath.append(i[0])
    ypath.append(i[1])

xpath = np.array(xpath)
ypath = np.array(ypath)

plt.show()



gs = gridspec.GridSpec(2, 2)
fig2 = plt.figure()
ax4 = fig2.add_subplot(gs[:,0])

ax4.imshow(np.rot90(Map),cmap=plt.cm.gray, interpolation='nearest',extent=[mins[1],mins[0],mins[3],mins[2]])
ax4.plot(xpath,ypath)


ax5 = fig2.add_subplot(gs[:,1])

ax5.imshow(np.rot90(Density),cmap=plt.cm.plasma, interpolation='nearest',extent=[mins[1],mins[0],mins[3],mins[2]])

def mapClick(event):
    global xpath
    global ypath
    if event.inaxes == ax4:

        print('you can add points here')
        Map[int(event.xdata), int(event.ydata)] = 4
        Density[int(event.xdata), int(event.ydata)] = np.inf
        ax4.cla()
        ax4.imshow(np.rot90(Map), cmap=plt.cm.gray, interpolation='nearest',
                   extent=[mins[1], mins[0], mins[3], mins[2]])
        ax4.plot(xpath, ypath)
    if event.inaxes == ax5:
        print('click here to set path')
        userPath = SLAM.astardense(Map,Density,[5,5],[int(event.xdata), int(event.ydata)])
        xUserPath = []
        yUserPath = []

        for i in userPath:
            xUserPath.append(i[0])
            yUserPath.append(i[1])

        xUserPath = np.array(xUserPath)
        yUserPath = np.array(yUserPath)
        print(xUserPath)
        ax5.cla()
        ax5.imshow(np.rot90(Density), cmap=plt.cm.plasma, interpolation='nearest',
                   extent=[mins[1], mins[0], mins[3], mins[2]])
        ax5.plot(xUserPath, yUserPath)
        print(userPath)
    plt.pause(0.1)
    print([event.xdata, event.ydata])

fig2.canvas.mpl_connect('button_press_event',mapClick)

plt.show()
#
# plt.scatter(X, Y)
# plt.plot(xpath*scanResolution,ypath*scanResolution)
# plt.scatter(roverStart[0], roverStart[1])
# plt.axis([-2, 14, -2, 14])
# plt.axes().set_aspect('equal')
# plt.show()