import RethinkHelper as db
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import SLAM
from numpy import pi

gs = gridspec.GridSpec(2, 2)

fig1 = plt.figure()

ax2 = fig1.add_subplot(gs[0, :])
ax2.set_aspect('equal')

ax3 = fig1.add_subplot(gs[1, :],projection='polar')
ax3.set_rticks([5,10,15,20,25])
ax3.set_ylim(0,25)
db.disScanRequest()

nodes = [[0,0],[10,0],[10,10]]
nodeNum = 0
while 1:

    db.putWaypoints([nodes[nodeNum][0], nodes[nodeNum][1]])
    nodeNum = nodeNum + 1
    if nodeNum > 2:
        nodeNum = 0
    print("Sending Next Node")
    db.setScanRequest()
    print("Requesting Scan")
    db.disScanComplete()
    db.getScanComplete()
    print("Scan Complete")
    R, Theta = db.getPoints()
    db.clearPoints()

    rectPoints = []
    theta = []
    r = []
    x = []
    y = []

    for index, rval in enumerate(R):
        point = SLAM.polar2cords([0,0], Theta[index]*(pi/180),rval)

        rectPoints.append(point)
        x.append(point[0])
        y.append(point[1])
        r.append(rval)
        theta.append(Theta[index]*(pi/180))
    print(Theta)
    print(theta)
    print(r)
    ax3.cla()
    ax3.scatter(theta,r)
    ax3.scatter(0, 0, marker="s")

    ax2.cla()
    ax2.scatter(x,y)
    ax2.scatter(0, 0, marker="s")
    plt.pause(10)
