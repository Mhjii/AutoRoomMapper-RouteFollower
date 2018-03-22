import json
import numpy as np
import matplotlib.pyplot as plt
import SLAM
import rethinkdb as r

#r.connect("localhost",29015).repl()

data = [
    {"PointNumber": "1", "R": 6, "angle": 90},
    {"PointNumber": "2", "R": 6.08, "angle": 80.54},
    {"PointNumber": "3", "R": 6.32, "angle": 71.57},
    {"PointNumber": "4", "R": 6.71, "angle": 63.43},
    {"PointNumber": "5", "R": 7.21, "angle": 56.31},
    {"PointNumber": "6", "R": 7.81, "angle": 50.19},
    {"PointNumber": "7", "R": 8.49, "angle": 45},
    {"PointNumber": "8", "R": 7.81, "angle": 39.81},
    {"PointNumber": "9", "R": 7.21, "angle": 33.69},
    {"PointNumber": "10", "R": 6.71, "angle": 26.57},
    {"PointNumber": "11", "R": 6.32, "angle": 18.43},
    {"PointNumber": "12", "R": 6.08, "angle": 9.46},
    {"PointNumber": "13", "R": 6, "angle": 0},
    {"PointNumber": "14", "R": 6.08, "angle": 350.54},
    {"PointNumber": "15", "R": 5.39, "angle": 338.20},
    {"PointNumber": "16", "R": 5, "angle": 323.13},
    {"PointNumber": "17", "R": 5, "angle": 306.87},
    {"PointNumber": "18", "R": 5.39, "angle": 291.80},
    {"PointNumber": "19", "R": 6.08, "angle": 279.46},
    {"PointNumber": "20", "R": 6, "angle": 270},
    {"PointNumber": "21", "R": 6.08, "angle": 260.54},
    {"PointNumber": "22", "R": 6.32, "angle": 251.57},
    {"PointNumber": "23", "R": 6.71, "angle": 243.43},
    {"PointNumber": "24", "R": 7.21, "angle": 236.31},
    {"PointNumber": "25", "R": 7.81, "angle": 230.19},
    {"PointNumber": "26", "R": 8.49, "angle": 225},
    {"PointNumber": "27", "R": 7.81, "angle": 219.81},
    {"PointNumber": "28", "R": 7.21, "angle": 213.69},
    {"PointNumber": "29", "R": 6.71, "angle": 206.57},
    {"PointNumber": "30", "R": 6.32, "angle": 198.43},
    {"PointNumber": "31", "R": 6.08, "angle": 189.46},
    {"PointNumber": "32", "R": 6, "angle": 180},
    {"PointNumber": "33", "R": 6.08, "angle": 170.54},
    {"PointNumber": "34", "R": 6.32, "angle": 161.57},
    {"PointNumber": "35", "R": 6.71, "angle": 153.43},
    {"PointNumber": "36", "R": 5.83, "angle": 149.04},
    {"PointNumber": "37", "R": 5, "angle": 143.13},
    {"PointNumber": "38", "R": 4.24, "angle": 135},
    {"PointNumber": "39", "R": 5, "angle": 126.87},
    {"PointNumber": "40", "R": 5.83, "angle": 120.96},
    {"PointNumber": "41", "R": 6.71, "angle": 116.57},
    {"PointNumber": "42", "R": 6.32, "angle": 108.43},
    {"PointNumber": "43", "R": 6.08, "angle": 99.46},
]

#dataJSOn =
#r.table("points").insert(data)

def graph(w,x_range):
    x = np.array(x_range)
    y = x*w[1]+w[0]
    plt.plot(x, y)

def lidar2cords(pos, ang, dist):
    point = [0, 0]
    point[1] = pos[0] + dist * np.sin(ang * np.pi / 180)
    point[0] = pos[1] + dist * np.cos(ang * np.pi / 180)
    return point


points = []
x = []
y = []

for point in data:
    rectPoint = lidar2cords([0, 0], point["angle"], point["R"])
    points.append(rectPoint)

for point in points:
    x.append(point[0])
    y.append(point[1])



ranTest = []
SLAM.ransac([[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7],[8,7],[9,6],[10,5],[11,4],[12,3],[13,2],[14,1],[15,0]], ws=ranTest, s=2, play=3, c=1, num_outliers = 5, numcycles = 20)

for w in ranTest:
    graph(w,[0,15])

plt.scatter([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15],[0,1,2,3,4,5,6,7,7,6,5,4,3,2,1,0])
plt.axis([0,15,0,10])
plt.show()

ws = []
#print('running ransac')
SLAM.ransac(points, ws, s=2, play=1.5, c=1, num_outliers = 5, numcycles = 20)
print(ws)
for w in ws:
    graph(w,[-8,8])

plt.scatter(x, y)
plt.axis([-8,8,-8,8])
plt.show()

