import numpy as np
import SLAMObj
from numpy import array, transpose, dot, min, max, repeat
from numpy.linalg import inv
import random
import math
from scipy.stats import gaussian_kde
import matplotlib.pyplot as plt


def lidar2cords(pos, ang, dist):
    point = [0, 0]
    point[0] = pos[0] + dist * np.sin(ang * np.pi / 180)
    point[1] = pos[1] + dist * np.cos(ang * np.pi / 180)
    return point


def polar2cords(pos, theta, dist):
    point = [0, 0]
    point[0] = pos[0] + dist * np.cos(theta)
    point[1] = pos[1] + dist * np.sin(theta)
    return point


def rec2polar(position,point):

    r = math.sqrt((point[0] - position[0]) ** 2 + (point[1] - position[1]) ** 2)
    theta = math.atan2((point[1] - position[1]), (point[0] - position[0]))
    return [r,theta]


def ransac(data, ws, s=10, play=2, c=200, num_outliers=100, numcycles=100):
    while data.__len__() > num_outliers:
        con_reached = False
        while not con_reached:
            random.shuffle(data)
            rand_sample = data.pop()
            i = 0
            samples = [rand_sample]
            while i < s:
                for cycles in range(numcycles):
                    samp = list(data.pop())
                    diff = np.subtract(rand_sample, samp)
                    dist = math.hypot(diff[0], diff[1])
                    np.delete(diff, 0)
                    if dist < play:
                        i += 1
                        samples.append(samp)
                    else:
                        data.insert(0, samp)
            #print(rand_sample)
            w = linreg(samples)
            [indices, w, con_reached] = consensus(data, w, play, c)
            if not con_reached:
                for j in samples:
                    data.append(j)
                print('consensus failed')
            else:
                data = [i for j, i in enumerate(data) if j not in indices]
                ws.append(w)
                print(ws)
                print('consensus Reached')
                print('num samples left')
                print(data.__len__())
    print('all samples associated')
    print(ws)
    return ws


def linreg(data):
    x_temp = []
    ones = np.ones(len(data))
    y_temp = []
    for i in data:
        x_temp.append(i[0])
        y_temp.append(i[1])
    x = array(x_temp)
    t = array(y_temp)
    x = np.column_stack((ones, x))
    xt = transpose(x)
    product = dot(xt, x)
    inverse = inv(product)
    w = dot(dot(inverse, xt), t)

    return w


def consensus(test, w, play, c=200):
    test = array(test)
    min_x = min(test[:, 0])
    max_x = max(test[:, 0])
    p1 = [w[0] + min_x * w[1], min_x]
    p2 = [w[0] + max_x * w[1], max_x]
    p2_p1 = np.subtract(p2, p1)
    test_p1 = np.subtract(test, p1)
    dists = np.cross(p2_p1, test_p1)
    consentors = []
    indices = []
    for j, i in enumerate(dists):
        if i < play:
            consentors.append(test[j])
            indices.append(j)

    if consentors.__len__() > c:
        print("number of consenters")
        print(consentors.__len__())
        print(indices)
        w = linreg(consentors)
        return indices, w, True
    else:
        return indices, w, False


ranTest = []


# ransac([[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7],[8,7],[9,6],[10,5],[11,4],[12,3],[13,2],[14,1],[15,0]], ws=ranTest, s=2, play=3, c=1, num_outliers = 5, numcycles = 20)
# print(ranTest)


def spike(data, spikes, maxdelta):
    n = data.__len__()
    spikepos = []
    for i in range(1, n - 1):
        delta1 = np.subtract(data[i], data[i - 1])
        delta2 = np.subtract(data[i], data[i + 1])
        hypot1 = np.hypot(delta1[0], delta1[1])
        hypot2 = np.hypot(delta2[0], delta2[1])
        diffTot = hypot1 + hypot2
        if diffTot > maxdelta:
            # spike detected
            spikepos.append(i)
    k = 0
    for j in spikepos:
        for i in spikes:
            blob = i.inblob(j)
            if not blob:
                spikes.append(SLAMObj.SpikeFeature(data[j - k]))
            del data[j - k]
            k += 1
    return spikes, data


def myround(x, base):
    return int(round(float(x) / base))


def mypointround(point, base):
    return

def gridConsensus(Map,consentors,wallThreshold):
    wallMap = np.zeros_like(consentors)
    for index,consentors in np.ndenumerate(consentors):
        if consentors > wallThreshold:
            wallMap[index] = 4
    for index,grid in np.ndenumerate(Map):
        if grid == 4:
            print('wall already discovered')
            wallMap[index] = 4
        if grid == 1:
            print('unreachable node')
            wallMap[index] = 1
    return wallMap



# def griddensity(points, oldMap, offset):
#     densemap = oldMap
#     for point in points:
#         [x, y] = point
#         x += offset[0]
#         y += offset[1]
#         set5 = [[x, y + 1], [x + 1, y], [x, y - 1], [x - 1, y]]
#         set25 = [[x, y + 2], [x + 1, y + 1], [x + 2, y], [x + 1, y - 1], [x, y - 2], [x - 1, y - 1], [x - 2, y],
#                  [x - 1, y + 1]]
#         for val in set5:
#             if val[1] <= (densemap.shape[1] - 1) and val[0] <= (densemap.shape[0] - 1) and val[1] >= 0 and val[0] >= 0:
#                 densemap[val[0], val[1]] += 5
#                 continue
#         for val in set25:
#             if val[1] <= (densemap.shape[1] - 1) and val[0] <= (densemap.shape[0] - 1) and val[1] >= 0 and val[0] >= 0:
#                 densemap[val[0], val[1]] += 2.5
#                 continue
#
#         densemap[x, y] += 10
#     return densemap

def griddensity(points, oldMap,radius, density, offset):
    densemap = oldMap
    for point in points:
        neighborIndex = np.zeros([radius * 2 + 1, radius * 2 + 1])
        possibleNeighbors = np.zeros([radius * 2 + 1, radius * 2 + 1, 2])

        possibleNeighbors[radius, radius] = point

        for index, neighbor in np.ndenumerate(neighborIndex):
            dist = np.sqrt((index[0] - radius) ** 2 + (index[1] - radius) ** 2)
            if dist < radius + np.sqrt(.25):
                possibleNeighbors[index] = [point[0] + index[0] - radius, point[1] + index[1] - radius]
                neighborIndex[index] = 1

        for index, val in np.ndenumerate(neighborIndex):
            if val == 1:
                x, y = possibleNeighbors[index]
                if y < (oldMap.shape[1]) and x < (oldMap.shape[0]) and y >= 0 and x >= 0:
                    densemap[int(x), int(y)] += density
                    continue
    return densemap

def pathdensity(points, oldmap, radius, offset = [0,0]):


    for point in points:
        neighborIndex = np.zeros([radius*2+1,radius*2+1])
        possibleNeighbors = np.zeros([radius*2+1,radius*2+1,2])

        possibleNeighbors[radius,radius]=point

        for index,neighbor in np.ndenumerate(neighborIndex):
            dist = np.sqrt((index[0]-radius)**2+(index[1]-radius)**2)
            if dist < radius+np.sqrt(.25):
                possibleNeighbors[index]=[point[0]+index[0]-radius,point[1]+index[1]-radius]
                neighborIndex[index]=1


        for index, val in np.ndenumerate(neighborIndex):
            if val == 1:
                x,y = possibleNeighbors[index]
                if  y < (oldmap.shape[1]) and x < (oldmap.shape[0]) and y >= 0 and x >= 0 and oldmap[int(x),int(y)]==0:
                    oldmap[int(x),int(y)] = 10
                    continue

    return oldmap

def nodeValid(point, oldmap, radius):

    neighborIndex = np.zeros([radius*2+1,radius*2+1])
    possibleNeighbors = np.zeros([radius*2+1,radius*2+1,2])

    possibleNeighbors[radius,radius]=point

    for index,neighbor in np.ndenumerate(neighborIndex):
        dist = np.sqrt((index[0]-radius)**2+(index[1]-radius)**2)
        if dist < radius+np.sqrt(.25):
            possibleNeighbors[index]=[point[0]+index[0]-radius+1,point[1]+index[1]-radius+1]
            neighborIndex[index]=1

    for index, val in np.ndenumerate(neighborIndex):
        if val == 1:
            x,y = possibleNeighbors[index]
            if  y > (oldmap.shape[1]) or x > (oldmap.shape[0]) or y < 0 or x < 0 or oldmap[int(x)-1,int(y)-1] == 1:
                return False
    return True


points = array([[2, 2], [1, 1], [3, 3], [3, 1], [1, 3]])


# map = griddensity(point)
def denseEnough(density, threshold):
    numnodes =  0
    x,y = density.shape
    totalnodes = x*y
    for index, density in np.ndenumerate(density):
        if density <= threshold:
            numnodes += 1
        continue

    percentcomplete=((numnodes / totalnodes) * 100)
    return percentcomplete


def gridassociation(data, resolution,buffer, gridMap, denseMap, mins):
    data = array(data)
    if len(data) == 0:
        return gridMap, denseMap, mins, True

    arrayround = np.vectorize(myround)

    data = arrayround(data, resolution)

    max_x = max(data[:, 0])
    min_x = min(data[:, 0])
    max_y = max(data[:, 1])
    min_y = min(data[:, 1])
    row, col = np.shape(gridMap)

    # print("Data set Mins/Maxes")
    # print([max_x, min_x, max_y, min_y])
    if min_x > mins[1]:
        min_x = mins[1]
    if min_y > mins[3]:
        min_y = mins[3]
    if max_x < mins[0]:
        max_x = mins[0]
    if max_y < mins[2]:
        max_y = mins[2]
    # print("Scan Mins/Maxes")
    # print([max_x, min_x, max_y, min_y])
    # print("Old scan Size")
    # print([row, col])
    x_offset = abs(min_x - mins[1])
    y_offset = abs(min_y - mins[3])
    # print([x_offset, y_offset])
    consentMap = np.zeros([(max_x - min_x + 1), (max_y - min_y + 1)])  # expand map
    # print("Map Size")
    # print(simpMap.shape)
    # print([x_offset,row,y_offset,col])
    # print(simpMap)
    # print(gridMap)
    # print(simpMap[x_offset:row,y_offset:col])
    consentMap[:row, :col] = gridMap  # insert old points
    consentMap = np.roll(consentMap, x_offset, 0)  # adjust for movement in the x
    consentMap = np.roll(consentMap, y_offset, 1)  # adjust for movement in the y

    newdense = np.zeros([max_x - min_x + 1, max_y - min_y + 1])
    newdense[:row, :col] = denseMap  # insert old points
    newdense = np.roll(newdense, x_offset, 0)  # adjust for movement in the x
    newdense = np.roll(newdense, y_offset, 1)  # adjust for movement in the y

    for point in data:
        consentMap[point[0] - min_x, point[1] - min_y] += 1

    for i in range(buffer):
        newdense = griddensity(data, newdense,i,0.25, [x_offset, y_offset])
    mins = [max_x, min_x, max_y, min_y]
    percentScanned = denseEnough(denseMap, 0)

    simpMap = gridConsensus(gridMap,consentMap,5)

    return consentMap,simpMap, newdense, mins, percentScanned


# testMap = gridassociation(
#     [[0, 0.1], [0, 0.3], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7], [8, 7], [9, 6], [10, 5], [11, 4],
#      [12, 3], [13, 2], [14, 1], [15, 0],], 1)


def heuristic(a, b):
    """a heuristic to return the new york distance between two points"""
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def dist(a,b):
    x = a[0]-b[0]
    y = a[1]-b[1]
    dist = math.sqrt(x**2+y**2)
    return dist


def pointAvg(a,b):
    x = a[0]+b[0]
    y = a[1]+b[1]
    point = [int(x/2),int(y/2)]
    return point
#pointAvg([-1,1],[1,1])

def pathSmoothing(path,pathsize):
    for index,node in enumerate(path):

        if index > pathsize-3:
            continue
        next = path[index+1]
        after = path[index+2]
        nextDist = dist(next,node)
        afterDist = dist(after,node)
        if nextDist*2>afterDist:
            path[index+1]=pointAvg(path[index],path[index+2])
    return path


#path = [[2,5],[3,6],[4,5],[5,6],[6,5],[7,6],[8,5],[9,4],[8,3],[9,2],[8,1],[9,0]]

#pathSmoothing(path,12)


def reconstruct_path(cameFrom, current, start):
    #print('Generating Path')
    total_path = [current]
    last_checked = current
    if current == start:
        return [current]
    numNodes = 1
    while True:
        current = cameFrom[int(last_checked[0]), int(last_checked[1])]
        current = [int(current[0]), int(current[1])]
        total_path.append(current)
        if np.all(current == start):
            break
        last_checked = current
        numNodes = numNodes+1
    total_path = pathSmoothing(total_path,numNodes)
    return total_path


def find_neighbors(point, map, wallThreshold):
    (x, y) = point
    neighborstest = [[x, y + 1], [x + 1, y + 1], [x + 1, y], [x + 1, y - 1], [x, y - 1], [x - 1, y - 1], [x - 1, y],
                     [x - 1, y + 1]]
    neighbors = []
    [max_x, max_y] = map.shape
    for neighbor in neighborstest:
        if neighbor[0] < 0 or neighbor[1] < 0 or neighbor[0] == max_x or neighbor[1] == max_y:
            #print(neighbor)
            #print("out of bounds")
            continue
        if map[neighbor[0], neighbor[1]] < wallThreshold:
            neighbors.append(neighbor)
            continue
        # print('out of bounds')
    return neighbors

# def find_neighbors(point, map, wallThreshold,radius):
#     (x, y) = point
#     neighborstest = [[x, y + 1], [x + 1, y + 1], [x + 1, y], [x + 1, y - 1], [x, y - 1], [x - 1, y - 1], [x - 1, y],
#                      [x - 1, y + 1]]
#     neighbors = []
#     [max_x, max_y] = map.shape
#     for neighbor in neighborstest:
#         if nodeValid(point,map,radius):
#             neighbors.append(neighbor)
#             continue
#         # print('out of bounds')
#     return neighbors


def simpNeighbors(point, map, wallThreshold):
    (x, y) = point
    neighborstest = [[x, y + 1], [x + 1, y], [x, y - 1], [x - 1, y]]
    neighbors = []
    [max_x, max_y] = map.shape
    for neighbor in neighborstest:
        if neighbor[0] < 0 or neighbor[1] < 0 or neighbor[0] == max_x or neighbor[1] == max_y:
            # print(neighbor)
            # print("out of bounds")
            continue
        if map[neighbor[0], neighbor[1]] < wallThreshold:
            neighbors.append(neighbor)
            continue
        # print('out of bounds')
    return neighbors


def dist_between(p1, p2):
    (x1, y1) = p1
    (x2, y2) = p2

    return np.sqrt(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2)


def astar(map, start, end, wallThreshold):
    if start == end:
        return "end reached"
    closedSet = []  # map of points already evaluated
    openSet = [start]  # map of points to be evaluated
    cameFrom = np.zeros([map.shape[0], map.shape[1], 2])  # empty map for path generation
    gScore = np.zeros_like(map)  # empty map for gscore calculation
    gScore.fill(np.inf)  # unevaluated points have infinate weight
    gScore[start] = 0  # start is 0 distace away from start
    fScore = np.zeros_like(map)  # map to store fscore values
    fScore.fill(np.inf)  # unevaluated points have infinate fscore
    fScore[start[0], start[1]] = heuristic(start, end)  # evaluate the distance from the start to the end
    # current = start
    while openSet:
        currentMinVal = np.inf  # set the min val to inf
        # print('new set')
        # print('open set')
        # print(openSet)
        # print('current Point')

        for (x, y) in openSet:  # for all points that havent been evaluatedin the open set
            f = fScore[x, y]  # find their associated fscore
            if f < currentMinVal:  # if the score is less than the current min
                currentMinVal = f  # set the current min score to the new score
                currentMin = [x, y]  # set the current min node to the current min
        current = currentMin  # set the current min value to the current node to be evaluated
        # current = list(np.unravel_index(np.argmin(fScore, axis=None), fScore.shape))
        # print(current)
        # print('end reached?')
        # print(np.all(current==end))
        if np.all(current == end):  # if we are at our end node
            print(cameFrom)
            path = reconstruct_path(cameFrom, end, start)  # reconstruct the path
            # print('path')
            # print(path)
            # print(fScore)
            return path  # return the path
        #print(current)
        openSet.remove(current)  # remove the current node from the open set
        closedSet.append(current)  # add it to the closed set
        neighbors = find_neighbors(current, map, wallThreshold)  # find all neighbors of the current
        for neighbor in neighbors:  # evaluate all neighbors
            # print(neighbor)
            if neighbor in closedSet:  # if the neighbor is in the closed set skip it
                # print("already evaluated")
                continue
            tempGscore = gScore[current[0], current[1]] + dist_between(current,
                                                                       neighbor)  # calculate a temporary gscore
            if tempGscore <= gScore[neighbor[0], neighbor[1]]:  # if the gscore is heigher than the current gscore
                # print("high g score")
                # print(tempGscore)
                # print(dist_between(current,neighbor))
                # print(gScore[current[0], current[1]])
                continue  # dont update score
            gScore[neighbor[0], neighbor[1]] = heuristic(neighbor, start)  # update gscore
            fScore[neighbor[0], neighbor[1]] = gScore[neighbor[0], neighbor[1]] + heuristic(neighbor,end)  # update fscore
            cameFrom[neighbor[0], neighbor[1]] = current  # camefrom value
            if neighbor not in openSet:  # if the  neighbor is not in the open set
                openSet.append(neighbor)  # add the neighbor
                # print(openSet)
    return []


def astardense(map, densemap, start, end):
    # print("grid Shape")
    # print(map.shape)
    print(start)
    closedSet = []  # map of points already evaluated
    openSet = [start]  # map of points to be evaluated
    cameFrom = np.zeros([map.shape[0], map.shape[1], 2])  # empty map for path generation
    gScore = np.zeros_like(map)  # empty map for gscore calculation
    gScore.fill(np.inf)  # unevaluated points have infinate weight
    gScore[start[0],start[1]] = 0  # start is 0 distace away from start
    fScore = np.zeros_like(map)  # map to store fscore values
    fScore.fill(np.inf)  # unevaluated points have infinate fscore
    fScore[start[0], start[1]] = heuristic(start, end) + densemap[
        start[0], start[1]]  # evaluate the distance from the start to the end
    current = start
    # print('new set')
    # print("end Node")
    # print(end)
    while openSet:
        currentMinVal = np.inf  # set the min val to inf

        # print('open set')
        # print(openSet)
        # print('current Point')
        for (x, y) in openSet:  # for all points that havent been evaluated in the open set
            f = fScore[x, y]  # find their associated fscore
            if f < currentMinVal:  # if the score is less than the current min
                currentMinVal = f  # set the current min score to the new score
                currentMin = [x, y]  # set the current min node to the current min

        current = currentMin  # set the current min value to the current node to be evaluated
        # current = list(np.unravel_index(np.argmin(fScore, axis=None), fScore.shape))
        #print(current)
        # print('end reached?')
        # print(np.all(current==end))
        if np.all(current == end):  # if we are at our end node
            # print(current)
            # print(end)
            # print(cameFrom)
            path = reconstruct_path(cameFrom, end, start)  # reconstruct the path
            # print('path')
            # print(path)
            # print(fScore)
            return path  # return the path
        openSet.remove(current)  # remove the current node from the open set
        closedSet.append(current)  # add it to the closed set
        neighbors = find_neighbors(current, map, 1)  # find all neighbors of the current
        # print(neighbors)
        for neighbor in neighbors:  # evaluate all neighbors
            if neighbor in closedSet:  # if the neighbor is in the closed set skip it
                # print("already evaluated")
                continue
            if densemap[neighbor[0],neighbor[1]]>20:
                continue
            tempGscore = gScore[current[0], current[1]] + dist_between(current,
                                                                       neighbor)  # calculate a temporary gscore
            if tempGscore >= gScore[neighbor[0], neighbor[1]]:  # if the gscore is heigher than the current gscore
                continue  # dont update score
            gScore[neighbor[0], neighbor[1]] = heuristic(neighbor, start)  # update gscore
            fScore[neighbor[0], neighbor[1]] = gScore[neighbor[0], neighbor[1]] + heuristic(neighbor, end) + densemap[
                neighbor[0], neighbor[1]]  # update fscore
            cameFrom[neighbor[0], neighbor[1]] = current  # camefrom value
            if neighbor not in openSet:  # if the  neighbor is not in the open set
                openSet.append(neighbor)  # add the neighbor
    # print(closedSet)
    path = reconstruct_path(cameFrom, closedSet.pop(), start)
    return path


def astarpathfind(map, start, end):
    closedSet = []  # map of points already evaluated
    openSet = [start]  # map of points to be evaluated
    cameFrom = np.zeros([map.shape[0], map.shape[1], 2])  # empty map for path generation
    gScore = np.zeros_like(map)  # empty map for gscore calculation
    gScore.fill(6400000)  # unevaluated points have infinate weight
    gScore[start] = 0  # start is 0 distace away from start
    fScore = np.zeros_like(map)  # map to store fscore values
    fScore.fill(6400000)  # unevaluated points have infinate fscore
    fScore[start[0], start[1]] = heuristic(start, end)  # evaluate the distance from the start to the end
    # current = start
    while openSet:
        currentMinVal = np.inf  # set the min val to inf
        # print('new set')
        # print('open set')
        # print(openSet)
        # print('current Point')

        for (x, y) in openSet:  # for all points that havent been evaluatedin the open set
            f = fScore[x, y]  # find their associated fscore
            if f < currentMinVal:  # if the score is less than the current min
                currentMinVal = f  # set the current min score to the new score
                currentMin = [x, y]  # set the current min node to the current min
        current = currentMin  # set the current min value to the current node to be evaluated
        # current = list(np.unravel_index(np.argmin(fScore, axis=None), fScore.shape))
        # print(current)
        # print('end reached?')
        # print(np.all(current==end))
        if np.all(current == end):  # if we are at our end node
            #print("end reached")
            return False
        # print(current)
        openSet.remove(current)  # remove the current node from the open set
        closedSet.append(current)  # add it to the closed set
        neighbors = simpNeighbors(current, map, 1)  # find all neighbors of the current
        for neighbor in neighbors:  # evaluate all neighbors
            if neighbor in closedSet:  # if the neighbor is in the closed set skip it
                # print("already evaluated")
                continue
            tempGscore = gScore[current[0], current[1]] + dist_between(current,
                                                                       neighbor)  # calculate a temporary gscore
            if tempGscore >= gScore[neighbor[0], neighbor[1]]:  # if the gscore is heigher than the current gscore
                continue  # dont update score

            gScore[neighbor[0], neighbor[1]] = heuristic(neighbor, start)  # update gscore
            fScore[neighbor[0], neighbor[1]] = gScore[neighbor[0], neighbor[1]] + heuristic(neighbor,
                                                                                            end)  # update fscore
            cameFrom[neighbor[0], neighbor[1]] = current  # camefrom value
            if neighbor not in openSet:  # if the  neighbor is not in the open set
                openSet.append(neighbor)  # add the neighbor
    return True


def travelDetect(Map,Density,Start):
    closedSet = []  # map of points already evaluated
    openSet = [Start]  # map of points to be evaluated

    while openSet:
        current = openSet.pop()
        closedSet.append(current)
        neighbors = simpNeighbors(current,Map,4)
        for neighbor in neighbors:
            if neighbor in closedSet:
                continue
            if neighbor not in openSet:
                openSet.append(list(neighbor))
    for index,node in np.ndenumerate(Map):
        if list(index) not in closedSet:
            if Map[index]!= 4:
                Map[index]=1
            Density[index] = np.inf
    return Map,Density


def diagDetect(Map):
    for index, node in np.ndenumerate(Map):
        if node == 0 and index[0] > 0 and index[1] > 0 and index[0]+1 < Map.shape[0] and index[1]+1< Map.shape[1]:
            if Map[index[0]-1,index[1]] == 1 and Map[index[0],index[1]-1] == 1 and Map[index[0]-1,index[1]-1] == 1:
                print(index)
                Map[index] = 1
            if Map[index[0] + 1, index[1]] == 1 and Map[index[0], index[1] - 1] == 1 and Map[
                    index[0] + 1, index[1] - 1] == 1:
                print(index)
                Map[index] = 1
            if Map[index[0] + 1, index[1]] == 1 and Map[index[0], index[1] + 1] == 1 and Map[
                    index[0] + 1, index[1] + 1] == 1:
                print(index)
                Map[index] = 1
            if Map[index[0] - 1, index[1]] == 1 and Map[index[0], index[1] + 1] == 1 and Map[
                    index[0] - 1, index[1] + 1] == 1:
                print(index)
                Map[index] = 1
    return Map


def enclosed(Map):
    x,y = Map.shape
    for i in np.concatenate((Map[0,:],Map[:,y-1],Map[x-1,:],Map[:,0])):
        if i == 0:
            return False
    return True

def unenclosed(Map):
    x,y = Map.shape
    unenclosed = []

    for i in range(x):
        if Map[i,0] == 0:
            unenclosed.append([i,0])
    for i in range(x):
        if Map[i,y-1] == 0:
            unenclosed.append([i,y-1])
    for i in range(y):
        if Map[0,i] == 0:
            unenclosed.append([0,i])
    for i in range(y):
        if Map[x-1,i] == 0:
            unenclosed.append([x-1,i])
    return unenclosed

def blockedPath(Map, start):
    #print("checking for blocked nodes")
    #Map = diagDetect(Map)
    for index, wall in np.ndenumerate(Map):
        if astarpathfind(Map, list(start), list(index)):
            #print("Blocked Node Discovered")
            Map[index] = 100
            continue
        else:
            Map[index] = 0
    return Map


def newHeading(gridmap, densemap, cameFrom,threshold, position, densefind):
    minpoint = [0, 0]
    minscore = np.inf
    mindensity = np.inf
    if densefind:
        for index, density in np.ndenumerate(densemap):

            if gridmap[index] <= threshold :
                if densefind:
                    if density != 0:
                        continue
                    distance = dist_between(index, position)
                    if distance > 30 or distance <4:
                        continue
                    if nodeValid(index, gridmap, 3):
                        continue
                    score = density+distance**2
                if score < minscore:
                    minscore = score
                    mindensity = density
                    minpoint = list(index)

    else:
        nodes = unenclosed(gridmap)
        print(nodes.__sizeof__())
        for node in nodes:
            distance = dist(position,node)
            if distance < minscore:
                minpoint = node
                minscore = distance
    print(minscore)
    return minpoint


def rayTrace(Map,Pos,threshold):
    max_x,max_y = Map.shape
    predicted = []
    invalid = []
    invalidRays = []
    tracedRays =[]
    rays = [[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0],[-1,1]]
    rayPos = np.zeros_like(rays)
    rayPos[:,0].fill(int(Pos[0]))
    rayPos[:,1].fill(int(Pos[1]))
    rayPos = list(rayPos)
    while rayPos:
        for index ,ray in enumerate(rays):
            rayPos[index] = [rayPos[index][0]+ray[0], rayPos[index][1]+ray[1]]
            if rayPos[index][0]==max_x or rayPos[index][1]==max_y or rayPos[index][0]<0 or rayPos[index][1]<0:
                invalid.append(rayPos[index])
                invalidRays.append(ray)
                continue
            if Map[rayPos[index][0],rayPos[index][1]]>=threshold:
                predicted.append(rayPos[index])
                tracedRays.append(ray)
                continue

        for pos in predicted:
            if pos in rayPos:
                rayPos.remove(pos)

        for pos in invalid:
            if pos in rayPos:
                rayPos.remove(pos)

        for ray in tracedRays:
            if ray in rays:
                #print("ray {} complete".format(ray))
                rays.remove(ray)

        for ray in invalidRays:
            if ray in rays:
                #print("ray {} complete".format(ray))
                rays.remove(ray)
    return predicted, tracedRays

def localize(Map,Pos,scanPoints,resolution):
    #rays at 90, 45, 0, 315, 270, 225, 180, and 135
    rays = [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]]
    validScans = []
    validationPoints = []
    predicted,tracedRays = rayTrace(Map,Pos)
    print(predicted)

    for index,ray in enumerate(tracedRays):
        if ray in rays:
            validScans.append(scanPoints[index])

    for point in validScans:
        validationPoints.append(polar2cords(Pos,point[1],point[0]))

    arrayround = np.vectorize(myround)
    print(validationPoints)
    validationPoints = arrayround(validationPoints, resolution)
    print(validationPoints)
    validationPoints -= predicted
    print(validationPoints)
    validationPoints = array(validationPoints)
    offset_x = np.average(validationPoints[:,0])
    offset_y = np.average(validationPoints[:,1])

    actual_position = [Pos[0] - offset_x,Pos[1] - offset_y]
    print(actual_position)

def initialize(Data,resolution):

    # scan 3 times to initalize

    minx = np.min(Data[:,0])
    miny = np.min(Data[:,1])
    minx = abs(myround(minx,resolution))
    miny = abs(myround(miny,resolution))
    currentPos = [minx,miny]

    return currentPos

