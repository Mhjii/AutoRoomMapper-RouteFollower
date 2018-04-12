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


def griddensity(points, oldMap, offset):
    max_x = max(points[:, 0])
    min_x = min(points[:, 0])
    max_y = max(points[:, 1])
    min_y = min(points[:, 1])
    # row, col = np.shape(oldMap)
    # densemap = np.zeros([max_x - min_x + 1, max_y-min_y + 1])
    # print(densemap)
    # densemap[:row,:col]=oldMap
    # print(densemap)
    # densemap = np.roll(densemap, offset[0], 0) # adjust for movement in the x
    # densemap = np.roll(densemap, offset[1], 1) # adjust for movement in the y
    # print(densemap)
    densemap = oldMap
    for point in points:
        [x, y] = point
        x += offset[0]
        y += offset[1]
        set5 = [[x, y + 1], [x + 1, y], [x, y - 1], [x - 1, y]]
        set25 = [[x, y + 2], [x + 1, y + 1], [x + 2, y], [x + 1, y - 1], [x, y - 2], [x - 1, y - 1], [x - 2, y],
                 [x - 1, y + 1]]
        for val in set5:
            if val[1] <= (densemap.shape[1] - 1) and val[0] <= (densemap.shape[0] - 1) and val[1] >= 0 and val[0] >= 0:
                densemap[val[0], val[1]] += .5
                continue
        for val in set25:
            if val[1] <= (densemap.shape[1] - 1) and val[0] <= (densemap.shape[0] - 1) and val[1] >= 0 and val[0] >= 0:
                densemap[val[0], val[1]] += .25
                continue

        densemap[x, y] += 1
    return densemap


points = array([[2, 2], [1, 1], [3, 3], [3, 1], [1, 3]])


# map = griddensity(point)
def denseEnough(density, threshold):
    numnodes =  0
    for index, density in np.ndenumerate(density):
        if density <= threshold:
            numnodes += 1

        continue
    if numnodes == 0:
        print("Scan Done")
        return True
    else:
        print("{} nodes remaining".format(numnodes))
        return False


def gridassociation(data, resolution, gridMap, denseMap, mins):
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
    simpMap = np.zeros([(max_x - min_x + 1), (max_y - min_y + 1)])  # expand map
    # print("Map Size")
    # print(simpMap.shape)
    # print([x_offset,row,y_offset,col])
    # print(simpMap)
    # print(gridMap)
    # print(simpMap[x_offset:row,y_offset:col])
    simpMap[:row, :col] = gridMap  # insert old points
    simpMap = np.roll(simpMap, x_offset, 0)  # adjust for movement in the x
    simpMap = np.roll(simpMap, y_offset, 1)  # adjust for movement in the y

    newdense = np.zeros([max_x - min_x + 1, max_y - min_y + 1])
    newdense[:row, :col] = denseMap  # insert old points
    newdense = np.roll(newdense, x_offset, 0)  # adjust for movement in the x
    newdense = np.roll(newdense, y_offset, 1)  # adjust for movement in the y

    for point in data:
        simpMap[point[0] - min_x, point[1] - min_y] = 1

    newdense = griddensity(data, newdense, [x_offset, y_offset])
    mins = [max_x, min_x, max_y, min_y]
    keepgoing = denseEnough(denseMap, 0)
    return simpMap, newdense, mins, keepgoing


# testMap = gridassociation(
#     [[0, 0.1], [0, 0.3], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7], [8, 7], [9, 6], [10, 5], [11, 4],
#      [12, 3], [13, 2], [14, 1], [15, 0],], 1)


def heuristic(a, b):
    """a heuristic to return the new york distance between two points"""
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def reconstruct_path(cameFrom, current, start):
    print('Generating Path')
    total_path = [current]
    last_checked = current
    if current == start:
        return [current]
    while True:
        current = cameFrom[int(last_checked[0]), int(last_checked[1])]
        current = [int(current[0]), int(current[1])]
        total_path.append(current)
        last_checked = current
        if np.all(current == start):
            break
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


def astar(map, start, end):
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
        neighbors = find_neighbors(current, map, 1)  # find all neighbors of the current
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
        neighbors = simpNeighbors(current,Map,1)
        for neighbor in neighbors:
            if neighbor in closedSet:
                continue
            if neighbor not in openSet:
                openSet.append(list(neighbor))
    for index,node in np.ndenumerate(Map):
        if list(index) not in closedSet:
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


def blockedPath(Map, start):
    print("checking for blocked nodes")
    #Map = diagDetect(Map)
    for index, wall in np.ndenumerate(Map):
        if astarpathfind(Map, list(start), list(index)):
            #print("Blocked Node Discovered")
            Map[index] = 1
            continue
        else:
            Map[index] = 0
    return Map


def newHeading(gridmap, densemap, cameFrom):
    minpoint = [0, 0]
    mindensity = np.inf
    for index, density in np.ndenumerate(densemap):
        if density < mindensity and gridmap[index] == 0:
            mindensity = density
            minpoint = list(index)
    print("new heading Density")
    print(mindensity)
    print("New Heading")
    print(minpoint)
    return minpoint
