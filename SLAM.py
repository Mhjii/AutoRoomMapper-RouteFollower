import numpy as np
import SLAMObj
from numpy import array, transpose, dot, min, max, repeat
from numpy.linalg import inv
import random
import math


def lidar2cords(pos, ang, dist):
    point = [0, 0]
    point[0] = pos[0] + dist * np.sin(ang * np.pi / 180)
    point[1] = pos[1] + dist * np.cos(ang * np.pi / 180)
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
            w = linreg(samples)
            [indices, w, con_reached] = consensus(data, w, play, c)
            print(indices)
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
ransac([[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7],[8,7],[9,6],[10,5],[11,4],[12,3],[13,2],[14,1],[15,0]], ws=ranTest, s=2, play=3, c=1, num_outliers = 5, numcycles = 20)
print(ranTest)


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
