import  numpy as np
from numpy import array, transpose,dot
from numpy.linalg import inv
import random
import math

def lidar2cords(pos,ang,dist):
    point=[0,0]
    point[0]=pos[0]+dist*np.sin(ang*np.pi/180)
    point[1] = pos[1] + dist * np.cos(ang * np.pi / 180)
    return point

def ransac(data,s=10,play=10,c=200,num_outliers=100,numcycles=100,ws):
    for n in repeat(none,numcycles):
        while data.__len__() >num_outliers:
            con_reached = False
            while not con_reached
                random.shuffle(data)
                rand_sample = data.pop()
                i=0
                samples = [rand_sample]
                while i < s:
                    samp = list(data.pop())
                    diff = np.subtract(rand_sample,samp)
                    dist = math.hypit(diff[0],diff[1])
                    np.delete(diff,0)
                    if dist<play:
                        i += 1
                        samples.append(samp)
                    else:
                        data.insert(0,samp)
                w = linreg(samples)
                con_reached = consensus(data,w,play,c)
                if not con_reached:
                    for j in samples:
                        data.append(j)
                    print('consensus failed')
                else:
                    ws.append(w)
                    print('consensus Reached')

def linreg(data):
    x_temp = []
    ones = np.ones(len(data))
    y_temp = []
    for i in data:
        x_temp.append(i[0])
        y_temp.append(i[1])
    X = array(x_temp)
    T = array(y_temp)
    X = np.column_stack((ones,X))
    Xt = transpose(X)
    product = dot(Xt,X)
    inverse = inv(product)

