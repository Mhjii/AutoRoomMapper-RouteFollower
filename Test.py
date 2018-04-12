import SLAM
import matplotlib.pyplot as plt
import numpy as np

Map = np.zeros([4,4])

path = SLAM.astarpathfind(Map,[1,8],[8,1])
print(path)

for index, node in np.ndenumerate(Map):
    if index == [0,0]:
        continue
    print(list(index))
    path = SLAM.astarpathfind(Map,[0,0],list(index))
    print(path)

TestMap = SLAM.blockedPath(Map,list([0,0]))
print(TestMap)
