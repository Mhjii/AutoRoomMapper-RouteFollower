import numpy as np
import RethinkHelper as db
import matplotlib.pyplot as plt
import SLAM

R,Theta = db.getPoints()
print(R)
print(Theta)

ax = plt.subplot(111, projection='polar')
ax.scatter(Theta, R)
ax.set_rmax(5)
ax.grid(True)

ax.set_title("Points from lidar grabbed off database", va='bottom')
plt.show()