from numpy import hypot

def lidar2cords(pos,ang,dist):
    point=[0,0]
    point[0]=pos[0]+dist*np.sin(ang*np.pi/180)
    point[1] = pos[1] + dist * np.cos(ang * np.pi / 180)
    return point

class Point:
    def __init__(self,posx,posy):
        self.x = posx
        self.y = posy

    def distance(self,point):
        return

class LIDARPoint(Point)
    """Points generated from Lidar Data"""
    def __init__(self,posx,posy,ang,dist):
        self.roverPos = Point(posx,posy)
        self.lidarAng = ang
        self.featureDist = dist
        self.pos = lidar2cords(self.roverPos,ang,dist)
        self.x,self.y = self.pos
        super(LIDARPoint,self).__init__(self.x,self.y)

