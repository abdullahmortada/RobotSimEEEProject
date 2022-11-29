from roboticstoolbox import LandmarkMap
from robot import Robot 
from pathfind import *
from scipy.io import loadmat
import matplotlib.pyplot as plt
import numpy as np

#defining the main function (works as a launcher for the robot)
def main():
    vars = loadmat("../map1.mat")
    map = vars['map']
    # th = ThetaStar(map, filter=True, scale=3)
    # points = th.plan((20,90),(50, 30))
    lmap = LandmarkMap(140, 100)
    veh = Robot(
            animPath="../car.png", map=map, 
            animScale=6,
            filter=True, filterScale=2,
            randMap=lmap, x0=[50, 30, 0],
            sensorRange=4
                )
    x = []
    y = []
    for i in range(np.shape(map)[0]):    
        for j in range(np.shape(map)[1]):
            #x is j and y is i
            if map[i, j]:
                x.append(j)
                y.append(i)

    #plotting the points for the path of the robot
    plt.scatter(x, y)
    lmap.plot()
    plt.gca().set_xlim(0, 100)
    plt.gca().set_ylim(0, 100)
    veh.planAndGo((20, 90))
    plt.pause(50)
    


if __name__ == "__main__":
    main()
