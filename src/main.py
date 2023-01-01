from roboticstoolbox import LandmarkMap
from robot import Robot 
from pathfind import *
from scipy.io import loadmat
import matplotlib.pyplot as plt
import numpy as np
import sys

#Important note: "Before running the code you have to make sure that you are in the folder in the terminal."


def validatePos(map, x0, y0, xt, yt):
    if map[y0, x0]:
        print("Cannot start inside wall")
        exit()
    if map[yt, xt]:
        print("Target in wall")
        exit()
    return True

#defining the main function (works as a launcher for the robot).
def main(**kwargs):
    map = loadmat("./map1.mat")['map']

    x = []
    y = []

    #adding ordinates of obstacles in occupancy grid to separate arrays
    #x is j and y is i, as input array is transposed, very important to notice
    for i in range(np.shape(map)[0]):    
        for j in range(np.shape(map)[1]):
            if map[i, j]:
                x.append(j)
                y.append(i)

    
    #input coordinates
    x0, y0 = kwargs['start'].split(',')
    xt, yt = kwargs['goal'].split(',')
    x0 = int(x0)
    xt = int(xt)
    yt = int(yt)
    y0 = int(y0)

    validatePos(map, x0, y0, xt, yt)

    animScale = int(kwargs['animScale']) if 'animScale' in kwargs and len(kwargs['animScale']) > 0 else 6
    filter = True if 'filter' in kwargs and kwargs['filter'][0] == 't' else False
    filterScale = int(kwargs['filterScale']) if 'filterScale' in kwargs and len(kwargs['filterScale']) > 0 else 2
    obsDensity = int(kwargs['obstacleDensity']) if 'obstacleDensity' in kwargs and len(kwargs['obstacleDensity']) > 0 else 140
    lmap = LandmarkMap(obsDensity, 100) if 'randomObstacles' in kwargs and kwargs['randomObstacles'][0] == 't' else None 
    solver = BreadthFirst if 'solver' in kwargs and kwargs['solver'][0] == 't' else ThetaStar

    #initialize robot 
    veh = Robot(
            animPath="./car.png", map=map, 
            animScale=animScale, scaleRatio=1,
            filter=filter, filterScale=filterScale,
            x0=[x0, y0, 0], randMap=lmap, solver=solver, 
            sensorRange=4,
                )


    #plotting the map where the robot will be moving in.
    plt.scatter(x, y)
    plt.scatter(xt, yt, color='g')
    if lmap:
        lmap.plot()
    plt.gca().set_xlim(0, 100)
    plt.gca().set_ylim(0, 100)

    #plan and go to the target
    veh.planAndGo((xt, yt))
    plt.pause(5)
    


if __name__ == "__main__":
    main(**dict(arg.split('=') for arg in sys.argv[1:]))
