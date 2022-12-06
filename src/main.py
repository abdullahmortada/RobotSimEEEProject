from roboticstoolbox import LandmarkMap
from robot import Robot 
from pathfind import *
from scipy.io import loadmat
import matplotlib.pyplot as plt
import numpy as np

#Important note: "Before ruining the code you have to make sure that you are in the folder in the terminal."


#defining the main function (works as a launcher for the robot).
def main():
    vars = loadmat("./map1.mat")
    map = vars['map']
    lmap = LandmarkMap(140, 100)
    veh = Robot(
            animPath="./car.png", map=map, 
            animScale=6, scaleRatio=1,
            filter=True, filterScale=2,
            x0=[50, 30, 0], randMap=lmap, # solver=BreadthFirst, 
            sensorRange=4,
                )
    x = []
    y = []
    #x is j and y is i, as input array is transposed, very important to notice
    for i in range(np.shape(map)[0]):    
        for j in range(np.shape(map)[1]):
            if map[i, j]:
                x.append(j)
                y.append(i)

    #plotting the map where the robot will be moving in.
    plt.scatter(x, y)
    lmap.plot()
    plt.gca().set_xlim(0, 100)
    plt.gca().set_ylim(0, 100)
    
    #help the robot to reach the goal.
    veh.planAndGo((20, 90))
    plt.pause(1000)
    


if __name__ == "__main__":
    main()
