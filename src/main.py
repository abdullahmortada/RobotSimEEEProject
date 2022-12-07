from roboticstoolbox import LandmarkMap
from robot import Robot 
from pathfind import *
from scipy.io import loadmat
import matplotlib.pyplot as plt
import numpy as np

#Important note: "Before running the code you have to make sure that you are in the folder in the terminal."


#defining the main function (works as a launcher for the robot).
def main():
    map = loadmat("./map1.mat")['map']
    lmap = LandmarkMap(140, 100)

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
    while True:
        try:
            x0 = int(input("Starting x:"))
            y0 = int(input("Starting y:"))
            xt = int(input("Target x:"))
            yt = int(input("Target y:"))
        except:
            print("Please try again.")
            continue

        if map[y0, x0]:
            print("Cannot start inside wall")
            continue
        break

    #initialize robot 
    veh = Robot(
            animPath="./car.png", map=map, 
            animScale=6, scaleRatio=1,
            filter=True, filterScale=2,
            x0=[x0, y0, 0], randMap=lmap, # solver=BreadthFirst, 
            sensorRange=4,
                )


    #plotting the map where the robot will be moving in.
    plt.scatter(x, y)
    lmap.plot()
    plt.gca().set_xlim(0, 100)
    plt.gca().set_ylim(0, 100)

    #plan and go to the target
    veh.planAndGo((xt, yt))
    plt.pause(1000)
    


if __name__ == "__main__":
    main()
