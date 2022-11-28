from robot import Robot 
from pathfind import *
from scipy.io import loadmat
import matplotlib.pyplot as plt
import numpy as np

def main():
    vars = loadmat("../map1.mat")
    map = vars['map']
    th = ThetaStar(map, filter=True, scale=3)
    points = th.plan((20,90),(50, 30))
    veh = Robot(animPath="../car.png", map=map, dim=100, animScale=6, x0=[20,90,0])
    x = []
    y = []
    for i in range(np.shape(map)[0]):    
        for j in range(np.shape(map)[1]):
    #x is j and y is i
            if map[i, j]:
                x.append(j)
                y.append(i)


    plt.scatter(x, y)
    plt.plot(20,90,'rx')
    plt.plot(50,30,'gx')
    plt.gca().set_xlim(0, 100)
    plt.gca().set_ylim(0, 100)
    for point in points:
        veh.go((point[0], point[1]))
    plt.pause(50)
    


if __name__ == "__main__":
    main()
