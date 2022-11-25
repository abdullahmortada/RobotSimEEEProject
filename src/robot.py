from roboticstoolbox import Bicycle, VehicleIcon 
from typing import Tuple, Union
from math import atan2, pi
import matplotlib.pyplot as plt

class robot(Bicycle):
    def __init__(self, iconPath, iconSize:int=2, 
                 pltDim=10, speed=3, tol=0.4,
                 l=1, steer_max=..., **kwargs): 
        self._anim = VehicleIcon(iconPath, scale=iconSize)
        Bicycle.__init__(self, l=l, steer_max=steer_max, animation= self._anim, dim=pltDim, **kwargs)
        self._tolerance = tol
        self._speed = speed
        self.init(plot=True)

    def go(self, 
           goal: Tuple[Union[int,float], Union[int,float]]):
        while(True):
            g = atan2(
                    goal[1] - self.x[1],
                    goal[0] - self.x[0]
                    )
            steer = g - self.x[2]
            if steer > pi:
                steer = steer - 2*pi

            self.step(self._speed, steer)
            self._animation.update(self.x)
            plt.pause(0.005)
            if((abs(self.x[0] - goal[0]) < self._tolerance) and 
               (abs(self.x[1] - goal[1]) < self._tolerance)):
                break 

