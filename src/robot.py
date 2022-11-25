from roboticstoolbox import Bicycle, VehicleIcon, RandomPath
from typing import Tuple, Union
from math import atan2, pi
import matplotlib.pyplot as plt

class Robot(Bicycle):
    def __init__(self, animPath:str, animScale=2, speed=3, tol=0.4, **kwargs): 
        anim = VehicleIcon(animPath, scale=animScale)
        super().__init__(animation=anim, control=RandomPath, **kwargs)
        self.init(plot=True)
        self._tolerance = tol
        self._speed = speed

    def go(self, 
           goal: Tuple[Union[int,float], Union[int,float]]):
        """
        Function which takes goal point's x and y as a tuple, and simulates robot's movement to that position
        """
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

