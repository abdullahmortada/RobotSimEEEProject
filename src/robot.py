from roboticstoolbox import Bicycle, VehicleIcon, RandomPath
from typing import Union
from math import atan2, pi
from pathfind import Point
import matplotlib.pyplot as plt

class Robot(Bicycle):
    def __init__(self, map, animPath:Union[str, None] = None, animScale=2, speed=3, tol=0.4, **kwargs): 
        """
        Child class of bicycle which provides premade functions for movement and pathfinding.
        Requires:
            map: 2d occupancy grid, map to be traversed
        Optionals:
            animPath: string, path to icon file for animation purposes
            animScale: int, animated icon's scale 
            speed: int or float, robot's velocity
            tol: float, tolerance for robot's goal position, smalled values are more accurate but
                values have to be proportional to animScale
        """
        anim = VehicleIcon(animPath, scale=animScale) if animPath else None
        super().__init__(animation=anim, control=RandomPath, **kwargs)
        if anim:
            self.init(plot=True)
        else:
            self.init()
        self._tolerance = tol
        self._speed = speed
        self._map = map

    def go(self, goal: Point):
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
            if self._plot:
                self._animation.update(self.x)
                plt.pause(0.005)
            if((abs(self.x[0] - goal[0]) < self._tolerance) and 
               (abs(self.x[1] - goal[1]) < self._tolerance)):
                break 

