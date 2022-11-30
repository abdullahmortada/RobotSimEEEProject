from roboticstoolbox import Bicycle, VehicleIcon, RandomPath, LandmarkMap, RangeBearingSensor
from typing import Union, List
from math import atan2, cos, pi, sin
from pathfind import Point, ThetaStar
import matplotlib.pyplot as plt
import numpy as np

class Robot(Bicycle):
    def __init__(self, map, randMap: Union[LandmarkMap, None] = None,
                 sensorRange=2, sensorAngle:float=pi/4,
                 animPath:Union[str, None] = None, 
                 animScale=2, scaleRatio=1, speed=3, tol=0.4, 
                 solver = ThetaStar, filter:bool=False,
                 filterScale:int=1, **kwargs): 
        """
        Child class of bicycle which provides premade functions for movement and pathfinding.
        Requires:
            map: 2d occupancy grid, map to be traversed
        Optionals:
            randMap: LandmarkMap, map of obstacles that robot will sense during traversal
            sensorRange: int or float, max range of sensor 
            sensorAngle: float, angle in radians of sensor range
            animPath: string, path to icon file for animation purposes
            animScale: int, animated icon's scale 
            speed: int or float, robot's velocity
            tol: float, tolerance for robot's goal position, smalled values are more accurate but
                values have to be proportional to animScale
            solver: Any available PathPlanner class from this library, planning algorithm of choice
            filter and filterScale: bool and int, whether to filter at all and intensity of filter
        """
        anim = VehicleIcon(animPath, scale=animScale) if animPath else None
        super().__init__(animation=anim, control=RandomPath, **kwargs)
        if anim:
            self.init(plot=True)
        else:
            self.init()
        self._tolerance = tol
        self._speed = speed
        self.solver = solver(map, filter, filterScale)
        self.sensor = RangeBearingSensor(robot=self, map=randMap) if randMap else None
        self.sensorRange = sensorRange 
        self.sensorAngle = sensorAngle
        y = animScale / 2
        x = y * scaleRatio
        self.vertexInfo = ((y**2 + x**2)**0.5, pi/2 - atan2(y, x))
        self.vertices = []
        self.updateVertices()
        self.points: List[Point] = []


    def updateVertices(self):
        theta1 = self.vertexInfo[1] + self.x[2]
        theta2 = self.vertexInfo[1] - self.x[2]
        x = self.vertexInfo[0] * cos(theta1)
        y = self.vertexInfo[0] * sin(theta1)
        x2 = self.vertexInfo[0] * cos(theta2)
        y2 = self.vertexInfo[0] * sin(theta2)
        self.vertices = [(self.x[0] + x, self.x[1] +  y), (self.x[0] - x,self.x[1] - y), (self.x[0] + x2, self.x[1] - y2), (self.x[0] - x2, self.x[1] + y2)]
        self.solver.vertices = self.vertices
            
    

    def go(self, goal: Point):
        
        #Function which takes goal point's x and y as a tuple, and simulates robot's movement to that position
        while(True):
            g = atan2(
                    goal[1] - self.x[1],
                    goal[0] - self.x[0]
                    )
            steer = g - self.x[2]
            if steer > pi:
                steer = steer - 2*pi

            self.step(self._speed, steer)
            self.updateVertices()

            for vertex in self.vertices:
                if self.solver.grid[int(vertex[1]), int(vertex[0])]:
                    self.updateMap(vertex)
                    return False

            if self._plot:
                self._animation.update(self.x)
                plt.pause(0.005)
                
            if self.sensor:
                scanned = self.sensor.h(self.x)
                r = scanned[:, 0]
                index = np.where(r == min(r))
                
                if r[index] < self.sensorRange and abs(scanned[index, 1]) < self.sensorAngle:
                    coord = self.sensor.g(self.x, scanned[index][0])
                    if self.solver.grid[int(coord[1]), int(coord[0])] == 0:
                        self.updateMap(coord)
                        return False
                    

            if((abs(self.x[0] - goal[0]) < self._tolerance) and 
               (abs(self.x[1] - goal[1]) < self._tolerance)):
                return True 


    def plan(self, start: Point, goal: Point):
        self.points = self.solver.plan(start, goal)


    def updateMap(self, point:Point):
        point = (int(point[0]), int(point[1]))
        self.solver.grid[point[1], point[0]] = 1
        for neighbor in self.solver.gridNeighbors(point):
            self.solver.grid[neighbor[1], neighbor[0]] = 1

    
    def planAndGo(self, goal:Point):
        self.plan((int(self.x[0]), int(self.x[1])), goal)
        while True:
            go = False
            for point in self.points:
                go = self.go(point)
                if not go:
                    break

            if go:
                break
            else:
                self.solver.resetSelf()
                self.plan((int(self.x[0]), int(self.x[1])), goal)
