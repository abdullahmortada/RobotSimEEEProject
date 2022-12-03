# RobotSimEEEProject
This project is used to simulat a robot with a given target point and occupancy map to reach the ending point using fastest path and wihout colliding with any obstacles. Many tools are provided for both high level and low level contexts.

## roboticstoolbox
This project is based on Peter Corke's robotics toolbox for python.
roboticstoolbox provides tools and algorithms for designing, simulating, testing, and deploying manipulator and mobile robot applications.

## Project Structure
The source code is composed of two library files and a "main.py" file used as an example for how to use them.

### Robot.py
Center point of the project which includes a "Robot" class that encapsulates all the provided functionality to make everything easy to set up for brainstorming and demos, and is also a child class of the roboticstoolbox Bicycle class so includes all its functionality.

# usage
```python 
from robot import Robot

#load an occupancy grid from a matlab format file
from scipy.io import loadmat
map = loadmat("path/to/file")['map']

#initialize a robot instance
#only required parameter is occupancy grid, 
#if an empty map is needed initialize an empty numpy array
veh = Robot(map=map)
veh.go((x, y)) #moves to a given point

#uses the robot's specified pathplanner class to plan the shortest path through the map,
#and stores them in veh.points,
#if no pathplanner class from pathfind.py is passed during initialization the default ThetaStar class is used
veh.plan((startx, starty), (goalx, goaly)) 

#short hand function which plans the path and moves through all points
veh.planAndGo((goalx, goaly))


#a LandmarkMap can also be passed to the class to emulate unseen obstacles in a previously known environment.
#In this case a sensor is also initialized automatically and used to avoid these obstacles.
from roboticstoolbox import LandmarkMap 
rmap = LandmarkMap(140, 100)
veh = (
        #initialization options, 
        randMap=rmap
        )

#A map filter which makes obstacles seem bigger than reality to the pathplanner to force the robot to move further from walls is also available 
#by setting filter=True, and then filterScale to desired filter intensity.
#the robot also uses vertex detection by keeping track of the position of its vertices
```

#### pathfind.py
Contains all the calculations and classes used for pathfinding, and their associated methods.


           
