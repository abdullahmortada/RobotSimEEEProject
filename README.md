# RobotSimEEEProject
This project is used to simulate a robot with a given target point and occupancy map to reach the ending point using fastest path and wihout colliding with any obstacles. Many tools are provided for both high level and low level contexts.

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
#can also take a time value to try and reach the goal in the specified time
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
Classes:
    Parent PathPlanner class 
    ThetaStar class 
    BreadthFirst class 

The ThetaStar and BreadthFirst classes are implemented of their respective pathfinding algorithm, and the option to choose between them was to demonstrate the difference between some of the well known algorithms.

Breadth-First search uses a queue of points to check next and a dictionary of each visited point's parent point. When a point is taken from the queue, its unchecked neighbors are also placed in the queue, and this is repeated until the goal is reached, meaning it expands in all directions equally and does not use any scoring system which means there is many redundant iterations.

A* search uses both a heuristic to decide a point's priority in the queue, and a cost for each node that is constantly updated when more optimal parent nodes are found.
Theta* is an algorithm derived from A* which includes all its advantages, but also allows a point's parent to be at any angle and distance as long as there is a line of sight between them, providing less points to move to therefore more efficent paths and memory usage.
This paper provides more insight into other any-angle algorithms:
    https://ojs.aaai.org/index.php/SOCS/article/download/18382/18173/21898

In implemetation, the only difference between Theta* and A* is that Theta* prioritizes collapsing multiple nodes into just two nodes as long as there is a line of sight between all of the nodes.

## Limitations and known issues
When the robot's movement speed is set high relative to the tolerance and size of the robot, it moves in highly unexpected ways, happens especially when a low time condition is set, or when random obstacles cause robot to switch paths multiple times.

The robot also seems to be "colliding" with the wall but it is just a visual bug as the robot's position in the roboticstoolbox library is represented as a point. The vertex and map filter systems were both implemented to combat this problem and when both are combined it is greatly minimized, but improvements can still be made.

The algorithm is very resource intensive and even more so when random obstacles are placed, as the path is recalculated every time a new obstacle is detected. This causes may cause high run time on low powered devices.


