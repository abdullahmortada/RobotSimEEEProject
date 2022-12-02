# RobotSimEEEProject
This project used to make a robot with given tarting point to acheieve the ending point using fastest path wihout colliding with any obstacles.
##roboticstoolbox
roboticstoolbox provides tools and algorithms for designing, simulating, testing, and deploying manipulator and mobile robot applications.

###Robot.py
used to make the robot and give to it some functions 
#usage
'class Robot(Bicycle)'  Child class of bicycle which provides premade functions for movement and pathfinding.


'__init__' giving the robot  map: 2d occupancy grid, map to be traversed
reuired and some optional fetures like
animPath: string, path to icon file for animation purposes
animScale: int, animated icon's scale 
speed: int or float, robot's velocity
tol: float, tolerance for robot's goal position, smalled values are more accurate but values have to be proportional to animScale

'go'  Function which takes goal point's x and y as a tuple, and simulates robot's movement to that position
####pathfind.py
used to let the robot know its path avoiding obstacles
#usage
'heuristic(self, point: Point)' calculate the manhattan distance (the value of x + the value of y)
'euclidean(self, a: Point, b: Point)->' calculate the distance from point a to point b
#####main.py
integrates all the functions
#usage
'map[i, j]'giving the exact location (x,y)of the robot


           
