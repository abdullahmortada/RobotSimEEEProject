from math import atan2, cos, sin
from typing import Tuple, Union, List
from queue import PriorityQueue, Queue
from numpy import Infinity, shape, zeros

Point = Tuple[Union[float, int], Union[float, int]]
Interval = Point

class PathSolver:
    #defining the init function to run only once in the begining of the code    
    def __init__(self, grid, filter:bool=False, filterScale:int=1, vertInfo:List= []):
        self.grid = grid
        self.xlim = (0, shape(grid)[1] - 1)
        self.ylim = (0, shape(grid)[0] - 1)
        self.vertexInfo = vertInfo
        if filter:
            self.filterMap(filterScale)  


    #changing the scale of the obstacle by a fixed amount to make the robot avoid the obstacles. 
    def filterMap(self, scale: int):
        if scale < 1:
            print("Filter map aborted, invalid scale")
            return self.grid

        newGrid = zeros(shape(self.grid), int)
        for _ in range(scale):
            for i in range(self.xlim[1] - 1):
                newGrid[:, i] = (self.grid[:, i] + self.grid[:, i + 1])

            self.grid = newGrid

            for i in range(self.ylim[1] - 1):
                newGrid[i, :] = (self.grid[i, :] + self.grid[i + 1, :])

            self.grid = newGrid
        

    #calculates where the vertices of the robot will be ahead of time, so it can be considered while planning a path.
    def calcVertices(self, angle):
        theta1 = self.vertexInfo[1] + angle 
        theta2 = self.vertexInfo[1] - angle 
        x = self.vertexInfo[0] * cos(theta1)
        y = self.vertexInfo[0] * sin(theta1)
        x2 = self.vertexInfo[0] * cos(theta2)
        y2 = self.vertexInfo[0] * sin(theta2)
        return [(x, y), (-x, -y), (x2, -y2), (-x2, y2)]


    #checking there is no obstacles in the LineOfSight (the robot moves from a point to point sucssesfully), and vertices will not collide while following the line.
    def CheckLine(self, line: List[Point])-> bool:
        #calculate the supposed vertex positions if the robot were to pass through this line
        if self.vertexInfo:
            angle = atan2(line[-1][1] - line[0][1], line[-1][0] - line[0][0])
            vertices = self.calcVertices(angle)

        #for each point check if it has an obstacle, and then if any of the vertices will collide
        for point in line:
            if self.grid[int(point[1]), int(point[0])]:
                return False
            if self.vertexInfo:
                for vert in vertices:
                    if (self.ylim[0] < point[1] + vert[1] < self.ylim[1]) or not (self.xlim[0] < point[0] + vert[0] < self.xlim[1]):
                        if self.grid[int(point[1] + vert[1]), int(point[0] + vert[0])]:
                            return False


        return True



    #draw a line between two points, which will be used by checkline method 
    def LineOfSight(self, start: Point, goal: Point)-> List[Point]:
        #initialize starting values and intervals to be used
        x0, y0 = start
        x1, y1 = goal

        dy = y1 - y0
        dx = x1 - x0

        if dy < 0:
            dy = -dy
            sy = -1
        else:
            sy = 1

        if dx < 0:
            dx = -dx
            sx = -1
        else:
            sx = 1

        f = 0
        result = []

        #increment using y if dx is bigger, else using x
        if dx >= dy:
            while x0 != x1:
                f = f + dy

                if f >= dx:
                    result.append((x0+(sx-1)/2, y0+(sy-1)/2))
                    y0 = y0 + sy
                    f = f - dx
                if f != 0:
                    #there is a gradient so a second point at the same x value
                    result.append((x0+(sx-1)/2, y0+(sy-1)/2))
                if dy == 0:
                    a = (x0+(sx-1)/2, y0)
                    b = (x0+(sx-1)/2, y0-1)
                    result.extend((a, b))
                x0 = x0 + sx
        else:
            while y0 != y1:
                f = f + dx

                if f >= dy:
                    result.append((x0+(sx-1)/2, y0+(sy-1)/2))
                    x0 = x0 + sx
                    f = f - dy
                if f != 0:
                    result.append((x0+(sx-1)/2, y0+(sy-1)/2))
                if dx == 0:
                    a = (x0, y0+(sy-1)/2)
                    b = (x0-1, y0+(sy-1)/2)
                    result.extend((a, b))

                y0 = y0 + sy
        return result


    #manhattan distance (the value of x + the value of y)
    def heuristic(self, point: Point, goal: Point):
        return abs(goal[0] - point[0]) + abs(goal[1] - point[1])

    #the distance from a to b
    def euclidean(self, a: Point, b: Point)-> float:
        return ((a[1] - b[1])**2 + (a[0] - b[0])**2)**0.5


    #Finding the neighbors of the grid point
    def gridNeighbors(self, point: Point)-> List[Point]:
        res: List[Point] = []
        if point[0] > self.xlim[0]:
            res.append((point[0] - 1, point[1]))

        if point[1] > self.ylim[0]:
            res.append((point[0], point[1] - 1))

        if point[0] < self.xlim[1]:
            res.append((point[0] + 1, point[1]))

        if point[1] < self.ylim[1]:
            res.append((point[0], point[1] + 1))

        return res


    #shell methods that will be redefined polymorphically in child classes
    def resetSelf(self):
        pass

    def plan(self):
        pass



#defining the ThetaStar path finder method as a class
class ThetaStar(PathSolver):
    def __init__(self, grid, filter:bool = False, filterScale:int = 1, vertInfo = None):
        #initialized parent PathSolver class, then required hash tables and queues
        super().__init__(grid, filter, filterScale, vertInfo)
        self.gScore: dict = {}
        self.parent: dict = {}
        self.visited = set()
        self.open = PriorityQueue()


    #checking that the points are in the priority queue (array like)
    def pointInQ(self, point: Point)-> bool:
        for item in self.open.queue:
            if point in item:
                return True
        return False


    #Updating the score of the next point in the path, prioritizing the points parent as would provide a lower score
    def updatePoint(self, s: Point, neighbor: Point, goal: Point):
        #check if there is an unobstructed line of sight between the new point and the original point's parent
        #in that case the score and parent will ignore the old point and use its parent
        if self.CheckLine(self.LineOfSight(self.parent[s], neighbor)):
            #calculating the new score, and replacing it as the new score if it is more optimal than the existing one
            newG = self.gScore[self.parent[s]] + self.euclidean(self.parent[s], neighbor)
            if newG < self.gScore[neighbor]:
                self.gScore[neighbor] = newG 
                self.parent[neighbor] = self.parent[s]
                for item in self.open.queue:
                    if neighbor in item:
                        del item
                        break
                self.open.put((self.gScore[neighbor] + self.heuristic(neighbor, goal), neighbor))
        else:
            #same as the code before, but using the point and not it's parent, as there is no unobstructed path between the parent and the next point
            newG = self.gScore[s] + self.euclidean(s, neighbor)
            if newG < self.gScore[neighbor]:
                self.gScore[neighbor] = newG 
                self.parent[neighbor] = s 
                for item in self.open.queue:
                    if neighbor in item:
                        del item
                        break
                self.open.put((self.gScore[neighbor] + self.heuristic(neighbor, goal), neighbor))


    #Reconstruct the best path for the robot with no obstacles
    def makePath(self, s:Point)-> List[Point]:
        total_path:List[Point] = [s]

        #following the path from goal point till start using each node's parent node
        while self.parent[s] != s:
            total_path = [self.parent[s]] + total_path
            s = self.parent[s]
        self.resetSelf()
        return total_path


    #Planning the path that the robot will move in
    def plan(self, start: Point, goal: Point) -> List[Point]:
        self.gScore[start] = 0
        self.parent[start] = start

        self.open.put((self.heuristic(start, goal), start))

        #loop while there are unchecked points in priority queue
        while self.open:
            s = self.open.get()
            s = s[1]

            #return the reconstructed path if goal is reached
            if s == goal:
                return self.makePath(s)

            self.visited.add(s)

            #checking all the neighbors of current point
            for neighbor in self.gridNeighbors(s):
                #not processing neighbor if it contains an obstacle
                if self.grid[neighbor[1], neighbor[0]]:
                    continue

                #if point has not already been visited, its score or cost is updated, cost and parent are initialized if point is not already in the queue
                if neighbor not in self.visited:
                    if not self.pointInQ(neighbor):
                        self.gScore[neighbor] = Infinity 
                        self.parent[neighbor] = None 
                    self.updatePoint(s, neighbor, goal)


    #emptying all queues and hash tables for recalculation
    def resetSelf(self):
        self.gScore.clear()
        self.visited.clear()
        self.parent.clear()
        self.open.queue.clear()
            


#second solver class for demonstration of the differences between the two algorithms
class BreadthFirst(PathSolver):
    def __init__(self, grid, filter:bool = False, filterScale:int = 1, vertInfo = None):
        super().__init__(grid, filter, filterScale, vertInfo)
        self.frontier = Queue()
        self.parent = dict()


    def makePath(self, goal: Point) -> List[Point]:
        res = []
        current = goal 
        #trace back through the parent dictionary to reach start point from end point, reconstructing path along the way
        while self.parent[current] != None:
            res.insert(0, current)
            current = self.parent[current]

        res.insert(0, current)
        self.resetSelf()
        return res


    def plan(self, start: Point, goal: Point) -> List[Point]:
        self.frontier.put(start)
        self.parent[start] = None

        #go through the queue of points until it is empty.
        while not self.frontier.empty():
            current = self.frontier.get()
            #return reconstructed path if reached goal
            if current == goal:
                return self.makePath(goal)

            #check point neighbors
            for neighbor in self.gridNeighbors(current):
                if self.grid[neighbor[1], neighbor[0]]:
                    continue

                #make sure vertices dont collide with any obstacles at neighbor before adding it to queue
                if self.vertexInfo:
                    angle = atan2(neighbor[1] - current[1], neighbor[0] - current[0])
                    for vertex in self.calcVertices(angle):
                        if not (self.ylim[0] < neighbor[1] + int(vertex[1]) < self.ylim[1]) or not (self.xlim[0] < neighbor[0] + int(vertex[0]) < self.xlim[0]):
                                continue

                        if self.grid[neighbor[1] + int(vertex[1]), neighbor[0] + int(vertex[0])]:
                            continue

                #add neighbor if not already visited
                if neighbor not in self.parent:
                    self.frontier.put(neighbor)
                    self.parent[neighbor] = current

        return []

    def resetSelf(self):
        self.parent.clear()
        self.frontier = Queue()
