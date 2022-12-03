from math import atan2, cos, sin
from typing import Tuple, Union, List
from queue import PriorityQueue
from numpy import Infinity, shape, zeros

Point = Tuple[Union[float, int], Union[float, int]]
Interval = Point

class PathSolver:
    #defining the init function to run only once in the begining of the code    
    def __init__(self, grid, filter:bool=False, filterScale:int=1, vertInfo = None):
        self.grid = grid
        self.xlim = (0, shape(grid)[1] - 1)
        self.ylim = (0, shape(grid)[0] - 1)
        self.vertexInfo = vertInfo
        # self.vertices = []
        if filter:
            self.filterMap(filterScale)  


    #changing the scale of the obstacle by a fixed amount to make the robot move further away from the obstacles 
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
        

    def calcVertices(self, vertexInfo, angle):
        theta1 = vertexInfo[1] + angle 
        theta2 = vertexInfo[1] - angle 
        x = vertexInfo[0] * cos(theta1)
        y = vertexInfo[0] * sin(theta1)
        x2 = vertexInfo[0] * cos(theta2)
        y2 = vertexInfo[0] * sin(theta2)
        return [(x, y), (-x, -y), (x2, -y2), (-x2, y2)]


    #checking there is no obstacles in the LineOfSight (the robot moves from a point to point sucssesfully)
    def CheckLine(self, line: List[Point])-> bool:
        if self.vertexInfo:
            angle = atan2(line[-1][1] - line[0][1], line[-1][0] - line[0][0])
            vertices = self.calcVertices(self.vertexInfo, angle)

        for point in line:
            if self.grid[int(point[1]), int(point[0])]:
                return False
            if self.vertexInfo:
                for vert in vertices:
                    if self.grid[int(point[1] + vert[1]), int(point[0] + vert[0])]:
                        return False


        return True



    #draw a line between the points to navigate the robot correctly
    def LineOfSight(self, start: Point, goal: Point)-> List[Point]:
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

        if dx >= dy:
            while x0 != x1:
                f = f + dy

                if f >= dx:
                    result.append((x0+(sx-1)/2, y0+(sy-1)/2))
                    y0 = y0 + sy
                    f = f - dx
                if f != 0:
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


    def resetSelf(self):
        pass

    def plan(self):
        pass



#defining the ThetaSTar path finder method as a class
class ThetaStar(PathSolver):
    def __init__(self, grid, filter:bool = False, filterScale:int = 1, vertInfo = None):
        super().__init__(grid, filter, filterScale, vertInfo)
        self.gScore: dict = {}
        self.parent: dict = {}
        self.visited = set()
        self.open = PriorityQueue()


    #checking that the points are in queue (array like) to check the plan easily
    def pointInQ(self, point: Point)-> bool:
        for item in self.open.queue:
            if point in item:
                return True
        return False


    #Updating the score of the next point in the path
    def updateVertex(self, s: Point, neighbor: Point, goal: Point):
        if self.CheckLine(self.LineOfSight(self.parent[s], neighbor)):
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

        while self.parent[s] != s:
            total_path = [self.parent[s]] + total_path
            s = self.parent[s]
        return total_path


    #Tracing the plan that the robot will move in and checking that there is no obstacles in the path
    def plan(self, start: Point, goal: Point) -> List[Point]:
        self.gScore[start] = 0
        self.parent[start] = start

        self.open.put((self.heuristic(start, goal), start))

        while self.open:
            s = self.open.get()
            s = s[1]
            if s == goal:
                return self.makePath(s)

            self.visited.add(s)

            for neighbor in self.gridNeighbors(s):
                if self.grid[neighbor[1], neighbor[0]]:
                    continue
                if neighbor not in self.visited:
                    if not self.pointInQ(neighbor):
                        self.gScore[neighbor] = Infinity 
                        self.parent[neighbor] = None 
                    self.updateVertex(s, neighbor, goal)


    def resetSelf(self):
        self.gScore.clear()
        self.visited.clear()
        self.parent.clear()
        self.open.queue.clear()
            


