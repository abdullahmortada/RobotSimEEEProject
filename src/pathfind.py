from typing import Tuple, Union, List
from queue import PriorityQueue
from numpy import Infinity, shape

Point = Tuple[Union[float, int], Union[float, int]]
Interval = Point

class ThetaStar:
    def __init__(self, grid, goal: Tuple):
        self.goal = goal
        self.grid = grid
        self.xlim = (0, shape(grid)[1])
        self.ylim = (0, shape(grid)[0])
        self.gScore: dict = {}
        self.parent: dict = {}
        self.visited = set()
        self.open = PriorityQueue()



    def pointInQ(self, point: Point)-> bool:
        for item in self.open.queue:
            if point in item:
                return True
        return False


    def CheckLine(self, line: List[Point])-> bool:
        for point in line:
            if self.grid[int(point[1]), int(point[0])]:
                return False

        return True



    def LineOfSight(self, start: Point, goal: Point)-> List[Tuple]:
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

    def heuristic(self, point: Point):
        return abs(self.goal[0] - point[0]) + abs(self.goal[1] - point[1])
#the distance from a to b
    def euclidean(self, a: Point, b: Point)-> float:
        return ((a[1] - b[1])**2 + (a[0] - b[0])**2)**0.5


    def gridNeighbors(self, point: Point)-> List[Tuple]:
        res: List[Tuple] = []
        if point[0] > self.xlim[0]:
            res.append((point[0] - 1, point[1]))

        if point[1] > self.ylim[0]:
            res.append((point[0], point[1] - 1))

        if point[0] < self.xlim[1]:
            res.append((point[0] + 1, point[1]))

        if point[1] < self.ylim[1]:
            res.append((point[0], point[1] + 1))

        return res


    def updateVertex(self, s: Point, neighbor: Point):
        if self.CheckLine(self.LineOfSight(self.parent[s], neighbor)):
            newG = self.gScore[self.parent[s]] + self.euclidean(self.parent[s], neighbor)
            if newG < self.gScore[neighbor]:
                self.gScore[neighbor] = newG 
                self.parent[neighbor] = self.parent[s]
                for item in self.open.queue:
                    if neighbor in item:
                        del item
                        break
                self.open.put((self.gScore[neighbor] + self.heuristic(neighbor), neighbor))
        else:
            newG = self.gScore[s] + self.euclidean(s, neighbor)
            if newG < self.gScore[neighbor]:
                self.gScore[neighbor] = newG 
                self.parent[neighbor] = s 
                for item in self.open.queue:
                    if neighbor in item:
                        del item
                        break
                self.open.put((self.gScore[neighbor] + self.heuristic(neighbor), neighbor))


    def makePath(self, s:Point)-> List[Tuple]:
        total_path:List[Tuple] = [s]

        while self.parent[s] != s:
            total_path = [self.parent[s]] + total_path
            s = self.parent[s]
        return total_path


    def plan(self, start: Point):
        self.gScore[start] = 0
        self.parent[start] = start

        self.open.put((self.heuristic(start), start))

        while self.open:
            s = self.open.get()
            s = s[1]
            if s == self.goal:
                return self.makePath(s)

            self.visited.add(s)

            for neighbor in self.gridNeighbors(s):
                if self.grid[neighbor[1], neighbor[0]] == 1:
                    continue
                if neighbor not in self.visited:
                    if not self.pointInQ(neighbor):
                        self.gScore[neighbor] = Infinity 
                        self.parent[neighbor] = None 
                    self.updateVertex(s, neighbor)
            


