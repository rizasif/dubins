import math


class EucliHeur:

    Obstacles = [];
    Start = [];
    Goal = [];

    def __init__(self, start, goal, obstacles):
        self.Obstacles = obstacles
        self.Start = start
        self.Goal = goal

    def isInCollision(self, n):
        for ob in self.Obstacles:
            if ( self.getDistance(n, ob) <= ob[4] ):
                return True;
        return False;

    def getDistance(self, n1, n2):
        x = (math.pow((n1[2] - n2[2]), 2)) + (math.pow((n1[3] - n2[3]), 2));
        return math.sqrt(x);

    def getHeuristic(self, n):
        x = (math.pow((n[2] - self.Goal[2]), 2)) + (math.pow((n[3] - self.Goal[3]), 2));
        if (self.isInCollision(n)):
            return 2000;
        else:
            return math.sqrt(x);

    def getCost(self, n):
        x = (math.pow((n[2] - self.Start[2]), 2)) + (math.pow((n[3] - self.Start[3]), 2));
        return x;

    def getFixedCost(self):
        return 1;

