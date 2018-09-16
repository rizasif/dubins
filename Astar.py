from algorith_abstract import Algorithm
from euclidian_heuristic import EucliHeur
import math
import numpy as np

class AStar(Algorithm):
    # Assumption: state = (id,prim,x,y,r,F,g)

    #Static Flags
    SOLUTION_FOUND = 0
    GOAL_FOUND = 1
    NO_SUCC = 2
    SUCC = 3
    UNDISCOVERED = 4
    DISCOVERED = 5
    FALIURE = 6

    #STATE
    isSolutionFound = False


    def __init__(self, boundary, start, goal, obstacles, space_resolution, stepFunction, primitives=5, primitive_bounds=[-math.pi/4.0, math.pi/4.0]):

        #Abstract Variables
        self.OPEN = []
        self.CLOSE = []
        self.boundary = boundary
        self.Start = start
        self.Goal = goal
        self.Obstacles = obstacles
        self.Res = space_resolution
        self.Path = []

        #Heuristic Functions
        self.Heur = EucliHeur(self.Start, self.Goal, self.Obstacles)

        #Abstract Distance Function
        self.DFunc = self.Heur.getDistance

        #Class Variables
        self.StateSpace = []
        self.StepFunc = stepFunction
        self.NumPrimitives = primitives
        self.PrimBounds = primitive_bounds
        self.Primitives = self.GetPrimitives(self.NumPrimitives, self.PrimBounds)


    #--Overwrite Functions--

    def getNext(self):
        n = self.OPEN.pop(0)
        self.CLOSE.append(n)

        sn = ScaleState(n)
        if sn in self.StateSpace:
            return AStar.DISCOVERED, n
        elif self.isGoal(n):
            return AStar.GOAL_FOUND, n
        else:
            return AStar.UNDISCOVERED, n
       
    def updateFCost(self, n):
        n[5] = self.Heur.getHeuristic(n) + self.Heur.getFixedCost()

    def Expand(self, n):
        succs = []

        for prim in self.Primitives:
            nx, ny, tn = self.StepFunc(n[0],n[1],n[2],prim)
            state = [n[0], prim, nx, ny, tn, 0, 1000]
            if self.isInLimits(state) and (not self.ScaleState(state) in self.StateSpace):
                self.UpdateFCost(state)
                succs.append(state)

        if len(succs) == 0:
            return False, succs
        else:
            return True, succs

    #--Class Functions--

    def ScaleState(self, n):
        return math.round((self.DFunc(n, [0,0]))/self.Res, 4)

    def GetPrimitives(self, num_of_primitives, bounds):
        return np.linspace(bounds[0],bounds[1],num_of_primitives)

    def run(self):
        
        AStar.isSolutionFound = False
        
        self.Start[5] = self.Heur.getHeuristic(self.start)
        self.OPEN.append(self.Start)

        while(not AStar.isSolutionFound):
            
            assert(not self.isOpenEmpty())

            status, state = self.getNext()
            Path[getNextId()] = [state[0], state[1]] 
            
            if status == AStar.GOAL_FOUND:
                print("Goal found!")
                AStar.isSolutionFound = True
                break
            elif status == AStar.Discovered:
                continue
            elif status == AStar.FALIURE:
                print("Faliure")
                break

            succ_exist, succs = self.Expand(state)
            if not succ_exsit:
                print("No Successor Found")
                continue
            
            for s in succs:
                s[6] += self.Heur.getFixedCost()

                if(self.isInList(s, self.OPEN)):
                    temp, element = self.getFromList(s, self.OPEN)
                    self.OPEN.pop(element)
                self.OPEN.append(s)
                self.StateSpace.append(self.ScaleState(s))

            self.sortList(self.OPEN)
        
        if AStar.isSolutionFound:
            return self.Path
        else:
            return None
        


