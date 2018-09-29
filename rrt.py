import math
import numpy as np
import random

class RRT():

    def __init__(self, boundary, start, goal, obstacles, space_resolution, stepFunction,
     primitives=90, primitive_bounds=[-math.pi/4.0, math.pi/4.0], rounding=8, inflation=1.75):

        self.epsilon = 0.1
        self.num_of_nodes = 5000

        self.Boundry = boundary
        self.Start = start
        self.Goal = goal
        self.Obstacles = obstacles

        self.StepFunc = stepFunction
        self.NumPrimitives = primitives
        self.PrimBounds = primitive_bounds
        self.Primitives = self.GetPrimitives(self.NumPrimitives, self.PrimBounds)
        self.Rounding = rounding
        self.ObstacleInflation = inflation

        random.seed(100)

        #smallest_ob = 10000000
        #for ob in self.Obstacles:
        #    if smallest_ob > ob[2]:
        #        smallest_ob = ob[2]
        #self.ObstacleInflation = smallest_ob/2.0

        self.id = -1
        self.goal_found = False

    def getCurrentId(self):
        return self.id

    def getNewId(self):
        self.id +=1
        return self.id

    def getDistance(self, n1, n2):
        x = ( math.pow((n1[0] - n2[0]), 2)) + (math.pow((n1[1] - n2[1]), 2) )
        return math.sqrt(x)

    def steering_angle(self, V,Rg):
        V = [V[0],V[1],0]
        Rg = [Rg[0],Rg[1],0]
        angle = (np.dot(V,Rg))/( (np.linalg.norm(V))*(np.linalg.norm(Rg)) )
        direction = (np.dot(np.cross(V,Rg),[0,0,1]))/( (np.linalg.norm(V))*(np.linalg.norm(Rg)) )
        if angle < -0.25*math.pi:
            angle = -0.25*math.pi
        if angle > 0.25*math.pi:
            angle = 0.25*math.pi
        if direction >0:
            angle = angle
        if direction<0:
            angle = -angle
        return angle

    def step(self,p1,p2):
        # if self.getDistance(p1,p2) <= self.epsilon:
        #     return None
        # else:
        #     theta = math.atan2(p2[1]-p1[1],p2[0]-p1[0])
        #     return p1[0] + self.epsilon*math.cos(theta), p1[1] + self.epsilon*math.sin(theta), theta

        #phi = math.atan2(p2[1]-p1[1],p2[0]-p1[0])
        #phi = phi-p1[2]

        # assert( (phi >= self.PrimBounds[0]) and (phi <= self.PrimBounds[1]) )
        #if phi < self.PrimBounds[0]:
        #    phi = self.PrimBounds[0]
        #elif phi > self.PrimBounds[1]:
        #    phi = self.PrimBounds[1]

        #nx,ny,nt = self.StepFunc(p1[0], p1[1], p1[2], phi)
        #return nx,ny,nt,phi

        v = [math.cos(p1[3]), math.sin(p1[3])]
        g = [p2[0]-p1[0], p2[1]-p1[1]]
        phi = self.steering_angle(v,g)

        nx,ny,nt = self.StepFunc(p1[0], p1[1], p1[2], phi)
        return nx,ny,nt,phi


    def GetPrimitives(self, num_of_primitives, bounds):
        return np.linspace(bounds[0],bounds[1],num_of_primitives)

    def point2line(self, p1,p2):
        a = p2[1] - p1[1]
        b = p1[0] - p2[0]
        c = a*(p1[0]) + b*(p1[1])
        return a,b,c

    def checkCollision(self,a, b, c, x, y, radius):
        # Finding the distance of line
        # from center.
        dist = ((abs(a * x + b * y + c)) /
                math.sqrt(a * a + b * b))

        # Checking if the distance is less
        # than, greater than or equal to radius.
        if (radius == dist):
            # print("Touch")
            return True
        elif (radius > dist):
            # print("Intersect")
            return True
        else:
            # print("Outside")
            return False

    def circle_intersect(self,x1, y1, x2, y2, r1, r2):
        distSq = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)
        radSumSq = (r1 + r2) * (r1 + r2)
        if (distSq == radSumSq):
            return True
        elif (distSq > radSumSq):
            return False
        else:
            return True

    def isCollision(self, p1, p2):
        #a,b,c = self.point2line(p1,p2)
        #cx = (p1[0]+p2[0])/2.0
        #cy = (p1[1]+p2[1])/2.0
        #r = math.sqrt(math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2))/2.0

        #for ob in self.Obstacles:
        #    if self.circle_intersect(cx,cy, ob[0],ob[1],r,ob[2]*self.ObstacleInflation):
        #        if self.checkCollision(a,b,c,ob[0],ob[1],ob[2]*self.ObstacleInflation):
        #            return True
        #return False

        for ob in self.Obstacles:
            d = self.getDistance(p2,ob)
            if p2[0] < 0.0 or p2[0] > 20.0 or p2[1] < 0.0 or p2[1] > 10.0:
            #if p2[0] < 0.1 or p2[0] > 19.9 or p2[1] < 0.1 or p2[1] > 9.9:
                return True #collision occured
            if ob[2] == 0.2:
                if d <= 0.8:
                    #print("Collision")
                    return True
            if d<= ob[2]:
                # print("Collision")
                return True
        return False


    def run(self):
        nodes = {} # nodes[id] = (x,y,theta,phi,id,parent)

        nodes[0] = (self.Start[0], self.Start[1], 0.0, 0.0, self.getNewId(), -1) #(x,y,theta,phi,id,parent)
        finalNode = None
        i=1
        while i<self.num_of_nodes:
            i+=1
            #print("Iteration: {}".format(i))
            #rand = (random.random()*self.Boundry[1][0], random.random()*self.Boundry[1][1])
            rand = (random.uniform(0.0,20.0) , random.uniform(0.0,10.0))
            if i%int(self.num_of_nodes*0.01)==0:
                rand = (self.Goal[0],self.Goal[1])

            nn = nodes[0]
            for k in nodes.keys():
                p = nodes[k]
                if self.getDistance(p,rand) < self.getDistance(nn,rand):
                    nn = p
            #nx, ny, nt, phi = self.step(nn, rand)
            #newNode = (nx,ny,nt,phi,self.getNewId(),nn[4])

            v = [math.cos(nn[2]), math.sin(nn[2])]
            g = [rand[0]-nn[0], rand[1]-nn[1]]
            phi = self.steering_angle(v,g)

            xl = [nn[0]]
            yl = [nn[1]]
            thetal = [nn[2]]
            incr = 0
            xn = 0
            yn = 0
            thetan = 0
            while incr <= 1:
                xn , yn , thetan = self.StepFunc(xl[-1],yl[-1],thetal[-1],phi)
                if self.isCollision(None, (xn,yn)):
                    #print("Collision")
                    break
                xl.append(xn)
                yl.append(yn)
                thetal.append(thetan)
                incr = incr + 0.01 #the increments that the car.step function works by

            newNode = (xn,yn,thetan,phi,self.getNewId(),nn[4])
            if self.isCollision(nn, newNode):
                #print("Collision Found")
                pass
            else:
                nodes[newNode[4]] = newNode
                #print("NewNode: {}".format(newNode))

                if self.getDistance(newNode,self.Goal)<=0.5:
                    #nx, ny, nt, phi = self.step(newNode, self.Goal)
                    #finalNode = (nx,ny,nt,phi,self.getNewId(),newNode[4])
                    #nodes[finalNode[4]] = finalNode
                    finalNode = newNode
                    print("GoalNode: {}".format(finalNode))
                    self.goal_found = True
                    break

            if self.goal_found:
                break
        print("Iteration: {}".format(i))


        # Check goal
        if not self.goal_found:
            best_dist = 100000
            best_node = None
            for k in nodes.keys():
                node = nodes[k]
                dist_to_goal = self.getDistance(node, self.Goal)
                if dist_to_goal < best_dist:
                    best_dist = dist_to_goal
                    best_node = node
        else:
            best_node = finalNode
            best_dist = self.getDistance(best_node, self.Goal)

        print("Best Distance: {}".format(best_dist))


        #nx, ny, nt, phi = self.step(best_node, self.Goal)
        #finalNode = (nx,ny,nt,phi,self.getNewId(),best_node[4])
        #nodes[finalNode[4]] = finalNode

        path = []
        finalNode = best_node
        last_id = finalNode[4]
        while True:
            n = nodes[last_id]
            path.append(n[3])
            last_id = n[5]
            if last_id == -1:
                break

        return path
