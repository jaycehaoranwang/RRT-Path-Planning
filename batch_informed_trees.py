
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import ipdb 

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class Tree:
    def __init__(self, x_start, x_goal, radius):
        self.x_start = x_start
        self.goal = x_goal

        self.r = radius
        self.V = set()
        self.E = set()
        self.QE = set()
        self.QV = set()

        self.V_old = set()


class BITStar:
    def __init__(self, x_start, x_goal, map_size, search_radius=10, iter_max=500):
        self.x_start = Node(x_start[0], x_start[1])
        self.x_goal = Node(x_goal[0], x_goal[1])
        self.iter_max = iter_max

        self.fig, self.ax = plt.subplots()

        self.map_edge_clearance = 0.5
        self.obstacle_clearance = 3
        self.x_range = map_size[0]
        self.y_range = map_size[1]

        self.obstacles = []
        self.Tree = Tree(self.x_start, self.x_goal, search_radius)
        self.X_sample = set()
        self.g_T = dict()

    def init(self):
        self.Tree.V.add(self.x_start)
        self.X_sample.add(self.x_goal)

        self.g_T[self.x_start] = 0.0
        self.g_T[self.x_goal] = np.inf

        cMin, theta = self.calc_dist_and_angle(self.x_start, self.x_goal)
        C = self.RotationToWorldFrame(self.x_start, self.x_goal, cMin)
        xCenter = np.array([[(self.x_start.x + self.x_goal.x) / 2.0],
                            [(self.x_start.y + self.x_goal.y) / 2.0], [0.0]])

        return theta, cMin, xCenter, C

    def planning(self):
        theta, cMin, xCenter, C = self.init()

        for k in range(self.iter_max):
            if not self.Tree.QE and not self.Tree.QV:
                if k == 0:
                    m = 350
                else:
                    m = 200

                if self.x_goal.parent is not None:
                    self.draw_graph(xCenter, self.g_T[self.x_goal] ,cMin, theta)

                self.Prune(self.g_T[self.x_goal])
                self.X_sample.update(self.Sample(m, self.g_T[self.x_goal], cMin, xCenter, C))
                
                self.Tree.V_old = {v for v in self.Tree.V}
                self.Tree.QV = {v for v in self.Tree.V}
                #self.Tree.r = self.radius(len(self.Tree.V) + len(self.X_sample))
                #print("Tree Radius:", self.Tree.r)
            while self.BestVertexQueueValue() <= self.BestEdgeQueueValue():
                self.ExpandVertex(self.BestInVertexQueue())

            vm, xm = self.BestInEdgeQueue()
            self.Tree.QE.remove((vm, xm))

            if self.g_T[vm] + self.calc_dist(vm, xm) + self.h_estimated(xm) < self.g_T[self.x_goal]:
                actual_cost = self.cost(vm, xm)
                if self.g_estimated(vm) + actual_cost + self.h_estimated(xm) < self.g_T[self.x_goal]:
                    if self.g_T[vm] + actual_cost < self.g_T[xm]:
                        if xm in self.Tree.V:
                            # remove edges
                            edge_delete = set()
                            for v, x in self.Tree.E:
                                if x == xm:
                                    edge_delete.add((v, x))

                            for edge in edge_delete:
                                self.Tree.E.remove(edge)
                        else:
                            self.X_sample.remove(xm)
                            self.Tree.V.add(xm)
                            self.Tree.QV.add(xm)

                        self.g_T[xm] = self.g_T[vm] + actual_cost
                        self.Tree.E.add((vm, xm))
                        xm.parent = vm

                        set_delete = set()
                        for v, x in self.Tree.QE:
                            if x == xm and self.g_T[v] + self.calc_dist(v, xm) >= self.g_T[xm]:
                                set_delete.add((v, x))

                        for edge in set_delete:
                            self.Tree.QE.remove(edge)
            else:
                self.Tree.QE = set()
                self.Tree.QV = set()
                print("Batch Complete, Solution Path Cost:",)

                self.draw_graph(xCenter, self.g_T[self.x_goal] ,cMin, theta)
            #if k % 5 == 0:
                #self.animation(xCenter, self.g_T[self.x_goal], cMin, theta)

        print("Planning Done")
        return True
        #plt.plot(path_x, path_y, linewidth=2, color='r')
        #plt.pause(0.01)
        #plt.show()


    def draw_graph(self, x_center=None, c_best=None, dist=None, theta=None):
        self.ax.clear()
        # Plot obstacles
        for obstacle in self.obstacles:
            rect = plt.Rectangle((obstacle[0][0], obstacle[0][1]), obstacle[1][0] - obstacle[0][0], obstacle[1][1] - obstacle[0][1], color='black')
            self.ax.add_patch(rect)
        
        #Plot Samples
        for node in self.X_sample:
            # Plot nodes
            circle = plt.Circle((node.x, node.y), radius=0.3, color="b")
            self.ax.add_patch(circle)
        
        #Plot the explored tree vertexes
        for v in self.Tree.V:
            circle = plt.Circle((v.x, v.y), radius=0.3, color="orange")
            self.ax.add_patch(circle)
        
        #plot the explored tree edges
        for e in self.Tree.E:
            self.ax.plot([e[0].x, e[1].x], [e[0].y, e[1].y], linewidth=0.8, color="g")

        #plot ellipse and current solution
        if c_best != np.inf:
            self.draw_ellipse(x_center, c_best, dist, theta)
            path_x, path_y, path_cost = self.ExtractPath()
            self.ax.plot(path_x, path_y, '-r')
            print("Solution Path Cost:", round(path_cost,3))

        self.ax.plot(self.x_start.x, self.x_start.y, "xr")
        self.ax.plot(self.x_goal.x, self.x_goal.y, "xr")
        self.ax.set_xlim(self.x_range[0], self.x_range[1])
        self.ax.set_ylim(self.y_range[0], self.y_range[1])
        self.ax.grid(True)
        plt.pause(0.01)

    def add_obstacles(self,obstacle_coords):
        # obstacle_coords: [[x_min,y_min],[x_max,y_max]] corners of rectangular obstacle (ALL OBSTACLES ASSUMED TO BE RECTANGULAR)
        self.obstacles.append(obstacle_coords)
        return True 
    
    def ExtractPath(self):
        path_cost = 0
        node = self.x_goal
        path_x, path_y = [node.x], [node.y]

        while node.parent:
            path_cost += self.cost(node,node.parent)
            node = node.parent
            path_x.append(node.x)
            path_y.append(node.y)

        return path_x, path_y, path_cost

    def Prune(self, cBest):
        self.X_sample = {x for x in self.X_sample if self.f_estimated(x) < cBest}
        self.Tree.V = {v for v in self.Tree.V if self.f_estimated(v) <= cBest}
        self.Tree.E = {(v, w) for v, w in self.Tree.E
                       if self.f_estimated(v) <= cBest and self.f_estimated(w) <= cBest}
        self.X_sample.update({v for v in self.Tree.V if self.g_T[v] == np.inf})
        self.Tree.V = {v for v in self.Tree.V if self.g_T[v] < np.inf}

    def cost(self, start, end):
        if self.is_collision(start, end):
            return np.inf

        return self.calc_dist(start, end)

    def f_estimated(self, node):
        return self.g_estimated(node) + self.h_estimated(node)

    def g_estimated(self, node):
        return self.calc_dist(self.x_start, node)

    def h_estimated(self, node):
        return self.calc_dist(node, self.x_goal)

    def Sample(self, m, cMax, cMin, xCenter, C):
        if cMax < np.inf:
            return self.SampleEllipsoid(m, cMax, cMin, xCenter, C)
        else:
            return self.SampleFreeSpace(m)

    def SampleEllipsoid(self, m, cMax, cMin, xCenter, C):
        r = [cMax / 2.0,
             math.sqrt(cMax ** 2 - cMin ** 2) / 2.0,
             math.sqrt(cMax ** 2 - cMin ** 2) / 2.0]
        L = np.diag(r)

        ind = 0
        delta = self.map_edge_clearance
        Sample = set()

        while ind < m:
            xBall = self.SampleUnitNBall()
            x_rand = np.dot(np.dot(C, L), xBall) + xCenter
            node = Node(x_rand[(0, 0)], x_rand[(1, 0)])
            in_obs = self.inside_obstacle(node)
            in_x_range = self.x_range[0] + delta <= node.x <= self.x_range[1] - delta
            in_y_range = self.y_range[0] + delta <= node.y <= self.y_range[1] - delta

            if not in_obs and in_x_range and in_y_range:
                Sample.add(node)
                ind += 1

        return Sample

    def SampleFreeSpace(self, m):
        delta = self.map_edge_clearance
        Sample = set()

        ind = 0
        while ind < m:
            node = Node(random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                        random.uniform(self.y_range[0] + delta, self.y_range[1] - delta))
            if self.inside_obstacle(node):
                continue
            else:
                Sample.add(node)
                ind += 1

        return Sample
    
    def inside_obstacle(self,node):
        for obstacle in self.obstacles:
            if node.x >= obstacle[0][0] - self.obstacle_clearance and node.x <= obstacle[1][0] + self.obstacle_clearance and node.y >= obstacle[0][1] - self.obstacle_clearance and node.y <= obstacle[1][1] + self.obstacle_clearance:
                return True 
        return False

    def is_collision(self,start,end):

        if self.inside_obstacle(start) or self.inside_obstacle(end):
            return True

        #Check line intersection with edges of rectangular obstacles
        x2, y2 = end.x, end.y
        x1, y1 = start.x, start.y
        segment_start = (x1, y1)
        segment_end = (x2, y2)
        for obstacle in self.obstacles:
            obs_x_min, obs_y_min = obstacle[0][0] - self.obstacle_clearance, obstacle[0][1] - self.obstacle_clearance
            obs_x_max, obs_y_max = obstacle[1][0] + self.obstacle_clearance, obstacle[1][1] + self.obstacle_clearance
            # Bounding Box check
            if max(x1,x2) < obs_x_min or min(x1,x2) > obs_x_max or max(y1,y2) < obs_y_min or min(y1,y2) > obs_y_max:
                continue #no collision 
            #check for boundary intersections with node path
            if self.line_segments_intersection(segment_start,segment_end, (obs_x_min, obs_y_min), (obs_x_min,obs_y_max)):
                return True
            if self.line_segments_intersection(segment_start,segment_end, (obs_x_min,obs_y_max), (obs_x_max,obs_y_max)):
                return True
            if self.line_segments_intersection(segment_start,segment_end, (obs_x_max,obs_y_max), (obs_x_max,obs_y_min)):
                return True
            if self.line_segments_intersection(segment_start,segment_end, (obs_x_max,obs_y_min), (obs_x_min, obs_y_min)):
                return True
        
        return False
    
    def ExpandVertex(self, v):
        self.Tree.QV.remove(v)
        X_near = {x for x in self.X_sample if self.calc_dist(x, v) <= self.Tree.r}
        for x in X_near:
            if self.g_estimated(v) + self.calc_dist(v, x) + self.h_estimated(x) < self.g_T[self.x_goal]:
                self.g_T[x] = np.inf
                self.Tree.QE.add((v, x))

        if v not in self.Tree.V_old:
            V_near = {w for w in self.Tree.V if self.calc_dist(w, v) <= self.Tree.r}

            for w in V_near:
                if (v, w) not in self.Tree.E and \
                        self.g_estimated(v) + self.calc_dist(v, w) + self.h_estimated(w) < self.g_T[self.x_goal] and \
                        self.g_T[v] + self.calc_dist(v, w) < self.g_T[w]:
                    self.Tree.QE.add((v, w))
                    if w not in self.g_T:
                        self.g_T[w] = np.inf

    def BestVertexQueueValue(self):
        if not self.Tree.QV:
            return np.inf

        return min(self.g_T[v] + self.h_estimated(v) for v in self.Tree.QV)

    def BestEdgeQueueValue(self):
        if not self.Tree.QE:
            return np.inf

        return min(self.g_T[v] + self.calc_dist(v, x) + self.h_estimated(x)
                   for v, x in self.Tree.QE)

    def BestInVertexQueue(self):
        if not self.Tree.QV:
            print("QV is Empty!")
            return None

        v_value = {v: self.g_T[v] + self.h_estimated(v) for v in self.Tree.QV}

        return min(v_value, key=v_value.get)

    def BestInEdgeQueue(self):
        if not self.Tree.QE:
            print("QE is Empty!")
            return None

        e_value = {(v, x): self.g_T[v] + self.calc_dist(v, x) + self.h_estimated(x)
                   for v, x in self.Tree.QE}

        return min(e_value, key=e_value.get)

    @staticmethod
    def SampleUnitNBall():
        while True:
            x, y = random.uniform(-1, 1), random.uniform(-1, 1)
            if x ** 2 + y ** 2 < 1:
                return np.array([[x], [y], [0.0]])

    @staticmethod
    def RotationToWorldFrame(x_start, x_goal, L):
        a1 = np.array([[(x_goal.x - x_start.x) / L],
                       [(x_goal.y - x_start.y) / L], [0.0]])
        e1 = np.array([[1.0], [0.0], [0.0]])
        M = a1 @ e1.T
        U, _, V_T = np.linalg.svd(M, True, True)
        C = U @ np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(V_T.T)]) @ V_T

        return C

    @staticmethod
    def calc_dist(start, end):
        return math.hypot(start.x - end.x, start.y - end.y)

    @staticmethod
    def calc_dist_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def draw_ellipse(self,x_center, c_best, dist, theta):
        a = c_best/2
        c = dist/2
        b = np.sqrt(a**2 - c**2)
        ellipse = Ellipse(xy=x_center, width=2*a, height=2*b, angle=theta, linestyle='--', edgecolor='darkorange', facecolor='none')
        self.ax.add_patch(ellipse)
        return True
    @staticmethod
    def line_segments_intersection(p1, p2, q1, q2):
            """
            Determines if two line segments (p1 to p2) and (q1 to q2) intersect within their bounds.
            
            Args:
            p1, p2: Tuples representing the start and end points of the first line segment.
            q1, q2: Tuples representing the start and end points of the second line segment.
            
            Returns:
            True if the line segments intersect, False otherwise.
            """
            def orientation(a, b, c):
                """
                Helper function to determine the orientation of the triplet (a, b, c).
                It returns:
                0 -> if a, b, c are collinear
                1 -> if the sequence a, b, c is clockwise
                2 -> if the sequence a, b, c is counterclockwise
                """
                val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
                if val == 0:
                    return 0
                elif val > 0:
                    return 1
                else:
                    return 2

            def on_segment(a, b, c):
                """
                Given three collinear points a, b, c, checks if point b lies on line segment a-c.
                """
                if min(a[0], c[0]) <= b[0] <= max(a[0], c[0]) and min(a[1], c[1]) <= b[1] <= max(a[1], c[1]):
                    return True
                return False

            # Find the four orientations needed for the general and special cases
            o1 = orientation(p1, p2, q1)
            o2 = orientation(p1, p2, q2)
            o3 = orientation(q1, q2, p1)
            o4 = orientation(q1, q2, p2)

            # General case: the segments intersect if they straddle each other
            if o1 != o2 and o3 != o4:
                return True
            # Special cases:
            # p1, p2 and q1 are collinear and q1 lies on segment p1p2
            if o1 == 0 and on_segment(p1, q1, p2):
                return True
            # p1, p2 and q2 are collinear and q2 lies on segment p1p2
            if o2 == 0 and on_segment(p1, q2, p2):
                return True
            # q1, q2 and p1 are collinear and p1 lies on segment q1q2
            if o3 == 0 and on_segment(q1, p1, q2):
                return True
            # q1, q2 and p2 are collinear and p2 lies on segment q1q2
            if o4 == 0 and on_segment(q1, p2, q2):
                return True
            # If none of the cases apply, the segments do not intersect
            return False

def main():
    x_start = (2,30)  # Starting node
    x_goal = (58, 30)  # Goal node
    
    batch_informed_star = BITStar(x_start, x_goal, map_size=[[0,60],[0,60]], search_radius = 15, iter_max=100000)
                 #1, 0.10, 12, 1000)
    batch_informed_star.add_obstacles([(20,20),(30,30)])
    batch_informed_star.add_obstacles([(30,40),(40,50)])
    batch_informed_star.add_obstacles([(40,30),(50,40)])
    
    batch_informed_star.planning()


if __name__ == '__main__':
    main()