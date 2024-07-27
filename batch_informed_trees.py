
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import time
from dubins import plan_dubins_path, plot_arrow
import ipdb
class Node:
    def __init__(self, x, y, heading=0):
        self.x = x
        self.y = y
        self.parent = None
        self.heading = heading

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
    def __init__(self, x_start, x_goal, map_size, search_radius=10, iter_max=500, visualize=True, seed = None, enable_dubins_paths = False, min_turning_radius = None):
        self.x_start = Node(x_start[0], x_start[1], heading=x_start[2])
        self.x_goal = Node(x_goal[0], x_goal[1], heading=x_goal[2])
        self.iter_max = iter_max
        self.enable_dubins_paths = enable_dubins_paths
        self.fig, self.ax = plt.subplots()
        self.discrete_acceptable_headings = [45,22.5,0,-22.5,-45]
        self.map_edge_clearance = 0.5
        self.obstacle_clearance = 1
        self.x_range = map_size[0]
        self.y_range = map_size[1]
        self.visualize = visualize
        self.obstacles = []
        self.Tree = Tree(self.x_start, self.x_goal, search_radius)
        self.X_sample = set()
        self.g_T = dict()
        self.seed = seed
        self.min_turning_radius = min_turning_radius
        self.curvature = 1/self.min_turning_radius
        self.best_path = []
        self.best_path_cost = np.inf

    def init(self):
        self.Tree.V.add(self.x_start)
        self.X_sample.add(self.x_goal)

        if self.seed:
            random.seed(self.seed)

        self.g_T[self.x_start] = 0.0
        self.g_T[self.x_goal] = np.inf

        cMin, theta = self.calc_dist_and_angle(self.x_start, self.x_goal)
        C = self.RotationToWorldFrame(self.x_start, self.x_goal, cMin)
        xCenter = np.array([[(self.x_start.x + self.x_goal.x) / 2.0],
                            [(self.x_start.y + self.x_goal.y) / 2.0], [0.0]])
        
        return theta, cMin, xCenter, C

    def planning(self):
        theta, cMin, xCenter, C = self.init()
        start = time.time()
        for k in range(self.iter_max):
            no_solution = False
            if not self.Tree.QE and not self.Tree.QV:
                if k == 0:
                    m = 200 # Sample count on first batch
                else:
                    m = 100 # Sample count on other batches

                self.Prune(self.g_T[self.x_goal])
                self.X_sample.update(self.Sample(m, self.g_T[self.x_goal], cMin, xCenter, C))
                self.Tree.V_old = {v for v in self.Tree.V}
                self.Tree.QV = {v for v in self.Tree.V}
                
            while self.BestVertexQueueValue() <= self.BestEdgeQueueValue():
                v = self.BestInVertexQueue()
                if v is None:
                    no_solution = True
                    break
                else:
                    self.ExpandVertex(v)

            if no_solution:
                print("No Solution Found, going to Next batch")
                self.Tree.QE = set()
                self.Tree.QV = set()
                continue

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
                end = time.time()
                _,_,cost = self.ExtractPath()
                print("Batch Time:", end-start)
                print("Cost:", cost)
                start = time.time()
                if self.visualize:
                        print("Batch Complete")
                        self.draw_graph(xCenter, self.g_T[self.x_goal] ,cMin, theta)
            
        print("Planning Done")
        print("Solution path cost:",self.g_T[self.x_goal])
        self.draw_graph(xCenter, self.g_T[self.x_goal] ,cMin, theta)

        return True

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
        print("Path Length:", len(path_x))
        return path_x, path_y, path_cost
    
    def ExtractDubinsPath(self):
        path_cost = 0
        node = self.x_goal
        path_x, path_y = [node.x], [node.y]

        while node.parent:
            segment_x, segment_y, path_yaw, mode, lengths = plan_dubins_path(
                                                            node.parent.x,
                                                            node.parent.y,
                                                            np.deg2rad(node.parent.heading),
                                                            node.x,
                                                            node.y,
                                                            np.deg2rad(node.heading),
                                                            self.curvature)
            path_cost += sum(lengths)
            node = node.parent
            #insert segments at beginning of list
            path_x[0:0] = segment_x 
            path_y[0:0] = segment_y

        return path_x, path_y, path_cost

    def Prune(self, cBest):
        self.X_sample = {x for x in self.X_sample if self.f_estimated(x) < cBest}
        self.Tree.V = {v for v in self.Tree.V if self.f_estimated(v) <= cBest}
        self.Tree.E = {(v, w) for v, w in self.Tree.E
                       if self.f_estimated(v) <= cBest and self.f_estimated(w) <= cBest}
        self.X_sample.update({v for v in self.Tree.V if self.g_T[v] == np.inf})
        self.Tree.V = {v for v in self.Tree.V if self.g_T[v] < np.inf}


    def cost(self, start, end):
        if not self.enable_dubins_paths:
            if self.is_collision(start, end):
                return np.inf
            else:
                return self.calc_dist(start, end)
        else:
            #assess viable dubins paths according to acceptable end headings and pick the best one in terms of cost
            path_and_costs = []
            for yaw in self.discrete_acceptable_headings:
                path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(
                                                            start.x,
                                                            start.y,
                                                            np.deg2rad(start.heading),
                                                            end.x,
                                                            end.y,
                                                            np.deg2rad(yaw),
                                                            self.curvature)
                path_and_costs.append([[path_x, path_y],sum(lengths),yaw])

            path_and_costs.sort(key=lambda x: x[1])
            # Check for collisions in min cost dubin paths, if none, return the cost and set the heading
            for p in path_and_costs:
                # If the min path has a collision, goto the next min path
                if self.is_collision_dubins_path(p[0][0], p[0][1]):
                    continue
                #If there is no collision, then this path is viable
                end.heading = p[-1] #Set the end node's heading
                return p[1]

            return np.inf #If ALL paths have collisions, these nodes are not able to be connected

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

    def dubins_point_intersection(self, points_x,points_y, obs_rect):
            for x,y in zip(points_x,points_y):
                if x >= obs_rect[0][0] and x <= obs_rect[1][0] and y >= obs_rect[0][1] and y <= obs_rect[1][1]:
                    return True

            return False
    
    
    def is_collision_dubins_path(self,path_x,path_y):
        # Check for collision of dubin paths between nodes with obstacles, dubin paths are given as a series of (x,y) points 
        # so collision checking only involves seeing if points reside within illegal bounds of the obstacles
        def rectangle_intersection(rect1,rect2):
            '''
            Parameters:
            - rect1: A tuple (xmin1, ymin1, xmax1, ymax1) representing the first rectangle.
            - rect2: A tuple (xmin2, ymin2, xmax2, ymax2) representing the second rectangle
            '''
            xmin1, ymin1, xmax1, ymax1 = rect1[0][0], rect1[0][1], rect1[1][0], rect1[1][1]
            xmin2, ymin2, xmax2, ymax2 = rect2[0][0], rect2[0][1], rect2[1][0], rect2[1][1]

            # Check if one rectangle is to the left of the other
            if xmax1 < xmin2 or xmax2 < xmin1:
                return False
            
            # Check if one rectangle is above the other
            if ymax1 < ymin2 or ymax2 < ymin1:
                return False
        
            return True
        #Check for bounding box collision of path with any obstacle first
        min_x = min(path_x)
        max_x = max(path_x)
        min_y = min(path_y)
        max_y = max(path_y)
        path_box = [[min_x,min_y],[max_x,max_y]]
        # Check that the path box does not exceed env boundaries
        if path_box[0][0] <= self.x_range[0] + self.map_edge_clearance or path_box[1][0] >= self.x_range[1] - self.map_edge_clearance or path_box[0][1] <= self.y_range[0] + self.map_edge_clearance or path_box[1][1] >= self.y_range[1] - self.map_edge_clearance:
            return True
        
        for obstacle in self.obstacles:
            obs_x_min, obs_y_min = obstacle[0][0] - self.obstacle_clearance, obstacle[0][1] - self.obstacle_clearance
            obs_x_max, obs_y_max = obstacle[1][0] + self.obstacle_clearance, obstacle[1][1] + self.obstacle_clearance
            obstacle_rectangle = [[obs_x_min,obs_y_min],[obs_x_max,obs_y_max]]
            if rectangle_intersection(path_box,obstacle_rectangle):
                if self.dubins_point_intersection(path_x,path_y,obstacle_rectangle):
                    return True

        return False


    def is_collision(self,start,end):
        # Check for collision of straight line paths between nodes with obstacles

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

        #v_value = {v: self.g_T[v] + self.h_estimated(v) for v in self.Tree.QV}
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

    def draw_graph(self, x_center=None, c_best=None, dist=None, theta=None):
        self.ax.clear()
        # Plot obstacles
        for obstacle in self.obstacles:
            rect = plt.Rectangle((obstacle[0][0], obstacle[0][1]), obstacle[1][0] - obstacle[0][0], obstacle[1][1] - obstacle[0][1], color='black')
            self.ax.add_patch(rect)
        
        #Plot Samples
        for node in self.X_sample:
            # Plot nodes
            circle = plt.Circle((node.x, node.y), radius=0.2, color="b")
            self.ax.add_patch(circle)
        
        #Plot the explored tree vertexes
        for v in self.Tree.V:
            circle = plt.Circle((v.x, v.y), radius=0.2, color="orange")
            self.ax.add_patch(circle)
            #ipdb.set_trace()
            plot_arrow(v.x, v.y, np.deg2rad(v.heading), arrow_length=1, ax=self.ax, origin_point_plot_style=None)

        #plot the explored tree edges
        # for e in self.Tree.E:
        #     if not self.enable_dubins_paths:
        #         self.ax.plot([e[0].x, e[1].x], [e[0].y, e[1].y], linewidth=0.5,alpha=0.8, color="g")
        #     else:
        #         path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(
        #                                                     e[0].x,
        #                                                     e[0].y,
        #                                                     np.deg2rad(e[0].heading),
        #                                                     e[1].x,
        #                                                     e[1].y,
        #                                                     np.deg2rad(e[1].heading),
        #                                                     self.curvature)
        #         self.ax.plot(path_x, path_y,linewidth=0.5,alpha=0.8, color="g")

        #plot ellipse and current solution
        if c_best != np.inf:
            self.draw_ellipse(x_center, c_best, dist, theta)
            if self.enable_dubins_paths:
                path_x, path_y, path_cost = self.ExtractDubinsPath()
            else:
                path_x, path_y, path_cost = self.ExtractPath()
            if path_cost < self.best_path_cost:
                self.best_path_cost = path_cost
                self.best_path = []
                self.best_path.append(path_x)
                self.best_path.append(path_y)
            self.ax.plot(self.best_path[0], self.best_path[1], '-r')
            print("Solution Path Cost:", round(self.best_path_cost,3))

        self.ax.plot(self.x_start.x, self.x_start.y, "xr")
        self.ax.plot(self.x_goal.x, self.x_goal.y, "xr")
        self.ax.set_xlim(self.x_range[0], self.x_range[1])
        self.ax.set_ylim(self.y_range[0], self.y_range[1])
        self.ax.grid(True)
        plt.pause(5)

def main():
    x_start = (2,15)  # Starting node
    x_goal = (45, 15)  # Goal node
    
    batch_informed_star = BITStar(x_start, x_goal, map_size=[[0,50],[0,30]], search_radius = 10, iter_max=100000, visualize=True, enable_dubins_paths=False, min_turning_radius=6)
    batch_informed_star.add_obstacles([(10,10),(15,15)])
    batch_informed_star.add_obstacles([(20,20),(25,25)])
    batch_informed_star.add_obstacles([(25,15),(30,20)])
    '''
    #Intersection sim
    batch_informed_star.add_obstacles([(0,20),(20,30)])
    batch_informed_star.add_obstacles([(30,20),(50,30)])
    batch_informed_star.add_obstacles([(0,0),(20,10)])
    batch_informed_star.add_obstacles([(30,0),(50,10)])
    '''
    batch_informed_star.planning()


if __name__ == '__main__':
    main()