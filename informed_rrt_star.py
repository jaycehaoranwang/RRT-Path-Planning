
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class IRrtStar:
    def __init__(self, x_start, x_goal, step_len,
                 goal_sample_rate, search_radius, map_size, iter_max=10000):
        self.x_start = Node(x_start)
        self.x_goal = Node(x_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max

        self.fig, self.ax = plt.subplots()
        self.x_range = map_size[0]
        self.y_range = map_size[1]

        self.obstacles = []
        self.map_edge_clearance = 0.5
        self.nearby_radius_factor = 50
        self.obstacle_clearance = 3
        self.V = [self.x_start]
        self.X_soln = set()
        self.path = None
    def init(self):
        cMin, theta = self.get_distance_and_angle(self.x_start, self.x_goal)
        C = self.RotationToWorldFrame(self.x_start, self.x_goal, cMin)
        xCenter = np.array([[(self.x_start.x + self.x_goal.x) / 2.0],
                            [(self.x_start.y + self.x_goal.y) / 2.0], [0.0]])
        x_best = self.x_start
        return theta, cMin, xCenter, C, x_best

    
    def planning(self):
        plt.ion()

        theta, dist, x_center, C, x_best = self.init()
        c_best = np.inf

        for k in range(self.iter_max):
            if self.X_soln:
                cost = {node: self.Cost(node) for node in self.X_soln}
                x_best = min(cost, key=cost.get)
                c_best = cost[x_best]

            x_rand = self.Sample(c_best, dist, x_center, C)
            x_nearest = self.Nearest(self.V, x_rand)
            x_new = self.Steer(x_nearest, x_rand)

            if x_new and not self.is_collision(x_nearest, x_new):
                X_near = self.Near(self.V, x_new)
                c_min = self.Cost(x_nearest) + self.Line(x_nearest, x_new)
                self.V.append(x_new)

                # choose parent
                for x_near in X_near:
                    c_new = self.Cost(x_near) + self.Line(x_near, x_new)
                   
                    if c_new < c_min:
                        if not self.is_collision(x_near,x_new):
                            x_new.parent = x_near
                            c_min = c_new
                # rewire
                for x_near in X_near:
                    c_near = self.Cost(x_near)
                    c_new = self.Cost(x_new) + self.Line(x_new, x_near)
                   
                    if c_new < c_near:
                        if not self.is_collision(x_new,x_near):
                            
                            x_near.parent = x_new
                        
                if self.InGoalRegion(x_new):
                    if not self.is_collision(x_new, self.x_goal):
                        self.X_soln.add(x_new)

            if k % 10 == 0:
                if self.X_soln:
                    self.path = self.ExtractPath(x_best)
                self.draw_graph(x_center=x_center, c_best=c_best, dist=dist, theta=theta)
                print("Best Path Cost:",c_best)
        return True


    def add_obstacles(self,obstacle_coords):
        # obstacle_coords: [[x_min,y_min],[x_max,y_max]] corners of rectangular obstacle (ALL OBSTACLES ASSUMED TO BE RECTANGULAR)
        self.obstacles.append(obstacle_coords)
        return True 
    
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
            
        
    def Steer(self, x_start, x_goal):
        dist, theta = self.get_distance_and_angle(x_start, x_goal)
        dist = min(self.step_len, dist)
        node_new = Node((x_start.x + dist * math.cos(theta),
                         x_start.y + dist * math.sin(theta)))
        node_new.parent = x_start

        return node_new

    def Near(self, nodelist, node):
        n = len(nodelist) + 1
        r = self.nearby_radius_factor * math.sqrt((math.log(n) / n))

        dist_table = [(nd.x - node.x) ** 2 + (nd.y - node.y) ** 2 for nd in nodelist]
        X_near = [nodelist[ind] for ind in range(len(dist_table)) if dist_table[ind] <= r ** 2 and
                  not self.is_collision(nodelist[ind], node)]

        return X_near

    def Sample(self, c_max, c_min, x_center, C):
        if c_max < np.inf:
            r = [c_max / 2.0,
                 math.sqrt(c_max ** 2 - c_min ** 2) / 2.0,
                 math.sqrt(c_max ** 2 - c_min ** 2) / 2.0]
            L = np.diag(r)

            while True:
                x_ball = self.SampleUnitBall()
                x_rand = np.dot(np.dot(C, L), x_ball) + x_center
                if self.x_range[0] + self.map_edge_clearance <= x_rand[0] <= self.x_range[1] - self.map_edge_clearance and \
                        self.y_range[0] + self.map_edge_clearance <= x_rand[1] <= self.y_range[1] - self.map_edge_clearance:
                    break
            x_rand = Node((x_rand[(0, 0)], x_rand[(1, 0)]))
        else:
            x_rand = self.SampleFreeSpace()

        return x_rand

    @staticmethod
    def SampleUnitBall():
        while True:
            x, y = random.uniform(-1, 1), random.uniform(-1, 1)
            if x ** 2 + y ** 2 < 1:
                return np.array([[x], [y], [0.0]])

    def SampleFreeSpace(self):
        delta = self.map_edge_clearance

        if np.random.random() > self.goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.x_goal

    def ExtractPath(self, node):
        path = [[self.x_goal.x, self.x_goal.y]]

        while node.parent:
            path.append([node.x, node.y])
            node = node.parent

        path.append([self.x_start.x, self.x_start.y])

        return path

    def InGoalRegion(self, node):
        if self.Line(node, self.x_goal) < self.step_len:
            return True

        return False

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
    def Nearest(nodelist, n):
        return nodelist[int(np.argmin([(nd.x - n.x) ** 2 + (nd.y - n.y) ** 2
                                       for nd in nodelist]))]

    @staticmethod
    def Line(x_start, x_goal):
        return math.hypot(x_goal.x - x_start.x, x_goal.y - x_start.y)

    def Cost(self, node):
        if node == self.x_start:
            return 0.0

        if node.parent is None:
            return np.inf

        cost = 0.0
        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

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
    
    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def draw_graph(self, x_center=None, c_best=None, dist=None, theta=None):
        self.ax.clear()
        # Plot obstacles
        for obstacle in self.obstacles:
            rect = plt.Rectangle((obstacle[0][0], obstacle[0][1]), obstacle[1][0] - obstacle[0][0], obstacle[1][1] - obstacle[0][1], color='black')
            self.ax.add_patch(rect)
        
        for node in self.V:
            # Plot edges
            if node.parent:
                self.ax.plot([node.x, node.parent.x], [node.y, node.parent.y], linewidth=0.5, color="g")
            # Plot nodes
            circle = plt.Circle((node.x, node.y), radius=0.3, color="b")
            self.ax.add_patch(circle)
        
        if c_best != np.inf:
            self.draw_ellipse(x_center, c_best, dist, theta)
            self.ax.plot([x for (x, y) in self.path], [y for (x, y) in self.path], '-r')
            
        self.ax.plot(self.x_start.x, self.x_start.y, "xr")
        self.ax.plot(self.x_goal.x, self.x_goal.y, "xr")
        self.ax.set_xlim(self.x_range[0], self.x_range[1])
        self.ax.set_ylim(self.y_range[0], self.y_range[1])
        self.ax.grid(True)
        plt.pause(0.01)

    def draw_ellipse(self,x_center, c_best, dist, theta):
        a = c_best/2
        c = dist/2
        b = np.sqrt(a**2 - c**2)
        ellipse = Ellipse(xy=x_center, width=2*a, height=2*b, angle=theta, linestyle='--', edgecolor='darkorange', facecolor='none')
        self.ax.add_patch(ellipse)
        return True

def main():
    x_start = (2,30)  # Starting node
    x_goal = (58, 30)  # Goal node

    informed_rrt_star = IRrtStar(x_start, x_goal, step_len=1.5,
                        goal_sample_rate=0.1, search_radius=10, map_size=[[0,60],[0,60]], iter_max=10000)
                 #1, 0.10, 12, 1000)
    informed_rrt_star.add_obstacles([(20,20),(30,30)])
    informed_rrt_star.add_obstacles([(30,40),(40,50)])
    informed_rrt_star.add_obstacles([(40,30),(50,40)])
    
    informed_rrt_star.planning()


if __name__ == '__main__':
    main()