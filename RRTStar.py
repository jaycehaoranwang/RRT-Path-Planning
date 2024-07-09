import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import random
import time
import ipdb
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0
        self.children_count = 0

def do_line_segments_intersect(p1, p2, q1, q2):
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

class RRTStar:
    def __init__(self, start, goal, map_size, step_size=2, goal_sample_rate=0.15, max_iter=100000, fixed_node = 0):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.map_size = map_size
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list = [self.start]
        self.obstacles = []
        self.obstacle_clearance = 2
        self.search_radius = 5
        self.best_path = None
        self.kd_tree = None
        self.goal_found = False
        self.solution_nodes = []
        self.fixed_node = fixed_node
    def plan(self):
        plt.ion()
        fig, ax = plt.subplots()
        self.draw_graph(ax)
        count = 0
        no_obs = True
        while count < self.max_iter:
            #time.sleep(0.5)
            rnd_node = self.get_random_node()
            nearest_node = self.get_nearest_node(self.node_list, rnd_node)
            new_node = self.steer(nearest_node, rnd_node, self.step_size)

            if self.check_collision(new_node):
                near_nodes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_nodes)
                self.node_list.append(new_node)
                
                self.rewire(new_node, near_nodes)
                self.draw_graph(ax)

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.step_size:
                final_node = self.steer(self.node_list[-1], self.goal, self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y))
                if self.check_collision(final_node):
                    # We've found the goal node, stop sampling goal
                    print("Solution Node Found")
                    self.goal_sample_rate = 0
                    self.node_list.append(final_node)
                    self.solution_nodes.append(final_node)
                    self.goal_found = True
                    self.best_path, lowest_cost = self.generate_best_path()
                    print('Solution Path Cost:',lowest_cost)
            if self.goal_found:
                self.draw_graph(ax, self.best_path)
                if count%10 == 0: #update best path every 10 new iterations
                    self.best_path, lowest_cost = self.generate_best_path()
                    print('Solution Path Cost:',lowest_cost)

            count += 1
        print("Plan terminated")
        plt.ioff()
        plt.show()
        return True

    def get_random_node(self):
        if random.random() > self.goal_sample_rate:
            rnd = Node(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))
        else:
            rnd = self.goal
            print(f"Sampling Goal")
        return rnd

    def add_obstacle(self, min_vertex, max_vertex):
            
        self.obstacles.append([(min_vertex[0], min_vertex[1]),
                               (max_vertex[0], max_vertex[1])])

        return True

    def get_nearest_node(self, node_list, rnd_node):
        nodes = [(node.x, node.y) for node in node_list]
        kdtree = KDTree(nodes)
        self.kd_tree = kdtree
        _, idx = kdtree.query([(rnd_node.x, rnd_node.y)])
        return node_list[idx[0]]

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        new_node.x += min(extend_length, d) * np.cos(theta)
        new_node.y += min(extend_length, d) * np.sin(theta)
        new_node.cost = from_node.cost + d
        new_node.parent = from_node
        return new_node

    def obstacle_intersection(self, node):
        for obstacle in self.obstacles:
            x2, y2 = node.x, node.y
            x1, y1 = node.parent.x, node.parent.y
            obs_x_min, obs_y_min = obstacle[0][0] - self.obstacle_clearance, obstacle[0][1] - self.obstacle_clearance
            obs_x_max, obs_y_max = obstacle[1][0] + self.obstacle_clearance, obstacle[1][1] + self.obstacle_clearance
            # Bounding Box check
            if max(x1,x2) < obs_x_min or min(x1,x2) > obs_x_max or max(y1,y2) < obs_y_min or min(y1,y2) > obs_y_max:
                continue #no collision 
            segment_start = (x1, y1)
            segment_end = (x2, y2)
            #check for boundary intersections with node path
            if do_line_segments_intersect(segment_start,segment_end, (obs_x_min, obs_y_min), (obs_x_min,obs_y_max)):
                return True
            if do_line_segments_intersect(segment_start,segment_end, (obs_x_min,obs_y_max), (obs_x_max,obs_y_max)):
                return True
            if do_line_segments_intersect(segment_start,segment_end, (obs_x_max,obs_y_max), (obs_x_max,obs_y_min)):
                return True
            if do_line_segments_intersect(segment_start,segment_end, (obs_x_max,obs_y_min), (obs_x_min, obs_y_min)):
                return True
            
        return False
    
    def check_collision(self, node): #return true means no collision
        no_collision = True
        if 0 > node.x or node.x > self.map_size[0] or 0 > node.y or node.y > self.map_size[1]:
            no_collision = False
        
        for obstacle in self.obstacles:
            if node.x >= obstacle[0][0] - self.obstacle_clearance and node.x <= obstacle[1][0] + self.obstacle_clearance and node.y >= obstacle[0][1] - self.obstacle_clearance and node.y <= obstacle[1][1] + self.obstacle_clearance:
                no_collision = False
        
        # Check if new node and its parent node intersect obstacles
        if no_collision == True:
            if self.obstacle_intersection(node):
                no_collision = False
        
        return no_collision

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list)
        r = self.search_radius
        # r = 50.0 * np.sqrt((np.log(nnode) / nnode))
        if self.kd_tree is None:
            nodes = [(node.x, node.y) for node in self.node_list]
            kdtree = KDTree(nodes)
            self.kd_tree = kdtree
        idxs = self.kd_tree.query_ball_point([new_node.x, new_node.y], r)
        return [self.node_list[i] for i in idxs]

    def choose_parent(self, new_node, near_nodes):
        if not near_nodes:
            return new_node

        dlist = []
        for near_node in near_nodes:
            t_node = self.steer(near_node, new_node)
            if self.check_collision(t_node):
                dlist.append(near_node.cost + self.calc_distance_and_angle(near_node, new_node)[0])
            else:
                dlist.append(float("inf"))

        min_cost = min(dlist)
        min_cost_node = near_nodes[dlist.index(min_cost)]
        new_node.cost = min_cost
        new_node.parent = min_cost_node
        min_cost_node.children_count += 1
        return new_node

    def rewire(self, new_node, near_nodes,):
        for near_node in near_nodes:
            edge_node = self.steer(new_node, near_node)
            if not self.check_collision(edge_node):
                continue

            edge_node_cost = new_node.cost + self.calc_distance_and_angle(new_node, near_node)[0]

            if edge_node_cost < near_node.cost:
                near_node.parent.children_count -= 1
                assert(near_node.parent.children_count >= 0)
                near_node.parent = new_node
                near_node.cost = edge_node_cost
                new_node.children_count += 1

    def generate_best_path(self):
        best_path = []
        lowest_cost = float("inf")
        for node in self.solution_nodes:
            path_cost = 0
            path = []
            while node.parent is not None:
                path.append([node.x, node.y])
                path_cost += node.cost
                node = node.parent
            if path_cost < lowest_cost:
                best_path = path
                lowest_cost = path_cost

        best_path.append([self.start.x, self.start.y])
        return best_path, lowest_cost

    def calc_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = np.hypot(dx, dy)
        theta = np.arctan2(dy, dx)
        return d, theta

    def calc_dist_to_goal(self, x, y):
        return np.hypot(self.goal.x - x, self.goal.y -y)

    def draw_graph(self, ax, path=None, clear=False):
        ax.clear()

        # Plot obstacles
        for obstacle in self.obstacles:
            rect = plt.Rectangle((obstacle[0][0], obstacle[0][1]), obstacle[1][0] - obstacle[0][0], obstacle[1][1] - obstacle[0][1], color='black')
            ax.add_patch(rect)
        
        # Plot edges
        for node in self.node_list:
            if node.parent:
                ax.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g")
        
        # Plot nodes
        for node in self.node_list:
            circle = plt.Circle((node.x, node.y), radius=0.5, color='b')
            ax.add_patch(circle)
        
        if path:
            ax.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        ax.plot(self.start.x, self.start.y, "xr")
        ax.plot(self.goal.x, self.goal.y, "xr")
        ax.set_xlim(0, self.map_size[0])
        ax.set_ylim(0, self.map_size[1])
        ax.grid(True)
        plt.pause(0.01)

if __name__ == '__main__':
    # Example usage:
    start = (0, 30)
    goal = (59, 30)
    map_size = (60, 60)
    rrt_star = RRTStar(start, goal, map_size)
    rrt_star.add_obstacle((20,20),(30,30))
    rrt_star.add_obstacle((30,40),(40,50))
    rrt_star.add_obstacle((40,30),(50,40))
    #node1 = Node(20,20)
    #node1.parent = Node(25,25)
    #print(rrt_star.obstacle_intersection(node1))
    path = rrt_star.plan()

    #plt.show()

'''
Constrained Kinematic-aware RRT Sampling Planning

1. Initialize a Start node and goal node, each node will consist of X Y and heading position, discretize the possible headings in the space, (e.g. 16 possible headings 22.5 degrees apart)
2. Initialize a list of acceptable headings using start node's heading +/- feasible heading angles
2. Generate a random node at a random X,Y position with a random discrete heading within acceptable bounds that is not inside an obstacle
3. Find closest node in current tree 
4. Check for any path collisions to closest node if collision goto step 2
5. Check if the heading is feasible from the closest node, if not, goto step 2
6. Check if new node's position is within range of the goal and is within feasible headings of the closest node, if so, update the final feasible path or terminate
6. Steer closest node to new node by the step size and add it to the current tree.
7. Append the new node's heading to the feasible list
8. Goto step 2


Hybrid A*/Grid search Planning

Generate a grid of points of some set density across the space
Run A* across the grid, between each point 
https://www.youtube.com/watch?v=hXTnWN8NiKE&t=64s

'''


