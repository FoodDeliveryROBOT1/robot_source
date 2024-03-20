import numpy as np
import matplotlib.pyplot as plt
import math
import heapq
import random
import time
from typing import Tuple

p_create_random_obstacle = 0
class Node_:
    def __init__(self, x: int = 0, y: int = 0, cost: float = 0.0):
        """
        x:
        y: 
        cost:
        """
        self.x = x
        self.y = y
        self.cost = cost

def add_coordinates(node1: Node_, node2: Node_):
    new_node = Node_()
    new_node.x = node1.x + node2.x
    new_node.y = node1.y + node2.y
    new_node.cost = node1.cost + node2.cost
    return new_node


def compare_coordinates(node1: Node_, node2: Node_):
    return node1.x == node2.x and node1.y == node2.y


class DStarLite:
    #Possible motions
    motions = [
        Node_(1, 0, 1),
        Node_(0, 1, 1),
        Node_(-1, 0, 1),
        Node_(0, -1, 1),
        Node_(1, 1, math.sqrt(2)),
        Node_(1, -1, math.sqrt(2)),
        Node_(-1, 1, math.sqrt(2)),
        Node_(-1, -1, math.sqrt(2))
    ]
    def __init__(self, ox: list, oy: list):
        # self.x_min_world = int(min(ox))
        # self.y_min_world = int(min(oy))
        # self.x_max = int(abs(max(ox) - self.x_min_world))
        # self.y_max = int(abs(max(ox) - self.y_min_world))
        self.x_min_world = 0
        self.y_min_world = 0
        self.x_max = 500
        self.y_max = 500
        print(self.x_max, self.y_max)
        self.obstacles = [Node_(x - self.x_min_world, y - self.y_min_world) for x, y in zip(ox, oy)]
        self.obstacles_xy = np.array(
        [[obstacle.x, obstacle.y] for obstacle in self.obstacles]
        )
        self.start = Node_(0,0)
        self.goal = Node_(0,0)
        self.U = list()
        self.km = 0.0
        self.kold = 0.0
        self.rhs = self.create_grid(float("inf"))
        self.g = self.create_grid(float("inf"))
        self.detected_obstacles_xy = np.empty((0,2))
        self.initialized = False
        
        
        
    def create_grid(self, val: float):
        return np.full((self.x_max, self.y_max), val)
    
    def is_obstacles(self, node: Node_):
        x = np.array([node.x])
        y = np.array([node.y])
        
        obstacle_x_equal = self.obstacles_xy[:, 0] == x
        obstacle_y_equal = self.obstacles_xy[:, 1] == y
        is_in_obstacles = (obstacle_x_equal & obstacle_y_equal).any()
        
        is_in_detected_obstacles = False
        if self.detected_obstacles_xy.shape[0] > 0:
            is_x_equal = self.detected_obstacles_xy[:, 0] == x
            is_y_equal = self.detected_obstacles_xy[:, 1] == y
            is_in_detected_obstacles = (is_x_equal & is_y_equal).any()
            
        return is_in_obstacles or is_in_detected_obstacles
    
    def c(self, node1: Node_, node2: Node_):
        if self.is_obstacles(node2):
            # Attempting to move from or to an obstacle
            return math.inf
        new_node = Node_(node1.x-node2.x, node1.y-node2.y)
        detected_motion = list(filter(lambda motion:
                                      compare_coordinates(motion, new_node),
                                      self.motions))
        return detected_motion[0].cost
    
    def h(self, s: Node_):
        return math.hypot(self.start.x - s.x, self.start.y - s.y)
    
    def calculate_key(self, s: Node_):
        return (min(self.g[s.x][s.y], self.rhs[s.x][s.y]) + self.h(s) + self.km, min(self.g[s.x][s.y], self.rhs[s.x][s.y]))
    
    def is_valid(self, node: Node_):
        if 0 <= node.x < self.x_max and 0 <= node.y < self.y_max:
            return True
        return False
    
    def get_neighbours(self, u: Node_):
        return [add_coordinates(u, motion) for motion in self.motions if self.is_valid(add_coordinates(u, motion))]
    
    def pred(self, u:Node_):
        return self.get_neighbours(u)
    
    def succ(self, u:Node_):
        return self.get_neighbours(u)
    
    def initialize(self, start: Node_, goal: Node_):
        self.start.x = start.x - self.x_min_world
        self.start.y = start.y - self.y_min_world
        self.goal.x = goal.x - self.x_min_world
        self.goal.y = goal.y - self.y_min_world
        
        if not self.initialized:
            self.initialized = True, print('Initializing')
            self.U = list()
            self.km = 0.0
            self.rhs = self.create_grid(math.inf)
            self.g = self.create_grid(math.inf)
            
            self.rhs[self.goal.x][self.goal.y] = 0
            
            self.U.append((self.goal, self.calculate_key(self.goal)))
            self.detected_obstacles_xy =np.empty((0,2))
            
            
    def update_vertex(self, u: Node_):
        if not compare_coordinates(u, self.goal):
            self.rhs[u.x][u.y] = min([self.c(u, sprime) + self.g[sprime.x][sprime.y] for sprime in self.succ(u)])
        if any([compare_coordinates(u, node) for node, key in self.U]):
            self.U = [(node, key) for node, key in self.U if not compare_coordinates(node, u)]
            self.U.sort(key=lambda x: x[1])
            
        if self.g[u.x][u.y] != self.rhs[u.x][u.y]:
            self.U.append((u, self.calculate_key(u)))
            self.U.sort(key=lambda x: x[1])
            
    def compare_keys(self, key_pair1: "tuple[float, float]",
                    key_pair2: "tuple[float, float]"
                    ):
        return key_pair1[0] < key_pair2[0] or \
               (key_pair1[0] == key_pair2[0] and key_pair1[1] < key_pair2[1])
    
    def compute_shortest_path(self):
        self.U.sort(key = lambda x: x[1])
        has_elements = len(self.U) > 0
        start_key_not_updated = self.compare_keys(
            self.U[0][1], self.calculate_key(self.start)
        )
        rhs_not_equal_to_g = self.rhs[self.start.x][self.start.y] != self.g[self.start.x][self.start.y]
        
        while has_elements and start_key_not_updated or rhs_not_equal_to_g:
            self.kold = self.U[0][1]
            u = self.U[0][0]
            self.U.pop(0)
            k_new = self.calculate_key(u)
            if self.compare_keys(self.kold, k_new):
                self.U.append((u, k_new))
                self.U.sort(key = lambda x: x[1])
            elif (self.g[u.x, u.y] > self.rhs[u.x, u.y]).any():
                self.g[u.x, u.y] = self.rhs[u.x, u.y]
                for s in self.pred(u):
                    self.update_vertex(s)
            else:
                self.g[u.x, u.y] = math.inf
                for s in self.pred(u):
                    self.update_vertex(s)
            self.U.sort(key=lambda x: x[1])
            start_key_not_updated = self.compare_keys(
                self.U[0][1], self.calculate_key(self.start)
            )
            rhs_not_equal_to_g = self.rhs[self.start.x][self.start.y] != self.g[self.start.x][self.start.y]
                    
            
    def detect_changes(self):
        changed_vertices = list()
        if len(self.spoofed_obstacles) > 0:
            for spoofed_obstacle in self.spoofed_obstacles[0]:
                if compare_coordinates(spoofed_obstacle, self.start) or compare_coordinates(spoofed_obstacle, self.goal):
                    continue
                changed_vertices.append(spoofed_obstacle)
                self.detected_obstacles_xy = np.concatenate((
                    self.detected_obstacles_xy,
                    [[spoofed_obstacle.x, spoofed_obstacle.y]]
                )
                )
                
            self.spoofed_obstacles.pop(0)
        # Allow random generation of obstacles
        random.seed()
        if random.random() > 1 - p_create_random_obstacle:
            x = random.randint(0, self.x_max - 1)
            y = random.randint(0, self.y_max - 1)
            new_obs = Node_(x, y)
            if compare_coordinates(new_obs, self.start) or compare_coordinates(new_obs, self.goal):
                return changed_vertices
            changed_vertices.append(Node_(x, y))
            self.detected_obstacles_xy = np.concatenate(
                (
                    self.detected_obstacles_xy,
                    [[x, y]]
                )
            )
           
        return changed_vertices
    
    def compute_current_path(self):
        path = list()
        current_point = Node_(self.start.x, self.start.y)
        while not compare_coordinates(current_point, self.goal):
            path.append(current_point)
            current_point = min(self.succ(current_point),
                                key=lambda sprime:
                                self.c(current_point, sprime) + self.g[sprime.x][sprime.y]
                               )
        path.append(self.goal)
        return path
    
    def compare_paths(self, path1: list, path2: list):
        if len(path1) != len(path2):
            return False
        for node1, node2 in zip(path1, path2):
            if not compare_coordinates(node1, node2):
                return False
        return True
    
    def main(self, start: Node_, goal: Node_, spoofed_ox: list, spoofed_oy: list):
        # init_ox = [[],[],[],[1]]
        # init_oy = [[],[],[],[1]]
        # self.spoofed_obstacles = [[Node(x - self.x_min_world,
        #                                 y - self.y_min_world)
        #                           for x, y in zip(rowx, rowy)]
        #                          for rowx, rowy in zip(init_ox, init_oy)]
        self.spoofed_obstacles = [[Node_(x - self.x_min_world,
                                        y - self.y_min_world)
                                  for x, y in zip(rowx, rowy)]
                                 for rowx, rowy in zip(spoofed_ox, spoofed_oy)]
        pathx, pathy = [], []
        rx, ry = [], []
        self.initialize(start, goal)
        last = self.start
        self.compute_shortest_path()
        pathx.append(self.start.x + self.x_min_world)
        pathy.append(self.start.y + self.y_min_world)
#         rx.append(self.start.x + self.x_min_world)
#         ry.append(self.start.y + self.y_min_world)
        while not compare_coordinates(self.goal, self.start):
            if self.g[self.start.x][self.start.y]  == math.inf:
                print("No path possible")
                return False, pathx, pathy
            
            self.start = min(self.succ(self.start),
                            key = lambda sprime:
                             self.c(self.start, sprime) + 
                             self.g[sprime.x][sprime.y]
                            )
            pathx.append(self.start.x + self.x_min_world)
            pathy.append(self.start.y + self.y_min_world)
            
            changed_vertices = self.detect_changes()
        
            
            if len(changed_vertices) != 0:
                # print(len(changed_vertices))
                print("Obstacle detected")
                self.km += self.h(last)
                last = self.start
                for u in changed_vertices:
                    if compare_coordinates(u, self.start):
                        continue
                    # print(u.x, '\t', u.y)
                    self.rhs[u.x][u.y] = math.inf
                    self.g[u.x][u.y] = math.inf
                    self.update_vertex(u)
                self.compute_shortest_path()
        print("Path found")
        return True, pathx, pathy, rx, ry