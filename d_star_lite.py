#!/usr/bin/env python3

import numpy as np
import math
import matplotlib.pyplot as plt
import heapq

from pprint import pprint

class Node(object):
    def __init__(self, x, y, cost):
        self.x = x
        self.y = y
        self.cost = cost
        self.rhs = math.inf
        self.g = math.inf

    def __repr__(self):
        return "<x: %s, y: %s, cost: %s, rhs: %s, g: %s>" % (self.x, self.y, self.cost, self.rhs, self.g)


class DStarLite(object):
    def __init__(self):
        self.queue = []# U
        heapq.heapify(self.queue)
        self.successors = []
        self.predecessors = []
        self.motion = [[-1, -1],
                       [-1,  0],
                       [-1,  1],
                       [ 0, -1],
                       [ 0,  1],
                       [ 1, -1],
                       [ 1,  0],
                       [ 1,  1]]

    def get_heuristics(self, state, goal):
        # from a_star.py
        return math.sqrt((state.x - goal.x)**2 + (state.y - goal.y)**2);

    def get_g_value(self, state):
        return 0

    def get_rhs(self, state):
        if state == self.start:
            return 0
        else:
            return self.get_g_value(state) + self.get_cost(state, state)

    def get_cost(self, state, state_):
        return state_.cost

    def calculate_key(self, state):
        # (k1(s), k2(s)): k1(s) = (min(g(s), rhs(s)+h(s, sg))), k2(s) = min(g(s), rhs(s))
        g_s = self.get_g_value(state)
        state.rhs = self.get_rhs(state)
        return (min(g_s, state.rhs) + self.get_heuristics(state, self.goal), min(g_s, state.rhs))

    def initialize(self, start, goal):
        self.start = Node(start[0], start[1], 0)
        self.goal = Node(goal[0], goal[1], 0)
        self.queue = []
        self.queue.append([self.start, self.calculate_key(self.start)])
        print(self.queue)

    def update_vertex(self, u):
        if self.start != state:
            u.rhs = self.get_min_pred_cost(u.rhs)
        if u.rhs in self.queue:
            pass
        if self.get_g_value(u.rhs) != u.rhs:
            pass

    def compute_shortest_path(self):
        while (self.get_top_key(u) < self.calculate_key(self.goal)) or self.goal.rhs != self.goal.g:
            u = self.pop()
            if u.g > u.rhs:
                u.g = u.rhs
                for s in self.get_successors(u):
                    self.update_vertex(u)
            else:
                u.g = math.inf
                for s in self.get_successors(u):
                    self.update_vertex(u)

    def get_min_pred_cost(self, state):
        predecessors = []
        for m in self.motion:
            n = Node(state.x + m[0], state.y + m[1], math.sqrt(m[0]**2 + m[1]**2))
            if self.verify_node(n):
                predecessors.append(n)

    def get_successors(self, state):
        successors = []
        for m in self.motion:
            n = Node(state.x + m[0], state.y + m[1], math.sqrt(m[0]**2 + m[1]**2))
            if self.verify_node(n):
                successors.append(n)
        return successors

    def get_top_key(self, u):
        pass

    def pop(self):
        u = heapq.heappop(self.queue)
        return u

    def calc_xyindex(self, position, min_pos):
        # from a_star.py
        return round((position - min_pos) / self.reso)

    def calc_grid_index(self, node):
        # from a_star.py
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        # from a_star.py
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obmap[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy, reso, rr):
        # from a_star.py
        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))
        self.reso = reso
        self.rr = rr
        print("minx:", self.minx)
        print("miny:", self.miny)
        print("maxx:", self.maxx)
        print("maxy:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.reso)
        self.ywidth = round((self.maxy - self.miny) / self.reso)
        print("xwidth:", self.xwidth)
        print("ywidth:", self.ywidth)

        # obstacle map generation
        self.obmap = [[False for i in range(self.ywidth)]
                      for i in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(self.ywidth):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.sqrt((iox - x) ** 2 + (ioy - y) ** 2)
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
                        break

    def calc_grid_position(self, index, minp):
        """
        calc grid position

        :param index:
        :param minp:
        :return:
        """
        pos = index * self.reso + minp
        return pos

if __name__=='__main__':
    dsl = DStarLite()
    start = (10, 10)
    goal = (50, 50)
    dsl.initialize(start, goal)

    # obstacles
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)
    resolution = 0.1
    xwidth = round((max(ox) - min(ox)) / resolution)
    ywidth = round((max(oy) - min(oy)) / resolution)

    dsl.calc_obstacle_map(ox, oy, 1, 0.3)

    pprint(dsl.get_successors(dsl.start))

    plt.plot(ox, oy, ".k")
    # obstacle_map = [[False for i in range(xwidth)] for j in range(ywidth)]

    plt.show()
