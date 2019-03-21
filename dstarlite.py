import math
import heapq
import graphics
import time

from pqueue_remove import PriorityQueue

INT_MAX = 10000
VISIBILITY = 1
WINDOW_WIDTH = 600

class Point:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def to_graphics_rect(self, width_scale, height_scale):
        top_left = graphics.Point(self.x * width_scale, self.y * height_scale)
        top_right = graphics.Point((self.x + 1) * width_scale, (self.y + 1) * height_scale)

        return graphics.Rectangle(top_left, top_right)

    def __str__(self):
        return "({}, {})".format(self.x, self.y)

    def __repr__(self):
        return str(self)

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        return isinstance(other, Point) and self.x == other.x and self.y == other.y

    def __lt__(self,other):
        return False

class Grid:
    def __init__(self, width, height, obstacles, goal):
        self.width = width
        self.height = height
        self.win = None
        self.robot_elem = None
        self.robot_pos = None
        self.goal = goal

        # Required for fast draw
        self.to_draw_cells = []

        self.edges = [[{} for i in range(height)] for j in range(width)]
        self.obstacles = dict((Point(o[0], o[1]), 0) for o in obstacles)

        for i in range(width):
            for j in range(height):
                surrounding_cells = []
                for o1 in [-1, 0, 1]:
                    for o2 in [-1, 0, 1]:
                        if o1 == 0 and o2 == 0:
                            continue

                        if i + o1 < 0 or j + o2 < 0:
                            continue

                        if i + o1 >= width or j + o2 >= height:
                            continue

                        surrounding_cells.append(Point(i + o1, j + o2))

                for c in surrounding_cells:
                    self.edges[i][j][c] = math.sqrt((c.y - j) ** 2 + (c.x - i) ** 2)

        self.move_to(Point(0, 0), True)

    def get_visible_obstacles(self):
        obstacles = []

        for k, v in self.obstacles.items():
            if v == 1:
                obstacles.append(k)

        return obstacles

    def children(self, point, show_obstacles=False):
        children = []

        for point, dist in self.edges[point.x][point.y].items():
            if show_obstacles or dist < INT_MAX:
                children.append(point)

        return children

    def get_distance(self, fromp, to):
        return self.edges[fromp.x][fromp.y].get(to, INT_MAX)

    def move_to(self, position, add_edge=False):
        self.robot_pos = position

        visible_cells = self.get_visible_cells(position, 0)
        updated = []

        for obs, seen in self.obstacles.items():
            if obs not in visible_cells or seen != 0:
                continue

            self.obstacles[obs] = 1
            self.to_draw_cells.append(obs)

            neighbors = self.children(obs)

            for n in neighbors:
                updated.append((n, obs))
                updated.append((obs, n))

                if add_edge:
                    self.edges[n.x][n.y][obs] = INT_MAX
                    self.edges[obs.x][obs.y][n] = INT_MAX


        return updated

    def get_visible_cells(self, position, depth):
        if depth >= VISIBILITY:
            return []

        visible_cells = set(self.children(position))
        neighbors = self.children(position)

        for cell in neighbors:
            child_neighbors = self.get_visible_cells(cell, depth + 1)
            for child_cell in child_neighbors:
                if child_cell not in visible_cells and child_cell != position:
                    visible_cells.add(child_cell)

        return visible_cells

    def draw(self):
        window_height = WINDOW_WIDTH * (self.height / self.width)
        width_scale = WINDOW_WIDTH / self.width
        height_scale = window_height / self.height

        if self.win is None:
            self.win = graphics.GraphWin('SimSpace', WINDOW_WIDTH, window_height)
            pt = self.robot_pos.to_graphics_rect(width_scale, height_scale)
            pt.draw(self.win)
            pt.setFill('red')
            pt.setOutline('red')

            pt2 = self.goal.to_graphics_rect(width_scale, height_scale)
            pt2.draw(self.win)
            pt2.setFill('grey')
            pt2.setOutline('grey')


            self.robot_elem = pt
        else:
            self.robot_elem.setFill('blue')
            self.robot_elem = self.robot_pos.to_graphics_rect(width_scale, height_scale)
            self.robot_elem.draw(self.win)
            self.robot_elem.setFill('red')


        for o in self.to_draw_cells:
            r = o.to_graphics_rect(width_scale, height_scale)
            r.draw(self.win)
            r.setFill('black')

        self.to_draw_cells = []

        time.sleep(0.1)

class DStar:

    def __init__(self, graph, s_start, s_goal):
        self.queue = PriorityQueue()
        self.km = 0
        self.graph = graph
        self.start = s_start
        self.goal = s_goal
        self.g = [[INT_MAX for i in range(graph.height)] for j in range(graph.width)]
        self.rhs = [[INT_MAX for i in range(graph.height)] for j in range(graph.width)]
        self.set_rhs(self.goal, 0)
        self.queue.enqueue((self.calculate_key(self.goal), self.goal))

    def heuristic(self, start, end):
        return math.sqrt((end.y - start.y) ** 2 + (end.x - start.x) ** 2)

    def get_g(self, p):
        return self.g[p.x][p.y]

    def set_g(self, p, new_val):
        self.g[p.x][p.y] = min(new_val, INT_MAX)

    def get_rhs(self, p):
        return self.rhs[p.x][p.y]

    def set_rhs(self, p, new_val):
        self.rhs[p.x][p.y] = min(new_val, INT_MAX)

    def calculate_key(self, s):
        curr_best = min(self.get_g(s), self.get_rhs(s))
        heur = self.heuristic(self.start, s)
        priority = (curr_best + heur + self.km, curr_best)
        return priority

    def update_vertex(self, u):
        if u != self.goal:
            min_cost = INT_MAX
            for succ in self.graph.children(u, True):
                cost = self.graph.get_distance(u, succ) + self.get_g(succ)
                min_cost = min(min_cost, cost)
            self.set_rhs(u, min_cost)

        self.queue.remove(u)

        if (self.get_g(u) != self.get_rhs(u) and
            (self.get_g(u) < INT_MAX or self.get_rhs(u) < INT_MAX)):
            self.queue.enqueue((self.calculate_key(u), u))


    def compute_shortest_path(self):
        while (len(self.queue) > 0 and self.queue.peek()[0] < self.calculate_key(self.start)
            or self.get_rhs(self.start) != self.get_g(self.start)):

            k_old, u = self.queue.dequeue()
            k_new = self.calculate_key(u)

            if k_old < k_new:
                self.queue.enqueue((k_new, u))
            elif self.get_g(u) > self.get_rhs(u):
                self.set_g(u, self.get_rhs(u))
                self.queue.remove(u)
                for c in self.graph.children(u, True):
                    if (c != self.goal):
                        self.set_rhs(c, min(self.get_rhs(c),
                            self.graph.get_distance(c, u) + self.get_g(u)))

                    self.update_vertex(c)
            else:
                g_old = self.get_g(u)
                self.set_g(u, INT_MAX)

                for c in self.graph.children(u, True) + [u]:
                    if (self.get_rhs(c) ==
                        self.graph.get_distance(c, u) + self.get_g(u)):

                        if (c != self.goal):
                            min_c = INT_MAX
                            for cp in self.graph.children(c, True):
                                min_c = min(min_c, self.graph.get_distance(c, cp) + self.get_g(cp))

                            self.set_rhs(c, min_c)

                    self.update_vertex(c)


    def run(self):
        s_last = self.start
        self.compute_shortest_path()

        while (self.start != self.goal):
            self.graph.draw()

            if self.get_g(self.start) >= INT_MAX:
                return False

            min_dist = INT_MAX
            min_node = None

            for c in self.graph.children(self.start):
                d = self.graph.get_distance(self.start, c)
                dist = d + self.get_g(c)

                if dist < min_dist:
                    min_dist = dist
                    min_node = c

            self.start = min_node

            changes = self.graph.move_to(self.start)

            if len(changes) != 0:
                self.km += self.heuristic(s_last, self.start)
                s_last = self.start

                for ch in changes:
                    self.graph.edges[ch[0].x][ch[0].y][ch[1]] = INT_MAX

                    self.update_vertex(ch[0])

                self.compute_shortest_path()

obstacles = []

for i in range(25, 27):
    for j in range(75):
        obstacles.append((i, j))

for i in range(35, 36):
    for j in range(25):
        obstacles.append((i, j))

for i in range(33, 38):
    for j in range(25, 50):
        obstacles.append((i, j))


# for i in range(1):
#     obstacles.append((1, i))
#
# for i in range(2):
#     obstacles.append((3, i))

def get_maze():
    obstacles = [(0, 7), (0, 6), (1, 0),
    (1, 1), (1, 2), (1, 3), (1, 4), (1, 6), (2, 6), (3, 1), (3, 2), (3, 3), (3, 4),
    (3, 6), (4, 4), (4, 6), (5, 0), (5, 2), (5, 4), (6, 0), (6, 2), (6, 4), (6, 5),
    (6, 6), (7, 0), (7, 2), (7, 6), (8, 0), (8, 2), (8, 4), (8, 6), (9, 0), (9, 1),
    (9, 4), (9, 5), (9, 6), (8, 1)]

    return obstacles

obstacles = set(get_maze())
goal = Point(9, 2)

grid = Grid(10, 7, obstacles, goal)
dstar = DStar(grid, Point(0, 0), goal)

dstar.run()
grid.draw()
