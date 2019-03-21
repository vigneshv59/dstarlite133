import math
import heapq
import sys

from pqueue_remove import PriorityQueue
from grid import Grid, INT_MAX
from point import Point

class DStar:
    """
    Class representing the DStar algorithm and it's basic functionality.
    Path plans from a robot starting position to a goal given a graph,
    and updates the robots position as it moves.
    """

    def __init__(self, graph, s_start, s_goal):
        """
        Initializes the state of the D* Lite algorithm by creating the
        priority queue and G and RHS arrays, and initializing them appropriately,
        adding the goal to the queue (as the only locally inconsistent vertex).
        """

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
        """
        The heuristic is the euclidean distance between the start and end.
        """
        return math.sqrt((end.y - start.y) ** 2 + (end.x - start.x) ** 2)

    # Convenience setters and getters for G and RHS arrays.
    def get_g(self, p):
        return self.g[p.x][p.y]

    def set_g(self, p, new_val):
        self.g[p.x][p.y] = min(new_val, INT_MAX)

    def get_rhs(self, p):
        return self.rhs[p.x][p.y]

    def set_rhs(self, p, new_val):
        self.rhs[p.x][p.y] = min(new_val, INT_MAX)

    def calculate_key(self, s):
        """
        Computes the key of the priority queue based on the heuristic and the
        current lookahead value in RHS and the current distance value in G.
        Adds km to the heuristic, since this is the bound on the heuristic error
        by the triangle inequality assumption of the heuristic.
        """
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

            # Update the RHS array to be the minimium d + g of the children
            # of the vertex.
            self.set_rhs(u, min_cost)

        # Since we are updating this vertex, we remove this vertex.
        self.queue.remove(u)

        # This vertex is locally inconsistent, so by the invariants of the
        # algorithm, this must be added to the PQ.
        if (self.get_g(u) != self.get_rhs(u) and
            (self.get_g(u) < INT_MAX or self.get_rhs(u) < INT_MAX)):
            self.queue.enqueue((self.calculate_key(u), u))


    def compute_shortest_path(self):
        """
        Runs the compute shortest path step of the D* Lite algorithm.
        """

        # If the start vertex is locally inconsistent, or the first element in the
        # queue is less than the PQ element of the start vertex, we want to continue
        # finding a path by updating g.
        # If the queue is empty, there's no locally inconsistent vertices, and we're done.
        while len(self.queue) > 0 and (self.queue.peek()[0] < self.calculate_key(self.start)
            or self.get_rhs(self.start) != self.get_g(self.start)):

            # Dequeue the lowest vertex on the priority queue
            k_old, u = self.queue.dequeue()
            k_new = self.calculate_key(u)

            # This vertex has a larger cost than that currently on the queue,
            # we need to add it back to the queue, since its' priority has changed.
            if k_old < k_new:
                self.queue.enqueue((k_new, u))
            elif self.get_g(u) > self.get_rhs(u):
                # If this vertex is locally overconsistent, we need to fix it
                # by ensuring G = RHS, making it locally consistent. Then,
                # it can be removed from the queue, but since this can change
                # local consistency of children, we update these vertices.

                self.set_g(u, self.get_rhs(u))
                self.queue.remove(u)
                for c in self.graph.children(u, True):
                    if (c != self.goal):
                        self.set_rhs(c, min(self.get_rhs(c),
                            self.graph.get_distance(c, u) + self.get_g(u)))

                    self.update_vertex(c)
            else:
                # If the vertices are locally underconsistent, we can set g
                # to be infinity, making this vertex locally overconsistent.
                g_old = self.get_g(u)
                self.set_g(u, INT_MAX)

                # Update the children of this vertex, and the vertex itself,
                # since this vertex and its children may not be locally consistent.
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
        """
        Runs the main loop for the D*-lite algorithm.
        """

        # Initialize the last cell, and generate a first pass G matrix with no
        # detected obstacles.
        s_last = self.start
        self.compute_shortest_path()

        # Continue to pathfind until we reach the goal.
        while (self.start != self.goal):
            # Draw the state to the canvas
            self.graph.draw()

            # If the the start vertex is an infinite distance from the goal,
            # it is unreachable.
            if self.get_g(self.start) >= INT_MAX:
                return False

            min_dist = INT_MAX
            min_node = None

            # Pick the lowest cost vertex to visit next, and move there.
            for c in self.graph.children(self.start):
                d = self.graph.get_distance(self.start, c)
                dist = d + self.get_g(c)

                if dist < min_dist:
                    min_dist = dist
                    min_node = c

            self.start = min_node

            changes = self.graph.move_to(self.start)

            # If any new obstacles were found, we update the given obstacle-edge,
            # and update the km heuristic bound appropriately. We then recalculate
            # the shortest path.

            if len(changes) != 0:
                self.km += self.heuristic(s_last, self.start)
                s_last = self.start

                for ch in changes:
                    self.graph.edges[ch[0].x][ch[0].y][ch[1]] = INT_MAX

                    self.update_vertex(ch[0])

                self.compute_shortest_path()

        grid.draw()

        # If the loop ended, a successful path was found.
        return True


if __name__ == '__main__':
    import maze_gen

    if len(sys.argv) != 2:
        print("Usage: dstarlite.py [maze_num] \n The maze_num maps as follows. \n Small maze: 0 \n Large maze: 1 \n No path: 2")
        sys.exit(1)

    maze_num = int(sys.argv[1])

    # Use the maze generator class to create a set of obstacles for the robot.
    # There are three mazes:

    size, start, end, obstacles = None, None, None, None

    if maze_num == 0:
        print("Running the small maze simulation")
        size, start, end, obstacles = maze_gen.small_maze() # This maze is a small, true maze
    elif maze_num == 1:
        print("Running the large maze simulation")
        size, start, end, obstacles = maze_gen.large_maze() # This maze is a large, obstacle environment
    elif maze_num == 2:
        print("Running the impossible maze simulation")
        size, start, end, obstacles = maze_gen.impossible_maze() # This maze is a large, obstacle environment, but without a path from start to goal

    # Initialize the objects for the algorithm to work appropriately.
    goal = Point(end[0], end[1])
    grid = Grid(size[0], size[1], obstacles, goal, Point(start[0], start[1]))
    dstar = DStar(grid, Point(start[0], start[1]), goal)

    result = dstar.run()

    if result:
        print("Reached the goal!")
    else:
        print("Failed to find a path to the goal, it was not reachable.")
