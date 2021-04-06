
# Dakota Abernathy
# Neha Saini
# ENPM 661
# Project3-phase1- Implementation of Dijkstra.


import math
import random
from queue import PriorityQueue
import pygame
import time
from random import randint

HEIGHT = 300
WIDTH = 400
SCALE = 2

board = None
start = None
target = None
real_time = False

WHITE = (255, 255, 255)
BLACK = (0, 0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
CYAN = (0, 255, 255)
MAGENTA = (255, 0, 255)
YELLOW = (255, 255, 0)

BOT_RADIUS = 10
OBSTACLE_CLEARANCE = 5
CLEARANCE = BOT_RADIUS + OBSTACLE_CLEARANCE
THETA = 30
DELTA = .5
MAGNITUDE = 10
THRESHOLD = 10
nodes_visited = []
path = []
SQRT2 = math.sqrt(2)
nodes = None
found_path = True


# distance between two points
def distance(x1, y1, x2, y2):
    return math.sqrt(pow((x2-x1), 2)+pow((y2-y1), 2))


# round to nearest .5
def round_to_half(num):
    decimal = num - int(num)
    if decimal >= .75:
        return int(num) + 1
    elif decimal <= .25:
        return int(num)
    else:
        return int(num) + .5


# class to keep track of each place visited
class Node:
    def __init__(self, x, y, theta, end_x=0, end_y=0, parent=None):
        self.x = x
        self.y = y
        self.end_x = end_x
        self.end_y = end_y
        self.parent = parent
        self.theta = theta
        if parent:
            self.path_length = parent.path_length + MAGNITUDE
            self.g = parent.g + 1
        else:
            self.path_length = 0
            self.g = 0
        if target:
            self.h = self.heuristic()
        else:
            self.h = 0

    def heuristic(self):  # a* heuristic
        return math.sqrt(math.pow(target.x - self.end_x, 2) + math.pow(target.y - self.end_y, 2)) + (self.g * MAGNITUDE)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return "[" + str(self.x) + ", "+str(self.y) + ", " \
               + str(self.theta) + ", " + str(self.end_x)+", "+str(self.end_y) + "]"

    def __lt__(self, other):
        return self.g < other.g


# draws the given node on the board
def draw_node(node, color=CYAN):
    pygame.draw.line(board, color, [node.x * SCALE, (HEIGHT - node.y) * SCALE],
                     [node.end_x * SCALE, (HEIGHT - node.end_y) * SCALE], SCALE)


# gets ll valid neighbors from a given node
def get_neighbors_rigid(node):
    neighbors = []
    for angle in range(-60, 61, THETA):
        new_theta = node.theta + angle
        if new_theta < 0:
            new_theta = new_theta + 360
        else:
            new_theta = new_theta % 360
        new_x = round_to_half(node.end_x + MAGNITUDE * math.cos(math.radians(new_theta)))
        new_y = round_to_half(node.end_y + MAGNITUDE * math.sin(math.radians(new_theta)))
        if point_valid(new_x, new_y, False):
            neighbors.append(Node(node.end_x, node.end_y, new_theta, new_x, new_y, node))
    return neighbors


# returns a randomly-generated node
def random_node():
    point = random_point()
    node = Node(point[0], point[1], 0, point[0], point[1])
    new_nodes = get_neighbors_rigid(node)
    random.shuffle(new_nodes)
    if new_nodes:
        return new_nodes[0]


# draws a single point with a threshold area around it
def draw_point_with_threshold(point, color=GREEN):
    pygame.draw.circle(board, color, [point.x * SCALE, (HEIGHT - point.y) * SCALE], THRESHOLD * SCALE)


# makes default board
def make_board():
    global board
    pygame.init()
    board = pygame.display.set_mode((int(WIDTH * SCALE), int(HEIGHT * SCALE)))
    pygame.display.set_caption("Path finding algorithm")
    board.fill(WHITE)

    # easy
    pygame.draw.circle(board, BLACK, [90 * SCALE, (HEIGHT - 70) * SCALE], 35 * SCALE)
    pygame.draw.ellipse(board, BLACK, [186 * SCALE, (HEIGHT - 175) * SCALE, 120 * SCALE, 60 * SCALE], 0 * SCALE)

    # Line Segment
    pygame.draw.polygon(board, BLACK,
                        [(48 * SCALE, (HEIGHT - 108) * SCALE), (37 * SCALE, (HEIGHT - 124) * SCALE),
                         (159 * SCALE, (HEIGHT - 210) * SCALE), (170 * SCALE, (HEIGHT - 194) * SCALE)])

    # C shape
    pygame.draw.polygon(board, BLACK,  # back
                        [(200 * SCALE, (HEIGHT - 270) * SCALE), (210 * SCALE, (HEIGHT - 270) * SCALE),
                         (210 * SCALE, (HEIGHT - 240) * SCALE), (200 * SCALE, (HEIGHT - 240) * SCALE)])
    pygame.draw.polygon(board, BLACK,  # top
                        [(200 * SCALE, (HEIGHT - 280) * SCALE), (230 * SCALE, (HEIGHT - 280) * SCALE),
                         (230 * SCALE, (HEIGHT - 270) * SCALE), (200 * SCALE, (HEIGHT - 270) * SCALE)])
    pygame.draw.polygon(board, BLACK,  # bottom
                        [(200 * SCALE, (HEIGHT - 240) * SCALE), (230 * SCALE, (HEIGHT - 240) * SCALE),
                         (230 * SCALE, (HEIGHT - 230) * SCALE), (200 * SCALE, (HEIGHT - 230) * SCALE)])

    # Polygon ---- whats the error allowed? lot of rounding and re-rounding
    '''pygame.draw.polygon(board, BLACK,  # why is this so ugly
                        [(354 * SCALE, (HEIGHT - 138) * SCALE), (380 * SCALE, (HEIGHT - 170) * SCALE),
                         (380 * SCALE, (HEIGHT - 115) * SCALE), (328 * SCALE, (HEIGHT - 63) * SCALE),
                         (286 * SCALE, (HEIGHT - 105) * SCALE), (325 * SCALE, (HEIGHT - 143) * SCALE)])'''


# check if point in circle
def in_circle(x, y):
    if math.pow(x - 90, 2) + math.pow(y - 70, 2) >= math.pow(35 + CLEARANCE, 2):
        return False
    return True


# check if point in ellipse
def in_ellipse(x, y):
    center_x = 246
    center_y = 146
    horizontal_axis = 60 + CLEARANCE
    vertical_axis = 30 + CLEARANCE
    if ((math.pow(x - center_x, 2) / math.pow(horizontal_axis, 2)) +
            (math.pow(y - center_y, 2) / math.pow(vertical_axis, 2))) <= 1:
        return True
    return False


# check if point in C-shape
def in_c_shape(x, y):
    if (x >= 200 - CLEARANCE and x <= 210 + CLEARANCE and y <= 280 + CLEARANCE and y >= 230 - CLEARANCE) or \
       (x >= 210 - CLEARANCE and x <= 230 + CLEARANCE and y >= 270 - CLEARANCE and y <= 280 + CLEARANCE) or \
       (y >= 230 - CLEARANCE and y <= 240 + CLEARANCE and x >= 210 - CLEARANCE and x <= 230 + CLEARANCE):
        return True
    return False


# check if point in weird polygon
def in_poly(x, y):
    if ((y - 1 * x + 181.6 - CLEARANCE) < 0 and (y + 0.3 * x - 239.9 - CLEARANCE) < 0
            and (y + 249.2 * x - 95054 - CLEARANCE) < 0 and (y - x + 265 + CLEARANCE) > 0
            and (y + x - 389.3 + CLEARANCE) > 0) or ((y - 1.13 * x + 260.75 - CLEARANCE) < 0
            and (y + 249.2 * x - 95054 - CLEARANCE) < 0 and (y + .3 * x - 240.6 + CLEARANCE) > 0):
        return True
    return False


# check if point in rotated rectangle
def in_line_segment(x, y):
    if (y + 1.4 * x - 176.5 + CLEARANCE) > 0 and (y - 0.7 * x - 74.4 + CLEARANCE) > 0 \
            and (y - 0.7 * x - 98.8 - CLEARANCE) < 0 and (y + 1.4 * x - 438.1 - CLEARANCE) < 0:
        return True
    return False


# check if point is in any obstacle
def in_obstacle(x, y):
    if in_circle(x, y) or in_ellipse(x, y) or in_c_shape(x, y) or \
            in_line_segment(x, y):  # or in_poly(x, y):
        return True
    return False


def is_close(x, y, x_target, y_target):
    return distance(x, y, x_target, y_target) <= THRESHOLD


# check if point inside boundaries and not in any obstacle
def point_valid(x, y, talk=True):
    if x < 0 or x >= WIDTH:
        if talk:
            print("X is outside of boundary [0,", WIDTH, "]")
        return False
    if y < 0 or y > HEIGHT:
        if talk:
            print("Y is outside of boundary [0,", HEIGHT, "]")
        return False
    if in_obstacle(x, y):
        if talk:
            print("Point is inside an obstacle")
        return False
    return True


# checks to ensure visual and mathematical models of obstacles match
def sanity_check():
    node = random_node()
    for run in range(100000):
        draw_node(node)
        pygame.display.update()
        new_nodes = get_neighbors_rigid(node)
        for j in new_nodes:
            draw_node(j, RED)
        random.shuffle(new_nodes)
        if not len(new_nodes):
            time.sleep(10)
        node = new_nodes[0]
        time.sleep(.2)


# gets single valid point from user
def get_point_from_user(word):
    valid = False
    while not valid:
        try:
            x, y = input("Enter the <X> <Y> coordinates of the " + word + " point: ").split()
            x = int(x)
            y = int(y)
            valid = point_valid(x, y, True)
        except:
            print("Enter point as X/Y coordinate pair separated by a space")
    return x, y


# get single valid random point
def random_point():
    valid = False
    while not valid:
        x = randint(0, WIDTH)
        y = randint(0, HEIGHT)
        valid = point_valid(x, y, False)
    return x, y


# gets valid start and target point
def get_initial_conditions(human=True):
    if human:
        x1, y1 = get_point_from_user("start")
        x2, y2 = get_point_from_user("target")
    else:
        x1, y1 = random_point()
        x2, y2 = random_point()
    return Node(x1, y1, 0, x1, y1), Node(x2, y2, 0, x2, y2)


# returns list of nodes of all valid neighbors
def get_neighbors(parent):
    neighbors = []
    for i in range(-1, 2):
        for j in range(-1, 2):
            dist = SQRT2
            if i == j == 0:
                continue
            if point_valid(parent.x + i, parent.y + j, False):
                if i == 0 or j == 0:
                    dist = 1
                new_node = Node(parent.x + i, parent.y + j, parent, dist)
                neighbors.append(new_node)
    return neighbors


# breath-first search
def BFS():
    global found_path
    to_explore = PriorityQueue()
    to_explore.put(start)
    to_explore_check = {str(start): True}  # dict to make faster checks
    explored = {}
    itt = 0
    while not to_explore.empty():
        next_node = to_explore.get()  # get next node
        to_explore_check.pop(str(next_node))
        nodes_visited.append(next_node)
        explored[str(next_node)] = True

        if real_time:  # plot in real time
            itt = itt + 1
            draw_node(next_node)
            if itt % 50 == 0:
                pygame.display.update()
                pygame.event.get()

        if is_close(next_node.end_x, next_node.end_y, target.x, target.y):  # check if done
            print("Found path")
            target.parent = next_node
            return

        new_nodes = get_neighbors_rigid(next_node)  # get neighbors

        for new_node in new_nodes:
            # add them to list to check if they are new
            if str(new_node) not in explored and str(new_node) not in to_explore_check:
                to_explore.put(new_node)
                to_explore_check[str(new_node)] = True
    print("No path")
    found_path = False


# a* search
def a_star():
    global found_path
    open_list = PriorityQueue()
    open_list.put((0, start))
    open_check = {str(start): 1}
    closed = {str(start): 1}
    itt = 0
    while open_list.qsize():
        next_node = open_list.get()[1]
        if real_time:  # plot in real time
            itt = itt + 1
            draw_node(next_node)
            if itt % 50 == 0:
                pygame.display.update()
                pygame.event.get()

        closed[str(next_node)] = True

        nodes_visited.append(next_node)
        if is_close(next_node.end_x, next_node.end_y, target.x, target.y):  # check if done
            print("Found path")
            target.parent = next_node
            return

        neighbors = get_neighbors_rigid(next_node)

        for neighbor in neighbors:  # get neighbors and check if they have been checked yet
            if str(neighbor) in closed or str(neighbor) in open_check:
                continue
            open_list.put((neighbor.heuristic(), neighbor))
            open_check[str(neighbor)] = 1

    found_path = False


# work back from target to get path to start
def back_track():
    n = target
    while n:
        path.append(n)
        n = n.parent
    path.reverse()


# adds all visited nodes, the path, start and end points to board
def add_points():
    print("Visited: ", len(nodes_visited))

    draw_point_with_threshold(start)
    draw_point_with_threshold(target, RED)
    pygame.display.update()
    clock = pygame.time.Clock()
    itt = 0
    time.sleep(.5)

    if len(nodes_visited) > 100000:
        rate = 0
    elif len(nodes_visited) > 1000:
        rate = len(nodes_visited) / 4
    else:
        rate = 200

    for point in nodes_visited:
        draw_node(point, CYAN)
        if itt % 10 == 0:
            draw_point_with_threshold(start)
            draw_point_with_threshold(target, RED)
            pygame.display.update()
            pygame.event.get()
            clock.tick(rate)
        itt = itt + 1

    pygame.display.update()
    draw_point_with_threshold(start)
    draw_point_with_threshold(target, RED)

    print("Path: ", len(path))

    for point in path:
        draw_node(point, MAGENTA)
        pygame.display.update()
        pygame.event.get()
        clock.tick(12)
    pygame.display.update()
    if path:
        print("Path length: ", path[1].path_length)


# used for testing get_neighbors_rigid
def test_n():
    node = random_node()
    node.theta = 180
    pygame.draw.circle(board, CYAN, [node.end_x * SCALE, (HEIGHT - node.end_y) * SCALE], 4 * SCALE)
    new_nodes = get_neighbors_rigid(node)
    for node in new_nodes:
        draw_node(node)
    pygame.display.update()


if __name__ == "__main__":
    mode = 1
    start, target = get_initial_conditions(True)
    print("Finding path...")
    real_time = False

    if real_time:
        make_board()
        add_points()
    if mode == 1:
        a_star()
    elif mode == 2:
        BFS()
    elif mode == 0:
        sanity_check()

    if mode and found_path:
        make_board()
        back_track()
        add_points()
        pygame.display.update()
        print("Done")
        for i in range(501):
            time.sleep(.1)
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.QUIT:
                    raise SystemExit