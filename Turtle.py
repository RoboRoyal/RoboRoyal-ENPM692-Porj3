
# Dakota Abernathy
# Neha Saini
# ENPM 661
# Project3-phase3


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

BOT_RADIUS = 10.5  # 105mm
OBSTACLE_CLEARANCE = 5
CLEARANCE = BOT_RADIUS + OBSTACLE_CLEARANCE
THRESHOLD = 12
nodes_visited = []
actions = [55, 57, 0]
path = []
SQRT2 = math.sqrt(2)
nodes = None
found_path = True

r = 0.038
L = 0.354
dt = 0.1
stop_time = 6


# distance between two points
def distance(x1, y1, x2, y2):
    return math.sqrt(pow((x2-x1), 2)+pow((y2-y1), 2))


def round_to_n(num, n=4):
    return n * round(num / n)


# class to keep track of each place visited
class Node:
    def __init__(self, x, y, theta, end_x=0, end_y=0, parent=None, D = None, L = None, R = None, end_theta = 0):
        self.x = int(x)
        self.y = int(y)
        self.end_x = int(end_x)
        self.end_y = int(end_y)
        self.parent = parent
        self.theta = int(theta + .5) % 360
        self.end_theta = int(end_theta + .5) % 360
        if parent:
            self.path_length = parent.path_length + D
            self.L = L
            self.R = R
        else:
            self.path_length = 0
            self.L = 0
            self.R = 0
        if target:
            self.h = self.heuristic()
        else:
            self.h = 0

    def heuristic(self):  # a* heuristic
        return math.sqrt(math.pow(target.x - self.end_x, 2) + math.pow(target.y - self.end_y, 2)) + self.path_length / 1.1

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return "[" + str(round_to_n(self.end_x)) + ", " + str(round_to_n(self.end_y)) + ", " \
               + str(round_to_n(self.end_theta, 10))
               #+ ", " + str(self.end_x)+", "+str(self.end_y) + "]"

    def __lt__(self, other):
        return self.path_length < other.path_length


def draw_node_diff(node, color=CYAN):
    plot_curve(node.x, node.y, node.theta, node.L, node.R, color)


def get_neighbors_rigid_diff(node):
    neighbors = []
    # actions = [20, 22]
    for left in actions:
        for right in actions:
            if right == left == 0:
                continue
            Xn, Yn, Tn, D = move_curve(node.end_x, node.end_y, node.end_theta, left, right)
            if point_valid(Xn, Yn, False):
                neighbors.append(Node(node.end_x, node.end_y, node.end_theta, Xn, Yn, node, D, left, right, Tn))
    return neighbors


# returns a randomly-generated node
def random_node():
    point = random_point()
    node = Node(point[0], point[1], 0, point[0], point[1])
    new_nodes = get_neighbors_rigid_diff(node)
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
    global OBSTACLE_CLEARANCE
    if human:
        x1, y1 = get_point_from_user("start")
        x2, y2 = get_point_from_user("target")
        theta = int(input("Input starting theta in degrees: ")) % 360
        get_diff()
        OBSTACLE_CLEARANCE = int(input("Enter clearance for obstacles"))
    else:
        x1, y1 = random_point()
        x2, y2 = random_point()
        theta = random.randint(0, 11) * 30  # 30, 60, 90, etc
    return Node(x1, y1, theta, x1, y1), Node(x2, y2, 0, x2, y2)


# a* search
def turtle_a_star():
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
            draw_node_diff(next_node, CYAN)
            if itt % 50 == 0:
                pygame.display.update()
                pygame.event.get()

        closed[str(next_node)] = 1

        nodes_visited.append(next_node)
        if is_close(next_node.end_x, next_node.end_y, target.x, target.y):  # check if done
            target.parent = next_node
            found_path = True
            return True

        neighbors = get_neighbors_rigid_diff(next_node)

        for neighbor in neighbors:  # get neighbors and check if they have been checked yet
            if str(neighbor) in closed or str(neighbor) in open_check:
                continue
            open_list.put((neighbor.heuristic(), neighbor))
            open_check[str(neighbor)] = 1

    found_path = False
    return False


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
    draw_point_with_threshold(start, GREEN)
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
        # draw_node(point, CYAN)
        draw_node_diff(point)
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
        # draw_node(point, MAGENTA)
        draw_node_diff(point, MAGENTA)
        pygame.display.update()
        pygame.event.get()
        clock.tick(12)
    pygame.display.update()
    if path:
        print("Path length: ", path[-2].path_length)


def move_curve(Xi, Yi, Thetai, UL, UR):
    t = 0
    Xn = Xi
    Yn = Yi
    Thetan = 3.14 * Thetai / 180
    D = 0
    while t < stop_time:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += (0.5 * r * (UL + UR) * math.cos(Thetan) * dt)
        Yn += (0.5 * r * (UL + UR) * math.sin(Thetan) * dt)
        Thetan += (r / L) * (UR - UL) * dt
        D += distance(Xs, Ys, Xn, Yn)
    Thetan = 180 * Thetan / 3.14
    return Xn, Yn, Thetan, D


def plot_curve(Xi, Yi, Thetai, UL, UR, color = CYAN):
    t = 0
    Xn = Xi
    Yn = Yi
    Thetan = 3.14 * Thetai / 180
    while t < stop_time:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += (0.5 * r * (UL + UR) * math.cos(Thetan) * dt)
        Yn += (0.5 * r * (UL + UR) * math.sin(Thetan) * dt)
        Thetan += (r / L) * (UR - UL) * dt
        pygame.draw.line(board, color, [Xs * SCALE, (HEIGHT - Ys) * SCALE], [Xn * SCALE, (HEIGHT - Yn) * SCALE])
    return Xn, Yn


def node_check():
    n = Node(50, 50, 0)
    Xn, Yn = plot_curve(n.x, n.y, n.theta, 50, 60, RED)
    pygame.draw.circle(board, RED, [Xn, HEIGHT - Yn], 2)
    pygame.display.update()
    for i in range(501):
        time.sleep(.1)
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                raise SystemExit


def get_diff():
    print("This system works best with values in the range of 30-60")
    print("The values should not differ more than 5, and work best with a difference of 2 or 3")
    actions[0] = int(input("Enter the smaller value: "))
    actions[1] = int(input("Enter the larger value: "))


if __name__ == "__main__":
    mode = 1
    start, target = get_initial_conditions(False)
    print("Finding path...")
    real_time = False

    if real_time:
        make_board()
        add_points()

    if turtle_a_star():
        print("Path found")
    else:
        print("No path found")

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
    raise SystemExit
