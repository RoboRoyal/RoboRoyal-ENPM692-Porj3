
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

GRAIN = 30
HEIGHT = 10 * GRAIN
WIDTH = 10 * GRAIN
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

BOT_RADIUS = 1.05  # 105mm
OBSTACLE_CLEARANCE = 5
CLEARANCE = BOT_RADIUS + OBSTACLE_CLEARANCE
THRESHOLD = 6
nodes_visited = []
actions = [40, 42, 0]
path = []
SQRT2 = math.sqrt(2)
nodes = None
found_path = True

r = 0.038
L = 0.354
dt = 0.1
stop_time = 6
SCALAR = GRAIN * SCALE


# distance between two points
def distance(x1, y1, x2, y2):
    return math.sqrt(pow((x2-x1), 2)+pow((y2-y1), 2))


def round_to_n(num, n=3):
    return n * round(num / n)


# class to keep track of each place visited
class Node:
    def __init__(self, x, y, theta, end_x=0, end_y=0, parent=None, dist=None, ul=None, ur=None, end_theta=0):
        self.x = int(x)
        self.y = int(y)
        self.end_x = int(end_x)
        self.end_y = int(end_y)
        self.parent = parent
        self.theta = int(theta + .5) % 360
        self.end_theta = int(end_theta + .5) % 360
        if parent:
            self.path_length = parent.path_length + dist
            self.L = ul
            self.R = ur
        else:
            self.path_length = 0
            self.L = 0
            self.R = 0
        if target:
            self.h = self.heuristic()
        else:
            self.h = 0

    def heuristic(self):  # a* heuristic
        return math.sqrt(math.pow(target.x - self.end_x, 2) + math.pow(target.y - self.end_y, 2)) + self.path_length / 1

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return "[" + str(round_to_n(self.end_x)) + ", " + str(round_to_n(self.end_y)) + ", " \
               + str(round_to_n(self.end_theta, 10))

    def __lt__(self, other):
        return self.path_length < other.path_length


def draw_node_diff(node, color=CYAN):
    plot_curve(node.x, node.y, node.theta, node.L, node.R, color)


def get_neighbors_rigid_diff(node):
    neighbors = []
    for left in actions:
        for right in actions:
            if right == left == 0:
                continue
            xn, yn, tn, dist = move_curve(node.end_x, node.end_y, node.end_theta, left, right)
            if point_valid(xn, yn, False):
                neighbors.append(Node(node.end_x, node.end_y, node.end_theta, xn, yn, node, dist, left, right, tn))
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
    pygame.draw.circle(board, BLACK, [2 * SCALAR, (HEIGHT - 2 * GRAIN) * SCALE], 1 * SCALAR)
    pygame.draw.circle(board, BLACK, [2 * SCALAR, (HEIGHT - 8 * GRAIN) * SCALE], 1 * SCALAR)
    pygame.draw.rect(board, BLACK, pygame.Rect(
        .25 * SCALAR, (HEIGHT - 5.75 * GRAIN) * SCALE, 1.5 * SCALAR, 1.5 * SCALAR))
    pygame.draw.rect(board, BLACK, pygame.Rect(
        3.75 * SCALAR, (HEIGHT - 5.75 * GRAIN) * SCALE, 2.5 * SCALAR, 1.5 * SCALAR))
    pygame.draw.rect(board, BLACK, pygame.Rect(
        7.25 * SCALAR, (HEIGHT - 4 * GRAIN) * SCALE, 1.5 * SCALAR, 2 * SCALAR))


def in_circle(x, y):  # check if point in lower circle
    if math.pow(x - 2 * GRAIN, 2) + math.pow(y - (HEIGHT - 2 * GRAIN), 2) >= math.pow(1 * GRAIN + CLEARANCE, 2):
        return False
    return True


def in_circle_2(x, y):  # check if point in upper circle
    if math.pow(x - 2 * GRAIN, 2) + math.pow(y - (HEIGHT - 8 * GRAIN) , 2) >= math.pow(1 * GRAIN + CLEARANCE, 2):
        return False
    return True


def in_rect(x, y):    # check if point in rectangle
    if .25 * GRAIN - CLEARANCE <= x <= 1.75 * GRAIN + CLEARANCE and \
            5.75 * GRAIN + CLEARANCE >= y >= 4.25 * GRAIN - CLEARANCE:
        return True
    return False


def in_rect_2(x, y):
    if 3.75 * GRAIN - CLEARANCE <= x <= 6.25 * GRAIN + CLEARANCE and \
            5.75 * GRAIN + CLEARANCE >= y >= 4.25 * GRAIN - CLEARANCE:
        return True
    return False


def in_rect_3(x, y):
    if 7.25 * GRAIN - CLEARANCE <= x <= 8.75 * GRAIN + CLEARANCE and \
            4 * GRAIN + CLEARANCE >= y >= 2 * GRAIN - CLEARANCE:
        return True
    return False


# check if point is in any obstacle
def in_obstacle(x, y):
    if in_circle(x, y) or in_circle_2(x, y) or in_rect(x, y) or in_rect_2(x, y) or in_rect_3(x, y):
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
        draw_node_diff(point, MAGENTA)
        pygame.display.update()
        pygame.event.get()
        clock.tick(12)
    pygame.display.update()
    if path:
        print("Path length: ", path[-2].path_length)


def move_curve(x_i, y_i, theta_i, ul, ur):
    t = 0
    x_n = x_i
    y_n = y_i
    theta_n = 3.14 * theta_i / 180
    dist = 0
    while t < stop_time:
        t = t + dt
        x_s = x_n
        y_s = y_n
        x_n += (0.5 * r * (ul + ur) * math.cos(theta_n) * dt)
        y_n += (0.5 * r * (ul + ur) * math.sin(theta_n) * dt)
        theta_n += (r / L) * (ur - ul) * dt
        dist += distance(x_s, y_s, x_n, y_n)
    theta_n = 180 * theta_n / 3.14
    return x_n, y_n, theta_n, dist


def plot_curve(x_i, y_i, theta_i, ul, ur, color = CYAN):
    t = 0
    x_n = x_i
    y_n = y_i
    theta_n = 3.14 * theta_i / 180
    dist = 0
    while t < stop_time:
        t = t + dt
        x_s = x_n
        y_s = y_n
        x_n += (0.5 * r * (ul + ur) * math.cos(theta_n) * dt)
        y_n += (0.5 * r * (ul + ur) * math.sin(theta_n) * dt)
        theta_n += (r / L) * (ur - ul) * dt
        dist += distance(x_s, y_s, x_n, y_n)
        pygame.draw.line(board, color, [x_s * SCALE, (HEIGHT - y_s) * SCALE], [x_n * SCALE, (HEIGHT - y_n) * SCALE])
    return


def get_diff():
    print("This system works best with values in the range of 30-60")
    print("The values should not differ more than 5, and work best with a difference of 2 or 3")
    actions[0] = int(input("Enter the smaller value: "))
    actions[1] = int(input("Enter the larger value: "))


def sanity_check():
    for i in range(100000):
        x = randint(0, WIDTH)
        y = randint(0, HEIGHT)
        if point_valid(x, y):
            pygame.draw.circle(board, RED, [x * SCALE, (HEIGHT - y) * SCALE], 1 * SCALE)
        else:
            pygame.draw.circle(board, GREEN, [x * SCALE, (HEIGHT - y) * SCALE], 1 * SCALE)
        pygame.display.update()
        pygame.event.get()


if __name__ == "__main__":
    mode = 1
    start, target = get_initial_conditions(True)
    print("Finding path...")
    real_time = True

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
