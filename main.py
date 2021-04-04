#Dakota Abernathy
#ENPM692-Porj2

import math
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

nodes_visited = []
path = []
SQRT2 = math.sqrt(2)
nodes = None

# distance between two points
def distance(x1,y1,x2,y2):
    return math.sqrt(pow((x2-x1), 2)+pow((y2-y1), 2))


# class to keep track of each place visited
class Node:
    def __init__(self, x, y, parent, dist=0):
        self.x = x
        self.y = y
        self.parent = parent
        self.h = 0
        if parent:
            self.path_length = parent.path_length + dist
            self.g = parent.g + 1
        else:
            self.path_length = 0
            self.g = 0

    def heuristic(self):  # a* heuristic
        return distance(self.x, self.y, target.x, target.y) + self.g

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return "["+str(self.x)+", "+str(self.y) + "]"

    def __lt__(self, other):
        return self.path_length < other.path_length

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
    pygame.draw.polygon(board, BLACK,  # why is this so ugly
                        [(354 * SCALE, (HEIGHT - 138) * SCALE), (380 * SCALE, (HEIGHT - 170) * SCALE),
                         (380 * SCALE, (HEIGHT - 115) * SCALE), (328 * SCALE, (HEIGHT - 63) * SCALE),
                         (286 * SCALE, (HEIGHT - 105) * SCALE), (325 * SCALE, (HEIGHT - 143) * SCALE)])


# check if point in circle
def in_circle(x, y):
    if math.pow(x - 90, 2) + math.pow(y - 70, 2) >= 1225:
        return False
    return True


# check if point in ellipse
def in_ellipse(x, y):
    center_x = 246
    center_y = 146
    horizontal_axis = 60
    vertical_axis = 30
    if ((math.pow(x - center_x, 2) / math.pow(horizontal_axis, 2)) +
        (math.pow(y - center_y, 2) / math.pow(vertical_axis, 2))) <= 1:
        return True
    return False


# check if point in C-shape
def in_c_shape(x, y):
    if (x >= 200 and x <= 210 and y <= 280 and y >= 230) or \
        (x >= 210 and x <= 230 and y >= 270 and y <= 280) or \
        (y >= 230 and y <= 240 and x >= 210 and x <= 230):
        return True
    return False


# check if point in weird polygon
def in_poly(x, y):
    if ((y - 1 * x + 181.6) < 0 and (y + 0.3 * x - 239.9) < 0 and (y + 249.2 * x - 95054.25) <
        0 and (y - x + 265) > 0 and (y + 1 * x - 389.3) > 0) or \
        ((y - 1.13 * x + 260.75) < 0 and (y + 249.2 * x - 95054.25) < 0 and (y + .3 * x - 240.6) > 0):
        return True
    return False


# check if point in rotated rectangle
def in_line_segment(x, y):
    if (y + 1.4 * x - 176.5) > 0 and (y - 0.7 * x - 74.4) > 0 \
        and (y - 0.7 * x - 98.8) < 0 and (y + 1.4 * x - 438.1) < 0:
        return True
    return False


# # check if point is in any obstacle
def in_obstacle(x, y):
    if in_circle(x, y) or in_ellipse(x, y) or in_c_shape(x, y) or \
            in_line_segment(x, y) or in_poly(x, y):
        return True
    return False


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
    for i in range(100000):
        x = randint(0, 400)
        y = randint(0, 300)
        if point_valid(x, y):
            pygame.draw.circle(board, RED, [x * SCALE, (HEIGHT - y) * SCALE], 1 * SCALE)
        else:
            pygame.draw.circle(board, GREEN, [x * SCALE, (HEIGHT - y) * SCALE], 1 * SCALE)
        pygame.display.update()


# gets single valid point from user
def get_point_from_user(word):
    valid = False
    while not valid:
        x = int(input("Enter the X coordinate of the "+word+" point: "))
        y = int(input("Enter the Y coordinate of the " + word + " point: "))
        valid = point_valid(x, y, True)
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
    return Node(x1, y1, None), Node(x2, y2, None)


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
            if itt % 50 == 0:
                pygame.display.update()
                pygame.event.get()
            pygame.draw.rect(board, CYAN, [next_node.x * SCALE, (HEIGHT - next_node.y) * SCALE, 2 * SCALE, 2 * SCALE])

        if next_node == target:  # check if done
            print("Found path")
            target.parent = next_node.parent
            return

        new_nodes = get_neighbors(next_node)  # get neighbors
        for new_node in new_nodes:
            # add them to list to check if they are new
            if str(new_node) not in explored and str(new_node) not in to_explore_check:
                to_explore.put(new_node)
                to_explore_check[str(new_node)] = True
    print("No path")


# a* search
def a_star():
    open = [(start.heuristic(), start)]
    open_check = {str(start): 1}
    closed = {}
    itt = 0
    while len(open) > 0:
        open.sort(reverse=True)  # get next node and mark dict
        tmp = open.pop()
        next_node = tmp[1]
        open_check[str(next_node)] = open_check[str(next_node)] - 1
        if open_check[str(next_node)] == 0:
            open_check.pop(str(next_node))

        if real_time:  # print if in real time
            itt = itt + 1
            if itt % 50 == 0:
                pygame.display.update()
                pygame.event.get()
            pygame.draw.rect(board, CYAN, [next_node.x * SCALE, (HEIGHT - next_node.y) * SCALE, 2 * SCALE, 2 * SCALE])

        closed[str(next_node)] = True
        if next_node == target:  # check if found
            target.parent = next_node.parent
            return
        nodes_visited.append(next_node)
        neighbors = get_neighbors(next_node)
        for neighbor in neighbors:  # get neighbors and check if they have been checked yet
            if str(neighbor) in closed:
                continue
            if add_to_open(open, open_check, neighbor):  # see if we need to add it
                open.append((neighbor.heuristic(), neighbor))
                if str(neighbor) in open_check:  # update dict
                    open_check[str(neighbor)] = open_check[str(neighbor)] + 1
                else:
                    open_check[str(neighbor)] = 1

    return None


# check if new node needs to be added to open list
def add_to_open(open, open_check, neighbor):
    if str(neighbor) in open_check:
        for node in open:
            if neighbor == node[1] and neighbor.g < node[1].g:
                return False
    return True


# work back from target to get path to start
def back_track():
    n = target
    while n:
        path.append(n)
        n = n.parent
    path.reverse()


# adds all visited nodes, the path, start and end points to board
def add_points():
    pygame.draw.circle(board, GREEN, [start.x * SCALE, (HEIGHT - start.y) * SCALE], 4 * SCALE)
    pygame.draw.circle(board, MAGENTA, [target.x * SCALE, (HEIGHT - target.y) * SCALE], 4 * SCALE)
    pygame.display.update()
    print("Visited: ", len(nodes_visited))
    clock = pygame.time.Clock()
    itt = 0
    for point in nodes_visited:
        pygame.draw.rect(board, CYAN, [point.x * SCALE, (HEIGHT - point.y) * SCALE, 2 * SCALE, 2 * SCALE])
        if itt % 10 == 0:
            pygame.display.update()
            pygame.event.get()
        itt = itt + 1
        clock.tick(4000)
    pygame.display.update()
    pygame.draw.circle(board, GREEN, [start.x * SCALE, (HEIGHT - start.y) * SCALE], 4 * SCALE)
    pygame.draw.circle(board, MAGENTA, [target.x * SCALE, (HEIGHT - target.y) * SCALE], 4 * SCALE)
    print("Path: ", len(path))
    for point in path:
        pygame.draw.rect(board, RED, [point.x * SCALE, (HEIGHT - point.y) * SCALE, 2 * SCALE, 2 * SCALE])
        if itt % 10 == 0:
            pygame.display.update()
            pygame.event.get()
        itt = itt + 1
        clock.tick(100)
    pygame.display.update()
    if path:
        print("Path length: ", path[1].path_length)


if __name__ == "__main__":
    # mode = int(input("Choose 1 for a* or 2 for breath first search: "))
    mode = 2
    start, target = get_initial_conditions(True)
    print("Finding path...")
    real_time = False

    if real_time:
        make_board()
        add_points()

    if mode == 1:
        a_star()
    else:
        BFS()

    make_board()
    back_track()
    add_points()
    pygame.display.update()
    print("Done")
    for i in range(51):
        time.sleep(1)
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                raise SystemExit
