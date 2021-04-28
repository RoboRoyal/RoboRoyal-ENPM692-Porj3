#!/usr/bin/env python

import time

time.sleep(6) # wait for everything to load

import rospy
rospy.loginfo("Im Alive!")

import math
import tf
from geometry_msgs.msg import Twist, Point
# from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
try:
    from queue import PriorityQueue
except ImportError:
    from Queue import PriorityQueue




rospy.init_node("move_robot")
pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
velocity_msg = Twist()
rate = rospy.Rate(10)
tf_listener = tf.TransformListener()

odom_frame = 'odom'
base_frame = 'base_footprint'
try:
    tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
    base_frame = 'base_footprint'
except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    try:
        tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
        base_frame = 'base_link'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
        rospy.signal_shutdown("tf Exception")


# global vars not used for set-up
goal = (15.824992, 3.831962)
robot_move_speed = .3
robot_turn_speed = .5
goal_threshold = .5
avoidance_threshold = .5
angular_threshold = 0.04
POINT_THRESH = .16

GRAIN = 25
HEIGHT = 10 * GRAIN
WIDTH = 10 * GRAIN
SCALE = 2


BOT_RADIUS = 1.05  # 105mm
OBSTACLE_CLEARANCE = 6
CLEARANCE = BOT_RADIUS + OBSTACLE_CLEARANCE
THRESHOLD = 6
nodes_visited = []
actions = [30, 31, 0]
path = []
SQRT2 = math.sqrt(2)
nodes = None
found_path = True

r = 0.038
L = 0.354
dt = 0.1
stop_time = 6
SCALAR = GRAIN * SCALE
start = None
target = None

# Returns position of bot according to odometry data
def get_odom_data():
    try:
        (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception. (I can only assume that means 'The F@ck is this Exception')")
        return

    return Point(*trans), rotation[2]



# converts degrees to bot radians
def angle_to_dir(ang):
    if ang <= 180:
        return ang * math.pi / 180
    return -1 * (360 - ang) * math.pi / 180

# stop all motion in bot
def stop():
    velocity_msg.linear.x = 0.0
    velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)
    rate.sleep()
    pub.publish(velocity_msg)
    rate.sleep()
    time.sleep(.15)



# ----------------------------------


# distance between two points
def distance(x1, y1, x2, y2):
    return math.sqrt(pow((x2-x1), 2)+pow((y2-y1), 2))


def round_to_n(num, n=3):
    return n * round(num / n)


# class to keep track of each place visited
class Node:
    def __init__(self, x, y, theta, end_x=0, end_y=0, parent=None, dist=None, ul=None, ur=None, end_theta=0):
        self.x = (x)
        self.y = (y)
        self.end_x = (end_x)
        self.end_y = (end_y)
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
    wall_clear = .5 * GRAIN
    if x < wall_clear or x >= WIDTH - wall_clear:
        if talk:
            print("X is outside of boundary [0,", WIDTH, "]")
        return False
    if y < wall_clear or y > HEIGHT - wall_clear:
        if talk:
            print("Y is outside of boundary [0,", HEIGHT, "]")
        return False
    if in_obstacle(x, y):
        if talk:
            print("Point is inside an obstacle")
        return False
    return True

# a* search
def turtle_a_star():
    print("Starting path finding...")
    global found_path
    open_list = PriorityQueue()
    open_list.put((0, start))
    open_check = {str(start): 1}
    closed = {str(start): 1}
    itt = 0
    while open_list.qsize():
        next_node = open_list.get()[1]

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





# determines if the bot should spin left or right to turn towards specified goal
def left_spin(start, end):
    if (start >= 0) == (end >= 0):
        return start < end
    if (start >= 0) and (end <= 0):
        return abs(start - end) > math.pi
    else:
        return abs(start - end) < math.pi
    return True


# returns the difference in angle between the two given orientations
def get_diff(rot1, rot2):
    if (rot1 >= 0) == (rot2 >= 0):
        return abs(rot1) - abs(rot2)
    tmp = abs(rot1) + abs(rot2)
    if tmp > math.pi:
        return 2 * math.pi - tmp
    return tmp


# turns bot in direction of the specifies orientation
def turn(direction):
    blank, rot = get_odom_data()
    # rospy.loginfo("Target dir: "+str(direction))
    while angular_threshold < abs(get_diff(rot, direction)):
        turn_speed = robot_turn_speed
        if abs(get_diff(rot, direction)) < .2:
            turn_speed = robot_turn_speed / 3

        if left_spin(rot, direction):
            velocity_msg.angular.z = turn_speed
        else:
            velocity_msg.angular.z = -turn_speed

        pub.publish(velocity_msg)
        rate.sleep()
        blank, rot = get_odom_data()
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    # stop()




def navigate(target):
    position, rot = get_odom_data()
    x = position.x + 5
    y = position.y + 5
    end_x = target.end_x / GRAIN
    end_y = target.end_y / GRAIN
    print(x, y, end_x, end_y)
    if distance(x, y, end_x, end_y) < POINT_THRESH:
        print("Already there: ", distance(x, y, end_x, end_y))
        return
    direction = math.atan2(end_y - y, end_x - x)
    turn(direction)
    position, rot = get_odom_data()
    x = position.x + 5
    y = position.y + 5
    while distance(x, y, end_x, end_y) > POINT_THRESH:
        velocity_msg.angular.z = float(node.L - node.R) / float((node.L + node.R) * GRAIN + 1)
        # print(velocity_msg.angular.z)
        velocity_msg.linear.x = robot_move_speed
        #velocity_msg.angular.z = 0
        pub.publish(velocity_msg)
        rate.sleep()
        position, rot = get_odom_data()
        x = position.x + 5
        y = position.y + 5
        rospy.loginfo("Distance to next: " + str(distance(x, y, end_x, end_y)))
    stop()

# initiator of all my problems
if __name__ == "__main__":
    global start, target

    time.sleep(2)
    rospy.loginfo("Beep-Boop, here we go again")
    start_time = time.time()
    rospy.loginfo("I've got em in my sights! Distance to goal: " + str(goal))
    rospy.loginfo(get_odom_data())

    #tmp = Node(0, 0, 0, 7.7, 26)
    #navigate(tmp)

    start = Node(1 * GRAIN, 1 * GRAIN, 0, 1 * GRAIN, 1 * GRAIN)
    target = Node(9 * GRAIN, 9 * GRAIN, 0, 9 * GRAIN, 9 * GRAIN)
    turtle_a_star()
    back_track()

    # path is now a list of nodes
    rospy.loginfo("Path has " + str(len(path)) + " nodes")
    num = 0

    for node in path:
        print(node.end_x, node.end_y)

    if len(path) == 1:
        print("No go")
        SystemExit()

    for node in path:
        rospy.loginfo(str(num))
        print(node.end_x, node.end_y)
        num += 1
        # go from one node to the next
        navigate(node)
        pass

    print("Goal reached")
    stop()
    end_time = time.time()
    rospy.loginfo("Time to complete: {:.4} seconds. Not that its a race or anything....".format(end_time-start_time))

