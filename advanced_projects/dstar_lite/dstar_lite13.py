#!/usr/bin/python3

import math
import matplotlib.pyplot as plt
import random
import numpy as np
import heapq
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rosbag
import time

show_animation = True
pause_time = 0.1
p_create_random_obstacle = 0   
safety_distance = 0             

class Node:
    def __init__(self, x: int = 0, y: int = 0, cost: float = 0.0):
        self.x = x
        self.y = y
        self.cost = cost

    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)

def add_coordinates(node1: Node, node2: Node):
    new_node = Node()
    new_node.x = node1.x + node2.x
    new_node.y = node1.y + node2.y
    new_node.cost = node1.cost + node2.cost
    return new_node

def compare_coordinates(node1: Node, node2: Node):
    return node1.x == node2.x and node1.y == node2.y

class DStarLite:
    motions = [
        Node(1, 0, 1),
        Node(0, 1, 1),
        Node(-1, 0, 1),
        Node(0, -1, 1),
        Node(1, 1, math.sqrt(2)),
        Node(1, -1, math.sqrt(2)),
        Node(-1, 1, math.sqrt(2)),
        Node(-1, -1, math.sqrt(2))
    ]

    def __init__(self, ox: list, oy: list, min_obstacle_distance=5):
        rospy.init_node('dstar_lite_planner', anonymous=True)
        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=10)
        self.stop_pub = rospy.Publisher("/stop_point", PoseStamped, queue_size=10)
        self.park_pub = rospy.Publisher("/park_spot", PoseStamped, queue_size=10)

        self.x_min_world = int(min(ox))
        self.y_min_world = int(min(oy))
        self.x_max = int(abs(max(ox) - self.x_min_world))
        self.y_max = int(abs(max(oy) - self.y_min_world))
        self.obstacles = [Node(x - self.x_min_world, y - self.y_min_world) for x, y in zip(ox, oy)]
        self.obstacles_xy = np.array([[obstacle.x, obstacle.y] for obstacle in self.obstacles])
        self.buffered_obstacles_xy = self.create_buffered_obstacles()
        self.start = Node(0, 0)
        self.goal = Node(0, 0)
        self.U = []
        self.km = 0.0
        self.kold = 0.0
        self.rhs = self.create_grid(float("inf"))
        self.g = self.create_grid(float("inf"))
        self.detected_obstacles_xy = np.empty((0, 2))
        self.xy = np.empty((0, 2))
        self.min_obstacle_distance = min_obstacle_distance

        if show_animation:
            self.detected_obstacles_for_plotting_x = []
            self.detected_obstacles_for_plotting_y = []
        self.initialized = False

    def create_grid(self, val: float):
        return np.full((self.x_max, self.y_max), val)

    def create_buffered_obstacles(self):
        buffered_obstacles = set()
        for obstacle in self.obstacles:
            buffered_obstacles.add((obstacle.x, obstacle.y))
            for dx in range(-safety_distance, safety_distance + 1):
                for dy in range(-safety_distance, safety_distance + 1):
                    if dx != 0 or dy != 0:
                        buffered_obstacles.add((obstacle.x + dx, obstacle.y + dy))
        buffered_obstacles = np.array(list(buffered_obstacles))
        return buffered_obstacles

    def is_obstacle(self, node: Node):
        x = np.array([node.x])
        y = np.array([node.y])

        obstacle_x_equal = self.buffered_obstacles_xy[:, 0] == x
        obstacle_y_equal = self.buffered_obstacles_xy[:, 1] == y
        is_in_obstacles = (obstacle_x_equal & obstacle_y_equal).any()

        is_in_detected_obstacles = False
        if self.detected_obstacles_xy.shape[0] > 0:
            is_x_equal = self.detected_obstacles_xy[:, 0] == x
            is_y_equal = self.detected_obstacles_xy[:, 1] == y
            is_in_detected_obstacles = (is_x_equal & is_y_equal).any()

        return is_in_obstacles or is_in_detected_obstacles

    def c(self, node1: Node, node2: Node):
        if self.is_obstacle(node2):
            return math.inf
        new_node = Node(node1.x - node2.x, node1.y - node2.y)
        detected_motion = list(filter(lambda motion: compare_coordinates(motion, new_node), self.motions))
        return detected_motion[0].cost if detected_motion else math.inf

    def h(self, s: Node):
        return math.sqrt((self.goal.x - s.x)**2 + (self.goal.y - s.y)**2)

    def calculate_key(self, s: Node):
        return (min(self.g[s.x][s.y], self.rhs[s.x][s.y]) + self.h(s) + self.km, min(self.g[s.x][s.y], self.rhs[s.x][s.y]))

    def is_valid(self, node: Node):
        return 0 <= node.x < self.x_max and 0 <= node.y < self.y_max

    def get_neighbours(self, u: Node):
        return [add_coordinates(u, motion) for motion in self.motions if self.is_valid(add_coordinates(u, motion))]

    def pred(self, u: Node):
        return self.get_neighbours(u)

    def succ(self, u: Node):
        return self.get_neighbours(u)

    def initialize(self, start: Node, goal: Node):
        self.start.x = start.x - self.x_min_world
        self.start.y = start.y - self.y_min_world
        self.goal.x = goal.x - self.x_min_world
        self.goal.y = goal.y - self.y_min_world
        self.initialized = True
        self.U = []
        self.km = 0.0
        self.rhs = self.create_grid(math.inf)
        self.g = self.create_grid(math.inf)
        self.rhs[self.goal.x][self.goal.y] = 0
        heapq.heappush(self.U, (self.calculate_key(self.goal), self.goal))
        self.detected_obstacles_xy = np.empty((0, 2))

    def update_vertex(self, u: Node):
        if not compare_coordinates(u, self.goal):
            self.rhs[u.x][u.y] = min([self.c(u, sprime) + self.g[sprime.x][sprime.y] for sprime in self.succ(u)])
        if any([compare_coordinates(u, node) for key, node in self.U]):
            self.U = [(key, node) for key, node in self.U if not compare_coordinates(node, u)]
            heapq.heapify(self.U)
        if self.g[u.x][u.y] != self.rhs[u.x][u.y]:
            heapq.heappush(self.U, (self.calculate_key(u), u))

    def compare_keys(self, key_pair1: tuple, key_pair2: tuple):
        return key_pair1[0] < key_pair2[0] or (key_pair1[0] == key_pair2[0] and key_pair1[1] < key_pair2[1])

    def compute_shortest_path(self):
        while self.U and (self.compare_keys(self.U[0][0], self.calculate_key(self.start)) or self.rhs[self.start.x][self.start.y] != self.g[self.start.x][self.start.y]):
            kold, u = heapq.heappop(self.U)
            if self.compare_keys(kold, self.calculate_key(u)):
                heapq.heappush(self.U, (self.calculate_key(u), u))
            elif self.g[u.x, u.y] > self.rhs[u.x, u.y]:
                self.g[u.x, u.y] = self.rhs[u.x, u.y]
                for s in self.pred(u):
                    self.update_vertex(s)
            else:
                self.g[u.x, u.y] = math.inf
                for s in self.pred(u) + [u]:
                    self.update_vertex(s)

    def detect_changes(self):
        changed_vertices = []
        if len(self.spoofed_obstacles) > 0:
            for spoofed_obstacle in self.spoofed_obstacles[0]:
                if compare_coordinates(spoofed_obstacle, self.start) or compare_coordinates(spoofed_obstacle, self.goal):
                    continue
                changed_vertices.append(spoofed_obstacle)
                self.detected_obstacles_xy = np.concatenate((self.detected_obstacles_xy, [[spoofed_obstacle.x, spoofed_obstacle.y]]))
                if show_animation:
                    self.detected_obstacles_for_plotting_x.append(spoofed_obstacle.x + self.x_min_world)
                    self.detected_obstacles_for_plotting_y.append(spoofed_obstacle.y + self.y_min_world)
                    plt.plot(self.detected_obstacles_for_plotting_x, self.detected_obstacles_for_plotting_y, ".k")
                    plt.pause(pause_time)
            self.spoofed_obstacles.pop(0)

        random.seed()
        if random.random() > 1 - p_create_random_obstacle:
            x = random.randint(0, self.x_max - 1)
            y = random.randint(0, self.y_max - 1)
            new_obs = Node(x, y)
            if compare_coordinates(new_obs, self.start) or compare_coordinates(new_obs, self.goal):
                return changed_vertices
            if not self.is_too_close(new_obs):
                changed_vertices.append(Node(x, y))
                self.detected_obstacles_xy = np.concatenate((self.detected_obstacles_xy, [[x, y]]))
                if show_animation:
                    self.detected_obstacles_for_plotting_x.append(x + self.x_min_world)
                    self.detected_obstacles_for_plotting_y.append(y + self.y_min_world)
                    plt.plot(self.detected_obstacles_for_plotting_x, self.detected_obstacles_for_plotting_y, ".k")
                    plt.pause(pause_time)
        return changed_vertices

    def is_too_close(self, new_obs: Node):
        for ox, oy in self.obstacles_xy:
            if math.hypot(new_obs.x - ox, new_obs.y - oy) < self.min_obstacle_distance:
                return True
        for ox, oy in self.detected_obstacles_xy:
            if math.hypot(new_obs.x - ox, new_obs.y - oy) < self.min_obstacle_distance:
                return True
        return False

    def compute_current_path(self):
        path = []
        current_point = Node(self.start.x, self.start.y)
        while not compare_coordinates(current_point, self.goal):
            path.append(current_point)
            current_point = min(self.succ(current_point), key=lambda sprime: self.c(current_point, sprime) + self.g[sprime.x][sprime.y])
        path.append(self.goal)
        return path

    def compare_paths(self, path1: list, path2: list):
        if len(path1) != len(path2):
            return False
        for node1, node2 in zip(path1, path2):
            if not compare_coordinates(node1, node2):
                return False
        return True

    def display_path(self, path: list, colour: str, alpha: float = 1.0):
        px = [(node.x + self.x_min_world) for node in path]
        py = [(node.y + self.y_min_world) for node in path]
        drawing = plt.plot(px, py, colour, alpha=alpha)
        plt.pause(pause_time)
        return drawing

    def publish_path(self, path: list):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for node in path:
            pose = PoseStamped()
            pose.pose.position.x = node.x + self.x_min_world
            pose.pose.position.y = node.y + self.y_min_world
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        path_points = [(node.x + self.x_min_world, node.y + self.y_min_world) for node in path]
        rospy.loginfo(f"Planned path points: {path_points}")

        self.path_pub.publish(path_msg)

    def main(self, waypoints: list, stops: list, spoofed_ox: list, spoofed_oy: list):
        self.spoofed_obstacles = [[Node(x - self.x_min_world, y - self.y_min_world) for x, y in zip(rowx, rowy)] for rowx, rowy in zip(spoofed_ox, spoofed_oy)]
        pathx = []
        pathy = []
        visited_stops = set()  # Keep track of visited stops

        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            goal = waypoints[i + 1]
            self.initialize(start, goal)
            last = self.start
            self.compute_shortest_path()
            pathx.append(self.start.x + self.x_min_world)
            pathy.append(self.start.y + self.y_min_world)

            if show_animation:
                current_path = self.compute_current_path()
                previous_path = current_path.copy()
                previous_path_image = self.display_path(previous_path, ".c", alpha=0.3)
                current_path_image = self.display_path(current_path, ".c")

            self.publish_path(self.compute_current_path())

            while not compare_coordinates(self.goal, self.start):
                if self.g[self.start.x][self.start.y] == math.inf:
                    print("Path not found")
                    return False, pathx, pathy
                self.start = min(self.succ(self.start), key=lambda sprime: self.c(self.start, sprime) + self.g[sprime.x][sprime.y])
                pathx.append(self.start.x + self.x_min_world)
                pathy.append(self.start.y + self.y_min_world)
                if show_animation:
                    current_path.pop(0)
                    plt.plot(pathx, pathy, "-r")
                    plt.pause(pause_time)
                changed_vertices = self.detect_changes()
                if len(changed_vertices) != 0:
                    print("New obstacle detected")
                    self.km += self.h(last)
                    last = self.start
                    for u in changed_vertices:
                        if compare_coordinates(u, self.start):
                            continue
                        self.rhs[u.x][u.y] = math.inf
                        self.g[u.x][u.y] = math.inf
                        self.update_vertex(u)
                    self.compute_shortest_path()

                    if show_animation:
                        new_path = self.compute_current_path()
                        if not self.compare_paths(current_path, new_path):
                            current_path_image[0].remove()
                            previous_path_image[0].remove()
                            previous_path = current_path.copy()
                            current_path = new_path.copy()
                            previous_path_image = self.display_path(previous_path, ".c", alpha=0.3)
                            current_path_image = self.display_path(current_path, ".c")
                            plt.pause(pause_time)
                    self.publish_path(self.compute_current_path())

                # Check proximity to stops
                for stop in stops:
                    if (stop.x, stop.y) not in visited_stops and math.hypot(self.start.x + self.x_min_world - stop.x, self.start.y + self.y_min_world - stop.y) < self.min_obstacle_distance:
                        perform_stop_action(stop, self.stop_pub)
                        visited_stops.add((stop.x, stop.y))
                        # Print message when leaving the stop
                        print(f"Leaving stop: ({stop.x}, {stop.y})")

        print("Path found")
        return True, pathx, pathy

def draw_parking_spot(node: Node, park_pub):
    plt.plot(node.x, node.y, 'ob', markersize=20)
    plt.pause(pause_time)
    
    # Publish the parking spot message
    park_msg = PoseStamped()
    park_msg.header.stamp = rospy.Time.now()
    park_msg.header.frame_id = "map"
    park_msg.pose.position.x = node.x
    park_msg.pose.position.y = node.y
    park_msg.pose.orientation.w = 1.0
    park_pub.publish(park_msg)

def draw_stop_point(node: Node):
    plt.plot(node.x, node.y, 'ob', markersize=10)
    plt.pause(pause_time)

def perform_stop_action(stop, stop_pub):
    print(f"Durak noktasina gelindi: ({stop.x}, {stop.y})") 
    draw_stop_point(stop)  
    
    stop_msg = PoseStamped()
    stop_msg.header.stamp = rospy.Time.now()
    stop_msg.header.frame_id = "map"
    stop_msg.pose.position.x = stop.x
    stop_msg.pose.position.y = stop.y
    stop_msg.pose.orientation.w = 1.0
    stop_pub.publish(stop_msg)
    
    time.sleep(5)  
def main():
    bag_file_path = '/home/burak/talos_sim/src/cart_sim/scripts/anode.bag'
    x = []
    y = []

    with rosbag.Bag(bag_file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/node']):
            x1 = msg.x
            y1 = msg.y
            x.append(int(x1))
            y.append(int(y1))

    waypoints = []

    for i in range(len(x)):
        print(x[i], y[i])
        waypoints.append(Node(x[i], y[i]))

    ox, oy = [], []
    for i in range(-60, 60):
        ox.append(i)
        oy.append(-60.0)
    for i in range(-60, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-60, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-60, 61):
        ox.append(-60.0)
        oy.append(i)

    if show_animation:
        plt.plot(ox, oy, ".k")
        for waypoint in waypoints:
            plt.plot(waypoint.x, waypoint.y, "og")
        plt.grid(True)
        plt.axis("equal")
        label_column = ['Start', 'Waypoint', 'Goal', 'Taken Path', 'Current Calculated Path', 'Previous Calculated Path', 'Obstacles']
        columns = [plt.plot([], [], symbol, color=colour, alpha=alpha)[0] for symbol, colour, alpha in [['o', 'g', 1], ['o', 'y', 1], ['x', 'b', 1], ['-', 'r', 1], ['.', 'c', 1], ['.', 'c', 0.3], ['.', 'k', 1]]]
        plt.legend(columns, label_column, bbox_to_anchor=(1, 1), title="Legend:", fontsize="xx-small")
        plt.plot()
        plt.pause(pause_time)

    stops = [Node(58, 24)] 

    if show_animation:
        for stop in stops:
            plt.plot(stop.x, stop.y, "ob")

    spoofed_ox = [[], [], [], [i for i in range(0, 0)] + [30 for _ in range(0, 0)]]
    spoofed_oy = [[], [], [], [30 for _ in range(0, 0)] + [i for i in range(0, 0)]]

    dstarlite = DStarLite(ox, oy, min_obstacle_distance=5)

    success, pathx, pathy = dstarlite.main(waypoints, stops, spoofed_ox, spoofed_oy)

    if success:
        final_goal = waypoints[-1]
        draw_parking_spot(final_goal, dstarlite.park_pub) 
        print("Park noktasina gelindi") 

if __name__ == "__main__":
    main()
