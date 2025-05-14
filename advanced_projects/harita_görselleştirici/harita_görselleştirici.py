#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

class MapVisualizer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('map_visualizer', anonymous=True)

        # Subscriptions
        rospy.Subscriber('/current_position', Odometry, self.position_callback)
        rospy.Subscriber('/planned_path', Path, self.path_callback)
        rospy.Subscriber('/goal_position', Point, self.goal_callback)
        rospy.Subscriber('/obstacles', PointCloud, self.obstacle_callback)
        rospy.Subscriber('/gecici_engeller', Point, self.temp_obstacle_callback)  # New subscription for temporary obstacles

        # Initial values
        self.current_position = None
        self.path_points = []
        self.goal_position = None
        self.obstacles = []
        self.temp_obstacles = []  # List to store temporary obstacles

        # Matplotlib plot settings
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-50, 70)
        self.ax.set_ylim(-50, 70)
        self.robot_plot, = self.ax.plot([], [], 'bo', markersize=5, label='Robot Pozisyonu')
        self.path_plot, = self.ax.plot([], [], 'r-', label='Planlanan Yol')
        self.goal_plot, = self.ax.plot([], [], 'ro', markersize=7, label='Hedef Pozisyon')
        self.obstacle_plot, = self.ax.plot([], [], 'ko', markersize=3, label='Engeller')
        self.temp_obstacle_plot, = self.ax.plot([], [], 'mo', markersize=3, label='Geçici Engeller')  # Plot for temporary obstacles
        self.ax.legend()

        # Use FuncAnimation to update plots
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)

    def position_callback(self, msg: Odometry):
        self.current_position = msg.pose.pose.position

    def path_callback(self, msg: Path):
        self.path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def goal_callback(self, msg: Point):
        self.goal_position = msg

    def obstacle_callback(self, msg: PointCloud):
        self.obstacles = [(point.x, point.y) for point in msg.points]

    def temp_obstacle_callback(self, msg: Point):
        # Update temporary obstacles
        self.temp_obstacles.append((msg.x, msg.y))

    def update_plot(self, frame):
        # Update robot position
        if self.current_position:
            self.robot_plot.set_data(self.current_position.x, self.current_position.y)

        # Update planned path
        if self.path_points:
            path_x, path_y = zip(*self.path_points)
            self.path_plot.set_data(path_x, path_y)

        # Update goal position
        if self.goal_position:
            self.goal_plot.set_data(self.goal_position.x, self.goal_position.y)

        # Update obstacles
        if self.obstacles:
            obs_x, obs_y = zip(*self.obstacles)
            self.obstacle_plot.set_data(obs_x, obs_y)

        # Update temporary obstacles
        if self.temp_obstacles:
            temp_obs_x, temp_obs_y = zip(*self.temp_obstacles)
            self.temp_obstacle_plot.set_data(temp_obs_x, temp_obs_y)

        # Refresh plot limits and view
        self.ax.relim()
        self.ax.autoscale_view()

    def run(self):
        plt.show(block=True)
        rospy.spin()

if __name__ == '__main__':
    visualizer = MapVisualizer()
    try:
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
