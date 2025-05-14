#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
import math
import rosbag
import networkx as nx
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from rospy_tutorials.msg import Floats
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu 
from cart_sim.msg import AUTONOMOUS_SteeringMot_Control
from cart_sim.msg import AUTONOMOUS_BrakePedalControl
from cart_sim.msg import AUTONOMOUS_HB_MotorControl

steer_pwm_pub = 0
steer_pwm = 0
class TurtlebotPID:
    def __init__(self):
        rospy.init_node('bee1')
        rospy.Subscriber('/XEst', Floats, self.xest_callback)
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.Subscriber('/planned_path', Path, self.path_callback)

        self.steer_pub= rospy.Publisher('/beemobs/steering_target_value',Float64, queue_size=50)
       # self.steer_pub = rospy.Publisher('/beemobs/AUTONOMOUS_SteeringMot_Control', AUTONOMOUS_SteeringMot_Control, queue_size=50)
        self.speed_pub = rospy.Publisher('/beemobs/speed_target_value', Float64, queue_size=10)
        self.brake_pub = rospy.Publisher('/beemobs/AUTONOMOUS_BrakePedalControl', AUTONOMOUS_BrakePedalControl, queue_size=10)
        self.motor_pub = rospy.Publisher('/beemobs/AUTONOMOUS_HB_MotorControl', AUTONOMOUS_HB_MotorControl, queue_size=10)
        self.rate = rospy.Rate(50)
        self.target_position = Point()
        self.current_position = Point()
        self.current_yaw = 0
        self.kp = 0.5
        self.ki = 0.01
        self.kd = 0.1
        self.prev_error = 0
        self.integral = 0
        self.integral_max = 10
        self.integral_min = -10
        self.stop_flag = False
        self.path_points = []
        self.stop_flag = False

    def xest_callback(self, msg):
        self.current_position.x = msg.data[0]
        self.current_position.y = msg.data[1]

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)

    def path_callback(self, msg):
        if not msg.poses:
            rospy.logwarn("Boş yol verisi alındı")
            return

        self.path_points = [pose.pose.position for pose in msg.poses]
        rospy.loginfo(f"{len(self.path_points)} noktadan oluşan yol verisi alındı")
        self.new_path_received = True

    def calculate_distance(self):
        distance = math.sqrt((self.target_position.x - self.current_position.x) ** 2 +(self.target_position.y - self.current_position.y) ** 2)
        return distance

    def calculate_heading(self):
        delta_x = self.target_position.x - self.current_position.x
        delta_y = self.target_position.y - self.current_position.y     
        target_yaw = math.atan2(delta_y, delta_x)
        heading = (target_yaw - self.current_yaw)*180/math.pi
        print(self.current_yaw,target_yaw,heading)
        
        return heading

    def pid_control(self):
        global steer_pwm_pub, steer_pwm
        if self.stop_flag:
            return

        distance = self.calculate_distance()
        heading = self.calculate_heading()
        

        error = distance
        self.integral += error
        self.integral = np.clip(self.integral, self.integral_min, self.integral_max)
        derivative = error - self.prev_error

        control_output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        


        if heading>=0 and heading<90:
            steer_pwm=int(heading/30*127)
            steer_pwm_pub=np.clip(steer_pwm,0,127)  
            
        elif heading>90 and heading<180:
            steer_pwm=-(int((heading-90)/30*127))
            steer_pwm_pub=np.clip(steer_pwm,-127,0)
            
        elif heading>-90 and heading<0:
            steer_pwm=-int(heading/30*127)
            steer_pwm_pub=np.clip(steer_pwm,0,127)
            
        elif heading>-180 and heading<-90:
            steer_pwm=int((heading+90)/30*127)
            steer_pwm_pub=np.clip(steer_pwm,-127,0)
        print(steer_pwm_pub)

        breakmsg=AUTONOMOUS_BrakePedalControl()
        breakmsg.AUTONOMOUS_BrakePedalMotor_EN=1
        steer_msg = AUTONOMOUS_SteeringMot_Control()
        steer_msg.AUTONOMOUS_SteeringMot_EN = 1
        breakmsg.AUTONOMOUS_BrakeMotor_Voltage=1

        #command.header.stamp = rospy.Time.now()

        if distance > 2:    
            steer_val= steer_pwm_pub
            target_speed = Float64(np.clip(control_output, 0, 2))
        else:  
            steer_val = steer_pwm_pub
            target_speed = Float64(np.clip(control_output, 0, 2))
            breakmsg.AUTONOMOUS_BrakePedalMotor_PER=100
            self.stop_flag = True

        self.speed_pub.publish(target_speed)
        self.steer_pub.publish(steer_val)
        self.brake_pub.publish(breakmsg)

        self.prev_error = error
        self.rate.sleep()

    def execute_path(self):
        while not rospy.is_shutdown():
            if self.new_path_received:
                self.new_path_received = False
                for target_pos in self.path_points:
                    self.target_position = target_pos
                    rospy.loginfo(f"Hedefe gidiliyor: x={self.target_position.x}, y={self.target_position.y}")

                    while not rospy.is_shutdown() and not self.stop_flag:
                        self.pid_control()

                    rospy.loginfo(f"Hedef noktasina ulaşildi: x={self.target_position.x}, y={self.target_position.y}")
                    self.stop_flag = False

                rospy.signal_shutdown("Tüm hedef noktalarina ulaşildi.")

def create_graph(x_values, y_values):
    G = nx.complete_graph(len(x_values))
    pos = {i: (x_values[i], y_values[i]) for i in range(len(x_values))}
    nx.set_node_attributes(G, pos, 'pos')
    return G

if __name__ == '__main__':
    # bag_file_path = '/home/talos-baris/anode.bag'
    x_values = [4197919.5,4197932.0]
    y_values = [2375933.5,2375926.5]

    # with rosbag.Bag(bag_file_path, 'r') as bag:
    #     for topic, msg, t in bag.read_messages(topics=['/node']):
    #         x = msg.x
    #         y = msg.y
    #         x_values.append(x)
    #         y_values.append(y)
    
    required_points = [0,1]

    G = create_graph(x_values, y_values)

    path = []
    for i in range(len(required_points) - 1):
        segment = nx.shortest_path(G, source=required_points[i], target=required_points[i + 1])
        if i > 0:
            segment = segment[1:]
        path.extend(segment)

    try:
        pid_controller = TurtlebotPID()
        pid_controller.execute_path()
        for i in path:
            pid_controller.target_position.x = x_values[i]
            pid_controller.target_position.y = y_values[i]

            while not rospy.is_shutdown() and not pid_controller.stop_flag:
                pid_controller.pid_control()

            pid_controller.stop_flag = False 
            print(f"Reached target {i}")

        rospy.sleep(1)
        rospy.signal_shutdown("All target positions reached")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS düğümü sonlandirildi.")
