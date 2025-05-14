#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion

class PositionPublisher:
    def __init__(self):
        rospy.init_node('position_publisher', anonymous=True)
        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/current_position', Odometry, queue_size=10)
        self.orientation_pub = rospy.Publisher('/current_orientation', Quaternion, queue_size=10)
        rospy.spin()

    def odom_callback(self, msg):
        # Publish the entire Odometry message
        self.pub.publish(msg)

        # Calculate and publish the orientation (yaw)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        rospy.loginfo("Current Position: x = %.10f, y = %.10f", msg.pose.pose.position.x, msg.pose.pose.position.y)
        rospy.loginfo("Current Orientation: yaw = %.10f", yaw)

        # Create a Quaternion message for the orientation
        yaw_quaternion = Quaternion()
        yaw_quaternion.z = orientation_q.z
        yaw_quaternion.w = orientation_q.w

        self.orientation_pub.publish(yaw_quaternion)

if __name__ == '__main__':
    PositionPublisher()
