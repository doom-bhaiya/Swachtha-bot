#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Quaternion, Pose, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('odometry_publisher')

        # Robot pose variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocity commands
        self.vx = 0.0
        self.vtheta = 0.0

        # Odometry update rate
        self.update_rate = rospy.Rate(10)  # 10 Hz

        # Subscribers and Publisher
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

        self.run()

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vtheta = msg.angular.z

    def run(self):
        while not rospy.is_shutdown():
            # Update robot pose based on velocity commands
            dt = self.update_rate.sleep_dur.to_sec()

            self.x += self.vx * cos(self.theta) * dt
            self.y += self.vx * sin(self.theta) * dt
            self.theta += self.vtheta * dt

            # Create Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            # Set pose information
            odom_msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
            quat = self.get_quaternion(self.theta)
            odom_msg.pose.pose.orientation = quat
            odom_msg.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                        0, 0.1, 0, 0, 0, 0,
                                        0, 0, 0.1, 0, 0, 0,
                                        0, 0, 0, 0.1, 0, 0,
                                        0, 0, 0, 0, 0.1, 0,
                                        0, 0, 0, 0, 0, 0.1]

            odom_msg.twist.covariance = [0.1, 0, 0, 0, 0, 0,
                                         0, 0.1, 0, 0, 0, 0,
                                         0, 0, 0.1, 0, 0, 0,
                                         0, 0, 0, 0.1, 0, 0,
                                         0, 0, 0, 0, 0.1, 0,
                                         0, 0, 0, 0, 0, 0.1]
            # Set velocity information
            odom_msg.twist.twist.linear.x = self.vx
            odom_msg.twist.twist.angular.z = self.vtheta

            # Publish Odometry message
            self.odom_pub.publish(odom_msg)

            self.update_rate.sleep()

    def get_quaternion(self, theta):
        return Quaternion(*quaternion_from_euler(0, 0, theta))

if __name__ == '__main__':
    try:
        OdometryPublisher()
    except rospy.ROSInterruptException:
        pass