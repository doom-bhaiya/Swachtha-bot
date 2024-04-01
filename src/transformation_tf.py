#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class TFPublisher:
    def __init__(self):
        rospy.init_node('tf_publisher')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribe to the robot's pose from AMCL (Odometry message)
        rospy.Subscriber('amcl_pose', Odometry, self.amcl_pose_callback)

    def amcl_pose_callback(self, odom_msg):
        # Create a TransformStamped message
        transform_stamped = TransformStamped()

        # Set the header
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = "map"
        transform_stamped.child_frame_id = "base_link"

        # Get the pose from the AMCL-published Odometry message
        pose = odom_msg.pose.pose

        # Set the translation
        transform_stamped.transform.translation.x = pose.position.x
        transform_stamped.transform.translation.y = pose.position.y
        transform_stamped.transform.translation.z = pose.position.z

        # Set the rotation (quaternion)
        transform_stamped.transform.rotation = pose.orientation

        # Publish the dynamic transform
        self.tf_broadcaster.sendTransform([transform_stamped])

if __name__ == '__main__':
    try:
        tf_publisher = TFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
