#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def broadcast_map_transform():
    rospy.init_node('map_frame_setup')

    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "odom"  # Assume your odometry frame is named "odom"
    transform_stamped.child_frame_id = "map"
    transform_stamped.transform.translation.x = 0.0  # Set your map's origin coordinates
    transform_stamped.transform.translation.y = 0.0
    transform_stamped.transform.translation.z = 0.0
    transform_stamped.transform.rotation.w = 1.0  # No rotation in this example

    tf_broadcaster.sendTransform([transform_stamped])

    rospy.spin()

if __name__ == '__main__':
    broadcast_map_transform()
