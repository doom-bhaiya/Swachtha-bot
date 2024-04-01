#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped

class VelocityControllerTFPublisher:
    def __init__(self, controller_name, frame_id, link_name):
        self.controller_name = controller_name
        self.frame_id = frame_id
        self.link_name = link_name

        self.subscriber = rospy.Subscriber(
            "{}/command".format(self.controller_name),
            Float64,
            self.velocity_callback
        )

        self.broadcaster = tf.TransformBroadcaster()

    def velocity_callback(self, msg):
        linear_velocity = msg.data

        self.broadcaster.sendTransform(
            (linear_velocity, 0.0, 0.0),  # translation (x, y, z)
            (0.0, 0.0, 0.0, 1.0),         # rotation (quaternion)
            rospy.Time.now(),
            self.link_name,
            self.frame_id
        )

class StepperControllerTFPublisher:
    def __init__(self):
        self.subscriber = rospy.Subscriber(
            '/stepper_motor_controller/command',
            Float64,
            self.stepper_callback
        )

        self.broadcaster = tf.TransformBroadcaster()

    def stepper_callback(self, msg):
        position = msg.data

        self.broadcaster.sendTransform(
            (position, 0.0, 0.0),        # translation (x, y, z)
            (0.0, 0.0, 0.0, 1.0),         # rotation (quaternion)
            rospy.Time.now(),
            'stepper_link',
            'base_link'
        )

# class ServoControllerTFPublisher:
#     def __init__(self, controller_name, frame_id):
#         self.controller_name = controller_name
#         self.frame_id = frame_id

#         self.subscriber = rospy.Subscriber(
#             f'/{controller_name}/angle_command',
#             Float64,
#             self.servo_callback
#         )

#         self.broadcaster = tf.TransformBroadcaster()

#     def servo_callback(self, msg):
#         angle = msg.data

#         self.broadcaster.sendTransform(
#             (0.0, 0.0, 0.0),              # translation (x, y, z)
#             tf.transformations.quaternion_from_euler(0.0, 0.0, angle),  # rotation (quaternion)
#             rospy.Time.now(),
#             f'{self.controller_name}_link',
#             self.frame_id
#         )

if __name__ == '__main__':
    rospy.init_node('tf_publisher_node')

    # Velocity controllers
    vel_controller1 = VelocityControllerTFPublisher('velocity_controller1', 'base_link', "wheel1")
    vel_controller2 = VelocityControllerTFPublisher('velocity_controller2', 'base_link', "wheel2")
    vel_controller3 = VelocityControllerTFPublisher('velocity_controller3', 'base_link', "wheel3")
    vel_controller4 = VelocityControllerTFPublisher('velocity_controller4', 'base_link', "wheel4")

    # # Stepper motor controller
    # stepper_controller = StepperControllerTFPublisher()

    # # Servo motor controller
    # servo_controller = ServoControllerTFPublisher('servo_controller', 'base_link')

    rospy.spin()
