#!/usr/bin/env python
# Lucas Walter
# make a joint exactly what the command wants it to be- this only works
# for position control.

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class CommandToJointState:
    def __init__(self):
        self.joint_name = rospy.get_param("~joint_name")
        self.joint_state = JointState()
        self.joint_state.name.append(self.joint_name)
        self.joint_state.position.append(0.0)
        self.joint_state.velocity.append(0.0)
        self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.command_sub = rospy.Subscriber("velocity_controller1/command", Float64,
                                            self.command_callback1, queue_size=1)
        self.command_sub = rospy.Subscriber("velocity_controller2/command", Float64,
                                            self.command_callback2, queue_size=1)
        self.command_sub = rospy.Subscriber("velocity_controller3/command", Float64,
                                            self.command_callback3, queue_size=1)
        self.command_sub = rospy.Subscriber("velocity_controller4/command", Float64,
                                            self.command_callback4, queue_size=1)
        self.command_sub = rospy.Subscriber("stepper_motor/controller", Float64,
                                            self.command_callback5, queue_size=1)
        self.command_sub = rospy.Subscriber("servo_motor_controller1/command", Float64,
                                            self.command_callback6, queue_size=1)
        self.command_sub = rospy.Subscriber("servo_motor_controller2/command", Float64,
                                            self.command_callback7, queue_size=1)
        self.command_sub = rospy.Subscriber("servo_motor_controller3/command", Float64,
                                            self.command_callback8, queue_size=1)

    def command_callback1(self, msg):
        self.joint_state.position[0] = msg.data
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.joint_state)

    def command_callback2(self, msg):
        self.joint_state.position[0] = msg.data
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.joint_state)

    def command_callback3(self, msg):
        self.joint_state.position[0] = msg.data
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.joint_state)

    def command_callback4(self, msg):
        self.joint_state.position[0] = msg.data
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.joint_state)

    def command_callback5(self, msg):
        self.joint_state.position[0] = msg.data
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.joint_state)

    def command_callback6(self, msg):
        self.joint_state.position[0] = msg.data
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.joint_state)

    def command_callback7(self, msg):
        self.joint_state.position[0] = msg.data
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.joint_state)

    def command_callback8(self, msg):
        self.joint_state.position[0] = msg.data
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.joint_state)


if __name__ == '__main__':
    rospy.init_node('command_to_joint_state')
    command_to_joint_state = CommandToJointState()
    rospy.spin()