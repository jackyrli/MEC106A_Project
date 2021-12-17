#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from enum import Enum
from random import randint
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from time import sleep
import numpy as np
import sys

class Controller():
    def __init__(self):
        #linear velocity changes much more slower so we use a five-step buffer
        self.velocity_buffer = [0, 0, 0, 0, 0]
        self.rate = rospy.Rate(20)
        #angular velocity changes much faster so we use a three-step buffer
        self.z_buffer = [0, 0, 0]
        self.velocity_subscriber = rospy.Subscriber('control_master/vel', Twist, self.velocity_callback, queue_size = 5)
        self.velocity_publisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 5)
    
    #During the callback, we will process the data and add in buffers.
    def velocity_callback(self, msg):
        self.velocity_buffer[1:5] = self.velocity_buffer[0:4]
        self.velocity_buffer[0] = msg.linear.x
        self.z_buffer[1:3] = self.z_buffer[0:2]
        self.z_buffer[0] = msg.angular.z
        angular_z = self.z_buffer[0] * 0.5 + self.z_buffer[1] * 0.3 + self.z_buffer[2] * 0.2
        linear_x = self.velocity_buffer[0] * 0.4 + self.velocity_buffer[1] * 0.25 + self.velocity_buffer[2] * 0.15 + self.velocity_buffer[3] * 0.1 + self.velocity_buffer[4] * 0.1
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.velocity_publisher.publish(command)

# main call
if __name__ == '__main__':
    objects = sys.argv[1]
    print(objects)
    rospy.init_node('vel_control', anonymous=True)
    try:
        robotcontrol_object = Controller()
    except rospy.ROSInterruptException:
        print("exception")
        pass
    rospy.spin()