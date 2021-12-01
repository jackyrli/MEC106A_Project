#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg
import numpy as np
from geometry_msgs.msg import Twist
from random import randint
import time

class Turtle:
    
    def __init__(self):
        
        #initialize node
        rospy.init_node('turtlebot_control', anonymous=True)
        
        #publish velocity
        self.vel_publisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        # self.cmd = Twist()
        self.laser_msg = LaserScan()
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        
        #avoidance global parameters
        self.turn_needed = False
        self.turn_done_counter = 0
        self.stay_counter = 0
        self.velocity_buffer = [0,0,0] #for smoothness
        self.escape = 1
        
    def voice_command(self):
        return
    
    #use this function to publish the velocity
    def publish_once_in_cmd_vel(self, x, z):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        command = Twist()
        # self.cmd.linear.x = x
        # self.cmd.angular.z = z
        command.linear.x = x
        command.angular.z = z
        # while not rospy.is_shutdown():
        self.vel_publisher.publish(command)
        # self.rate.sleep()


    
    #main functions:
    def avoidance(self):
        r_preferred = 0.4
        idx_prefered = 320
        temp = np.array(self.laser_msg.ranges)
        temp[np.isnan(temp)] = 1000000
        idx = np.argsort(temp)[0:15]
        r_min_list = temp[idx]
        r_min_avg = np.average(r_min_list)
        
        r_min_avg = r_min_avg - r_preferred
        idx = idx - idx_prefered #renormalizing so that idx has range -320 ~ 319
        idx_avg = np.average(idx)
        x_gain = 0.2
        angle_gain = -0.001
        vel_max = 0.5
        ang_vel_max = 1.5
        if (self.turn_needed):
            print('turn!')
            if (self.turn_done_counter == 10):
                self.turn_done_counter = 0
                vel = 0
                # rand = (randint(0,1) - 0.5) *2
                # print(rand)

                # escape = rand
                ang_vel = 1
                self.turn_needed = False
            else:
                self.turn_done_counter += 1
                vel = 0
                ang_vel = 1.5
        else:
            if (r_min_avg < 0.15):
                vel = 0
                self.stay_counter += 1
            else:
                vel = x_gain * r_min_avg
            if (abs(idx_avg) < 20):
                ang_vel = 0
            else:
                ang_vel = angle_gain * idx_avg
        if self.stay_counter == 5:
            self.turn_needed = True
            self.stay_counter = 0
        
        #for abnomality, too close
        if r_min_avg > 5000:
            vel = 0
            ang_vel = 0
            self.stay_counter = 0
            
        self.velocity_buffer[1:3] =self.velocity_buffer[:2]
        self.velocity_buffer[0] = vel
        vel = self.velocity_buffer[0]*0.5 +self.velocity_buffer[1] * 0.3 + self.velocity_buffer[2] * 0.2
        vel = min(vel, vel_max)
        ang_vel = min(ang_vel, ang_vel_max)
        self.publish_once_in_cmd_vel(vel, ang_vel)
        print(self.turn_needed, r_min_avg, idx_avg, idx)

    # utility functions to get sensor values
    def get_laser_range(self):
        #returns a list of laser ranges
        # self.rate.sleep()
        return self.laser_msg.ranges
    
    def get_laser(self, idx):
        # self.rate.sleep()
        return self.laser_msg.ranges[idx]
    
    #callbacks
    def laser_callback(self, msg):
        self.laser_msg = msg
        print("callback_received")
        self.avoidance()
    
        
# main call
if __name__ == '__main__':
    #rospy.init_node('robot_control_node', anonymous=True)laser_callback
    robotcontrol_object = Turtle()
    try:
        robotcontrol_object.avoidance()
    except rospy.ROSInterruptException:
        print("exception")
        pass
    rospy.spin()