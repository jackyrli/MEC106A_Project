#!/usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg
import numpy as np
from geometry_msgs.msg import Twist
from random import randint

turn_needed = False
turn_done_counter = 0
stay_counter = 0
velocity_buffer = [0,0,0] #for smoothness
escape = 1
pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
def callback(msg):
    global turn_needed
    global turn_done_counter
    global stay_counter
    global velocity_buffer
    global escape
    # Loop over all ranges in the LaserScan.
    #print(len(msg.ranges)) #should be 640

    # seg0 = msg.ranges[0:159]
    # seg1 = msg.ranges[160:319]
    # seg2 = msg.ranges[320:479]msg.ranges
    # seg3 = msg.ranges[480:639]
    # seg0 = seg0(~np.isnan(seg0))
    # seg1 = seg1(~np.isnan(seg1))
    # seg2 = seg2(~np.isnan(seg2))
    # seg3 = seg3(~np.isnan(seg3))
    # for idx, r in enumerate(msg.ranges):
    #     # Randomly throw out some rays to speed this up.
    #     if np.isnan(r):if (turn_needed):
    #         seg0.append(r)
    #     elif idx >= 240 and idx <= 319:
    #         seg1.append(r)
    #     elif idx >= 320 and idx <= 399:
    #         seg2.append(r)
    #     elif idx >= 400 and idx <= 639:
    #         seg3.append(r)
    # seg = [np.average(seg0), np.average(seg1), np.average(seg2), np.average(seg3)]


    #640/4 segmentation of length from obstacle
    
    # min_seg = min([min(seg1), min(seg2)])
    # max_seg = min([np.average(seg1), np.average(seg2)])
    # print('min', min_seg)
    # print('max ', max_seg)
    # #min_thres
    # min_thres = 0.3
    # #max_thres, if all seg are larger than this one, don't turn and keep straight
    # max_thres = 2.5
    # direction = seg.index(min(seg))
    # #logic control
    # if direction == 0:
    #     vel = 0.1
    #     ang_vel = -0.3
    # elif direction == 1:
    #     vel = 0.03
    #     ang_vel = -2
    # elif direction == 2:
    #     vel = 0.03
    #     ang_vel = 2
    # else:
    #     vel = 0.1
    #     ang_vel = 0.3

    # if max_seg >= max_thres:
    #     vel = 0.2
    #     ang_vel = 0
    # if min_seg < min_thres:
    #     vel = 0
    #     ang_vel = 3

    # print('minseg1', min(seg1))
    # print('minseg2', min(seg2))
    
    
    # implementation 11/22/2021
    r_preferred = 0.6
    idx_prefered = 320
    temp = np.array(msg.ranges)
    temp[np.isnan(temp)] = 1000000
    idx = np.argsort(temp)[0:10]
    r_min_list = temp[idx]
    r_min_avg = np.average(r_min_list)
    
    r_min_avg = r_min_avg - r_preferred
    idx = idx - idx_prefered #renormalizing so that idx has range -320 ~ 319
    idx_avg = np.average(idx)
    x_gain = 0.16
    angle_gain = -0.001
    vel_max = 0.5
    ang_vel_max = 1.5
    if (turn_needed):
        print('turn!')
        if (turn_done_counter == 10):
            turn_done_counter = 0
            vel = 0
            # rand = (randint(0,1) - 0.5) *2
            # print(rand)

            # escape = rand
            ang_vel = 1
            turn_needed = False
        else:
            turn_done_counter += 1
            vel = 0
            ang_vel = 1.5
            
    else:
        if (abs(r_min_avg) < 0.2):
            vel = 0
            stay_counter += 1
        else:
            vel = x_gain * r_min_avg
        if (abs(idx_avg) < 30):
            ang_vel = 0
        else:
            ang_vel = angle_gain * idx_avg
    if stay_counter == 5:
        turn_needed = True
        stay_counter = 0
    
    #for abnomality, too close
    if r_min_avg > 5000:
        vel = 0
        ang_vel = 0
        stay_counter = 0

    
    velocity_buffer[1:3] = velocity_buffer[:2]
    velocity_buffer[0] = vel
    vel = velocity_buffer[0]*0.5 + velocity_buffer[1] * 0.3 + velocity_buffer[2] * 0.2
    vel = min(vel, vel_max)
    ang_vel = min(ang_vel, ang_vel_max)
    command = Twist()
    command.linear.x = vel
    command.angular.z = ang_vel
    pub.publish(command)
    print(turn_needed, r_min_avg)

        
    
def listener():
    rospy.init_node('avoidance')
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()
    
    
if __name__ == '__main__':
    listener()
    self.rate = rospy.Rate(1)