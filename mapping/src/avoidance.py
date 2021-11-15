#!/usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg
import numpy as np
from geometry_msgs.msg import Twist


pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
def callback(msg):
    # Loop over all ranges in the LaserScan.
    #print(len(msg.ranges)) #should be 640

    # seg0 = msg.ranges[0:159]
    # seg1 = msg.ranges[160:319]
    # seg2 = msg.ranges[320:479]
    # seg3 = msg.ranges[480:639]
    # seg0 = seg0(~np.isnan(seg0))
    # seg1 = seg1(~np.isnan(seg1))
    # seg2 = seg2(~np.isnan(seg2))
    # seg3 = seg3(~np.isnan(seg3))
    seg0 = []
    seg1 = []
    seg2 = []
    seg3 = []
    for idx, r in enumerate(msg.ranges):
        # Randomly throw out some rays to speed this up.
        if np.isnan(r):
            continue
        if idx <= 239 and idx >= 0:
            seg0.append(r)
        elif idx >= 240 and idx <= 319:
            seg1.append(r)
        elif idx >= 320 and idx <= 399:
            seg2.append(r)
        elif idx >= 400 and idx <= 639:
            seg3.append(r)
    seg = [np.average(seg0), np.average(seg1), np.average(seg2), np.average(seg3)]
            

    #640/4 segmentation of length from obstacle
    
    min_seg = min([min(seg1), min(seg2)])
    max_seg = min([np.average(seg1), np.average(seg2)])
    print('min', min_seg)
    print('max ', max_seg)
    #min_thres
    min_thres = 0.3
    #max_thres, if all seg are larger than this one, don't turn and keep straight
    max_thres = 2.5
    direction = seg.index(min(seg))
    #logic control
    if direction == 0:
        vel = 0.1
        ang_vel = -0.3
    elif direction == 1:
        vel = 0.03
        ang_vel = -2
    elif direction == 2:
        vel = 0.03
        ang_vel = 2
    else:
        vel = 0.1
        ang_vel = 0.3

    if max_seg >= max_thres:
        vel = 0.2
        ang_vel = 0
    if min_seg < min_thres:
        vel = 0
        ang_vel = 3

    # print('minseg1', min(seg1))
    # print('minseg2', min(seg2))
    command = Twist()
    command.linear.x = vel
    command.angular.z = ang_vel
    pub.publish(command)
    print("success_publish")

        
    
def listener():
    rospy.init_node('avoidance')
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()
    
    
if __name__ == '__main__':
    listener()
    