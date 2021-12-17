#!/usr/bin/python


import darknet_ros_msgs
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox, ObjectCount
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg
import numpy as np
from geometry_msgs.msg import Twist
from random import randint
import time
import os
import sys
from std_msgs.msg import Int8
from time import sleep

class Turtle:
    
    def __init__(self, objects):
        self.mileage = 7
        #publish velocity
        # self.bounding_boxes_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_boxes_callback)
        
        # self.cmd = Twist()
        self.laser_msg = LaserScan()
        self.rotate = False
        
        #avoidance global parameters
        self.turn_needed = False
        self.turn_done_counter = 0
        self.stay_counter = 0
        self.velocity_buffer = [0,0,0,0] #for smoothness
        self.escape = 1
        self.detected = False
        self.search_object = objects

        self.vel_publisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 10)
        
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size = 10)


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
        if (not self.rotate) or (x == 0):
        # print("modified-x", x)
        # print("rotate_status", self.rotate)
            command.linear.x = x
            command.angular.z = z
            self.mileage += x
        # while not rospy.is_shutdown():
            self.vel_publisher.publish(command)
        # self.rate.sleep()



    #main functions:
    def rotate_mode(self):
        # rotate one full round
        self.rotate = True
        self.mileage = 0
        for i in range(85):
            print(i)
            print(self.mileage)
            self.publish_once_in_cmd_vel(0, 0.25)
            try:
                bounding_boxes_msg = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes, timeout = 0.3)
                self.bounding_boxes_callback(bounding_boxes_msg)
            except:
                self.publish_once_in_cmd_vel(0, 0.15)
                # print("velocity published, exception called")
            if self.detected:
                list = [0, 0, 0]
                for index in range(3):
                    self.detected = False
                    try:
                        bounding_boxes_msg = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes, timeout = 0.3)
                        self.bounding_boxes_callback(bounding_boxes_msg)
                    except:
                        break
                    if self.detected:
                        list[index] = 1
                    else:
                        break
                if sum(list) > 1:
                    self.detected = True
                    break
            sleep(0.2)
        self.rotate = False
        if self.detected:
            self.turn_into_following()
        # else:
        #     self.avoidance()

            
    def turn_into_following(self):
        # print("turn into another following")
        follow_object = self.search_object
        try:
            # self.vel_publisher.unregister()
            # self.laser_subscriber.unregister()
            # print('unregister turtle, currently following', follow_object)
            following_turtlebot = Following(follow_object)
            # if len(self.search_object) == 0:
            #     print('all objects found')
            #     exit()
        except Exception:
            print("exception")
            pass
        # following_turtlebot.rate.sleep()

    def avoidance(self):
        r_preferred = 0.55
        idx_prefered = 320
        temp = np.array(self.laser_msg.ranges)
        # temp[np.isnan(temp)] = 0
        idx = np.argsort(temp)[0:15]
        r_min_list = temp[idx]
        r_min_avg = np.average(r_min_list)
        
        r_min_avg = r_min_avg - r_preferred
        idx = idx - idx_prefered #renormalizing so that idx has range -320 ~ 319
        idx_avg = np.average(idx)
        x_gain = 0.18
        angle_gain = -0.0005
        vel_max = 0.4
        ang_vel_max = 1
        if (self.turn_needed):
            # print('turn!')
            if (self.turn_done_counter == 8):
                self.turn_done_counter = 0
                vel = 0
                ang_vel = 0.5
                self.turn_needed = False
            else:
                self.turn_done_counter += 1
                vel = 0
                ang_vel = 0.6
        else:
            if (r_min_avg < 0.25):
                vel = 0
                self.stay_counter += 1
            else:
                vel = x_gain * r_min_avg
            if (abs(idx_avg) < 30):
                ang_vel = 0
            else:
                ang_vel = angle_gain * idx_avg
        if self.stay_counter == 5:
            self.turn_needed = True
            self.stay_counter = 0
        
        # #for abnomality, too close
        # if r_min_avg > 5000:
        #     vel = 0
        #     ang_vel = 0
        #     self.stay_counter = 0
            
        self.velocity_buffer[1:3] = self.velocity_buffer[:2]
        self.velocity_buffer[0] = vel
        vel = self.velocity_buffer[0]*0.4 +self.velocity_buffer[1] * 0.3 + self.velocity_buffer[2] * 0.2 + self.velocity_buffer[3] * 0.1
        vel = min(vel, vel_max)
        # print('vel', vel)
        # print('mileage', self.mileage)
        ang_vel = min(ang_vel, ang_vel_max)
        sleep(0.13)
        self.publish_once_in_cmd_vel(vel, ang_vel)
        

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
        if self.mileage < 7:
            self.avoidance()
            # print('call avoidance')
        else:
            self.rotate_mode()

    def bounding_boxes_callback(self, boundingbox_msg):
        for i in range(len(boundingbox_msg.bounding_boxes)):
            # print(boundingbox_msg.bounding_boxes[i].Class)
            if boundingbox_msg.bounding_boxes[i].Class == self.search_object:
                self.detected = True
                # print('detected', self.mileage)  
        if not self.detected:
            # print('turtle class not detected')
            return
    

class Following:
    #480 * 640
    
    def __init__(self, object_names):
        self.center_x = 320
        self.center_y = 240
        self.turn_value = 0.25
        self.linear_value = 0.1
        self.linear_tol = 0.05
        self.linear_coefficient = 0.15
        self.angular_tol = 15
        self.angular_coefficient = 0.0012
        self.object_names = object_names

        self.command = Twist()
        #set up publisher and subscriber
        self.xmin = 0
        self.xmax = 640
        self.ymin = 0
        self.ymax = 480
        self.sampling_depth = None
        self.control_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 0)
        self.bridge = CvBridge()
        self.depth_msg = None
        self.counter = 0

        # person following filters
        self.person_minimum_area = 13000

        # control buffers
        self.linear_velocity_buffer = [0, 0, 0]
        self.linear_velocity_max = 0.2

        '''TODO: define callback function'''
        self.bounding_boxes_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_boxes_callback)
        self.depth_callback = rospy.Subscriber('/camera/depth_registered/image', Image, self.depth_callback)
        # self.detection_image_sub = rospy.Subscriber('/darknet_ros/detection_image', BoundingBoxes, self.bounding_boxes_callback)
        # self.depth_sub = rospy.Subscriber('/camera/depth_registered/image', Image, self.depth_callback)
        # print('synchronized')

        

    # def depth_callback(self, image):
    #     self.image = image

    def bounding_boxes_callback(self, boundbox_msg):
        # print('doing angular callback')
        # depth_msg = rospy.wait_for_message('/camera/depth_registered/image', Image)

        detected = False
        # if we are tracking person, we want the person with the biggest frame area
        if 'person' in self.object_names:
            best_area = float('-inf')
            for i in range(len(boundbox_msg.bounding_boxes)):
                oj = boundbox_msg.bounding_boxes[i]
                area = (oj.ymax - oj.ymin) * (oj.xmax - oj.xmin)
                if (oj.Class in self.object_names) and ((area > best_area) and area > self.person_minimum_area):
                    cur_frame = oj
                    detected = True
                    best_area = area
        else:
            for i in range(len(boundbox_msg.bounding_boxes)):
                if boundbox_msg.bounding_boxes[i].Class in self.object_names:
                    cur_frame = boundbox_msg.bounding_boxes[i]
                    detected = True
                    break   
        if not detected:
            print('not detected and exit')
            return
        #detected object in frame
        else:
            self.xmin = cur_frame.xmin
            self.ymin = cur_frame.ymin
            self.xmax = cur_frame.xmax
            self.ymax = cur_frame.ymax
            name = cur_frame.Class
            self.command = Twist()
            adjusted_angular = self.adjust_orientation(self.xmin, self.ymin, self.xmax, self.ymax)
            if self.sampling_depth != None:
                goal_not_achieved = self.adjust_depth(self.sampling_depth) 
                print(self.command.linear.x)
        if self.command.linear.x != 0:
            print(self.linear_velocity_buffer)
            buf = self.linear_velocity_buffer
            self.command.linear.x = min(0.4 * self.command.linear.x + 0.3 * buf[0] + 0.2 * buf[1] + 0.1 * buf[2], self.linear_velocity_max)
        if (self.ymax + self.ymin)/2 >= 396 and abs((self.xmin + self.xmax) / 2  - 320) <= 20:
            print('should exit whole program')
            nodes = os.popen("rosnode list").readlines()
            for i in range(len(nodes)):
                nodes[i] = nodes[i].replace("\n","")
            for node in nodes:
                if ('main_node' in node) or ('darknet' in node):
                    os.system("rosnode kill "+ node)

        self.linear_velocity_buffer[1:3] = self.linear_velocity_buffer[0:2]
        self.linear_velocity_buffer[0] = self.command.linear.x
        self.control_pub.publish(self.command)
        print('linear: ', self.command.linear.x)
        print('angular', self.command.angular.z)
        

    def depth_callback(self, depth_msg):
        self.depth_msg = depth_msg
        cv_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        depth_matrix = np.array(cv_image)
        xmin = self.xmin
        xmax = self.xmax
        ymin = self.ymin
        ymax = self.ymax

        center = [xmin + (xmax - xmin)/2, ymin + (ymax - ymin)/2]
        chord = 5
        new_xmin = center[0] - chord
        new_xmax = center[0] + chord
        new_ymin = center[1] - chord
        new_ymax = center[1] + chord

        sampling_matrix = depth_matrix[new_ymin:new_ymax, new_xmin:new_xmax]
        sampling_depth = np.nanmean(sampling_matrix)

        self.sampling_depth = sampling_depth
        
    def adjust_orientation(self, xmin, ymin, xmax, ymax):
        #adjust the angular twist of the turtlebot to let object be in the center of the image
        xmid_point = xmin + (xmax - xmin)/2
        print('xmid',xmid_point)
        error = xmid_point - self.center_x
        if xmid_point > self.center_x and abs(xmid_point - self.center_x) > self.angular_tol:
            self.command.angular.z = -self.angular_coefficient * error
            return True
        elif xmid_point < self.center_x and abs(xmid_point - self.center_x) > self.angular_tol:
            self.command.angular.z = -self.angular_coefficient * error
            return True
        self.command.angular.z = 0
        
        return False
    
    def adjust_depth(self, sampling_depth):
        #adjust the depth to let image be of a fixed size
        target_depth = 0.35
        tol = self.linear_tol
        if sampling_depth > target_depth and abs(sampling_depth - target_depth) > tol:
            self.command.linear.x = self.linear_coefficient * (sampling_depth - target_depth)
            return True
        self.command.linear.x = 0
        return False

    
# main call
if __name__ == '__main__':
    #initialize node
    # objects = []

    # for i in sys.argv[1:]:
    #     objects.append(i)
    objects = sys.argv[1]
    print(objects)
    rospy.init_node('main_node', anonymous=True)
    try:
        # robotcontrol_object.avoidance()
        robotcontrol_object = Turtle(objects)
    except rospy.ROSInterruptException:
        print("exception")
        pass
    rospy.spin()