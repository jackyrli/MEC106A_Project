#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox, ObjectCount
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys
import os

class Following:
    def __init__(self, object_names):
        self.center_x = 320
        self.center_y = 240
        self.turn_value = 0.25
        self.linear_value = 0.1
        self.linear_tol = 0.05
        self.linear_coefficient = 0.2
        self.linear_derivative = 0.15
        self.angular_tol = 15
        self.angular_coefficient = 0.0015
        self.angular_derivative = 0.001
        self.object_names = object_names
        self.rate = rospy.Rate(50)
        self.command = Twist()
        #set up publisher and subscriber
        self.xmin = 0
        self.xmax = 640
        self.ymin = 0
        self.ymax = 480
        self.sampling_depth = None
        self.target_depth = 0.3
        self.control_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 0)
        self.bridge = CvBridge()
        self.depth_msg = None
        # This denotes the pervious error and will be used for Derivative calculation.
        self.error = 0
        self.angular_error = 0
        
        # person following filters, if the area of the boundingbox is too small, it is likely that the person is not the one we are following.
        self.person_minimum_area = 13000
        # Set the highest velocity threshold and create a four step buffer to smoothen the turtlebot movement.
        self.linear_velocity_buffer = [0, 0, 0, 0]
        self.linear_velocity_max = 0.4

        self.bounding_boxes_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_boxes_callback)
        self.depth_callback = rospy.Subscriber('/camera/depth_registered/image', Image, self.depth_callback)

    def bounding_boxes_callback(self, boundbox_msg):
        # depth_msg = rospy.wait_for_message('/camera/depth_registered/image', Image)
        detected = False
        # See if we have detected one person and how big that person is.
        # If the box does not exceed threshold, we deem this box to be invalid and will drop the message.
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
            print('not detected')
            return
        # We have detected object in the frame and would need to approach following state and try to keep the box within the center of our frame.
        else:
            self.xmin = cur_frame.xmin
            self.ymin = cur_frame.ymin
            self.xmax = cur_frame.xmax
            self.ymax = cur_frame.ymax
            name = cur_frame.Class
            self.command = Twist()
            adjusted_angular = self.adjust_orientation(self.xmin, self.ymin, self.xmax, self.ymax)
            if self.sampling_depth != None:
                self.adjust_depth(self.sampling_depth) 
                print(self.command.linear.x)
        if self.command.linear.x != 0:
            self.linear_velocity_buffer[1:3] = self.linear_velocity_buffer[0:2]
            self.linear_velocity_buffer[0] = self.command.linear.x
            weighted_vel = self.velocity_buffer[0]*0.4 +self.velocity_buffer[1] * 0.3 + self.velocity_buffer[2] * 0.2 + self.velocity_buffer[3] * 0.1
            self.command.linear.x = min(weighted_vel, self.linear_velocity_max)
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
        following_turtlebot.rate.sleep()
        
    def adjust_orientation(self, xmin, ymin, xmax, ymax):
        #adjust the angular twist of the turtlebot to let object be in the center of the image
        xmid_point = xmin + (xmax - xmin)/2
        print('xmid',xmid_point)
        error = xmid_point - self.center_x
        derivative = error - self.error
        self.error = error
        if abs(xmid_point - self.center_x) > self.angular_tol:
            self.command.angular.z = -self.angular_coefficient * error - self.angular_coefficient * derivative
            return True
        else:
            self.command.angular.z = 0
            return False
    
    def adjust_depth(self, sampling_depth):
        #adjust the depth to let image be of a fixed size
        # The pointcloud depth data may vary a bit from time to time so we use a tolerance to make
        # turtlebot stop when it is within a specific range
        error = sampling_depth - self.target_depth
        #This term calculates a one time derivative term of error
        derivative = error - self.error
        self.error = error
        if sampling_depth > self.target_depth and abs(sampling_depth - self.target_depth) > self.linear_tol:
            self.command.linear.x = self.linear_coefficient * (sampling_depth - self.target_depth) + self.linear_derivative * derivative
            return True
        else:
            self.command.linear.x = 0
            return False
    
if __name__ == "__main__":
    #Taking a string input telling us which object to track.
    follow_object = [sys.argv[1]]
    rospy.init_node('following_node', anonymous=True)
    print('INITIALIZED NODE')
    try:
        following_turtlebot = Following(follow_object)
    except rospy.ROSInterruptException:
        print("exception")
        pass
    rospy.spin()