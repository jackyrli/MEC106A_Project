#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox, ObjectCount
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from enum import Enum
from random import randint
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from time import sleep
import numpy as np
import sys
import os
import cv2


# This is our finite state machine for search and it is very helpful.
# We define four states in our search program.
class Status(Enum):
    #normal means we are running normally using pid control to keep distance from obstacle.
    # We will not do big rotations in this state.
    normal = 0
    # If the PID info from the laserscan is not enough to keep away from the obstacle and we are stuck.
    # We enter this state and we need to rotate until the laserscan reveals that there is no obstacle in front.
    block_avoid = 1
    # This is the state when the mileage reading exceeds the threshold and we need to do a whole rotation check 
    # to use camera data to find if there is object.
    search_rotate = 2
    # This is when the search does find something, we would turn into approach mode and try to approach the object.
    approach = 3
    # This is when we finally find the object and have to stop.
    goal = 4

class searchMaster:
    def __init__(self, object_names):
        self.status = Status.normal
        self.mileage = 7
        self.r_preferred = 0.4
        self.idx_prefered = 320
        self.rotate = False
        self.r_min_avg = 0.7
        self.idx_min_avg = 0
        #avoidance global parameters
        self.turn_needed = False
        self.detected = False
        self.search_object = object_names
        # This is the frequency that yolo network can handle the image input. 
        self.rate = rospy.Rate(22)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size = 1)
        # self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size = 1)
        self.vel_publisher = rospy.Publisher('control_master/vel', Twist, queue_size = 1)
        self.bounding_boxes_sub = None
        self.depth_subscriber = None

        ## These are parameters for following.
        self.center_x = 320
        self.center_y = 240
        self.turn_value = 0.25
        self.linear_value = 0.1
        self.linear_tol = 0.05
        self.linear_coefficient = 0.15
        self.angular_tol = 15
        self.angular_coefficient = 0.0012
        self.xmin = 0
        self.xmax = 640
        self.ymin = 0
        self.ymax = 480
        self.sampling_depth = None
        self.bridge = CvBridge()
        self.depth_msg = None
        self.counter = 0
        # person following filters
        self.person_minimum_area = 13000
        self.z = None
        self.x = None


    def loop(self):
        while not (self.status == Status.goal):
            if self.status == Status.normal:
                self.normal()
            elif self.status == Status.block_avoid:
                self.avoid()
            elif self.status == Status.search_rotate:
                self.search()
            elif self.status == Status.approach():
                self.approach()
                # This is because status goal is certain to be reached after approach state ends.
                # And the finite state machine does not need to output anything after we have entered the approach state.
                self.status = Status.goal
            self.rate.sleep()

    def normal(self):
        x_gain = 0.18
        angle_gain = -0.0005
        vel_max = 0.4
        ang_vel_max = 1
        # The turn_needed is only true when we have stopped at the same point doing nothing for five timestamps.
        if self.stay_counter == 5:
            self.turn_needed = True
            self.stay_counter = 0
        if (self.mileage > 7):
            self.status = Status.search_rotate
        if (self.turn_needed):
            self.status = Status.block_avoid
        else:
            if (self.r_min_avg < 0.25):
                vel = 0
                self.stay_counter += 1
            else:
                vel = x_gain * self.r_min_avg
            # This is to elinminate the oscillatory behavior by telling the robot to take no behavior when error is too small.
            if (abs(self.idx_avg) < 45):
                ang_vel = 0
            else:
                ang_vel = angle_gain * self.idx_avg
            vel = min(vel, vel_max)
            ang_vel = min(ang_vel, ang_vel_max)
            self.publish_cmd_vel(vel, ang_vel)
    
    # We have entered this state because we have detected object ahead and we need to turn to avoid.
    def avoid(self):
        turn_done_counter = 0
        ang_vel = 0.5
        vel = 0
        while (turn_done_counter > 9):
            self.publish_cmd_vel(vel, ang_vel)
            sleep(0.1)
        self.turn_needed = False
        # Switch back to normal to see if we have done enough rotations.
        self.status = Status.normal
    
    # This is when we have exceeded the mileage threshold and we need to rotate and search for object.
    def search(self):
        self.mileage = 0
        for i in range(60):
            try:
                bounding_boxes_msg = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes, timeout = 0.25)
                self.bounding_boxes_callback(bounding_boxes_msg)
                self.publish_once_in_cmd_vel(0, 0.25)
            # When the previous wait timesout, we know the yolo network hasn't discovered any objects and we will let turtlebot
            # rotate a little bit to change view.
            except:
                self.publish_once_in_cmd_vel(0, 0.15)
            if self.detected:
                # Due to the possibility of mis detection, we will record the last three frames detection data.
                # Only if we have seen the same object for more than 2/3 of the frames will we deem that the object is found.
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
        if self.detected:
            self.status = Status.approach
        else:
            self.status = Status.normal

    def approach(self):
        self.bounding_boxes_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_boxes_callback_following)
        self.depth_subscriber = rospy.Subscriber('/camera/depth_registered/image', Image, self.depth_callback)

    def bounding_boxes_callback(self, boundingbox_msg):
        for i in range(len(boundingbox_msg.bounding_boxes)):
            if boundingbox_msg.bounding_boxes[i].Class == self.search_object:
                self.detected = True
        if not self.detected:
            return

    def laser_callback(self, msg):
        # Approach is a locked state and you can only go to goal from this state.
        # We will unregister the laserscan subscriber to remove any future callbacks.
        if (self.status == Status.approach):
            self.laser_subscriber.unregister()
        # Preprocess the input msg data to obtain the index average and the r average.
        else:
            temp = np.array(msg.ranges)
            idx = np.argsort(temp)[0:15]
            r_min_list = temp[idx]
            r_min_avg = np.average(r_min_list)
            self.r_min_avg = r_min_avg - self.r_preferred
            #renormalizing so that idx has range -320 ~ 319
            idx = idx - self.idx_prefered 
            self.idx_avg = np.average(idx)

    def publish_cmd_vel(self, x, z):
        command = Twist()
        if (not self.rotate) or (x == 0):
            command.linear.x = x
            command.angular.z = z
            self.mileage += x
            self.vel_publisher.publish(command)

    def bounding_boxes_callback_following(self, boundbox_msg):
        best_area = float('-inf')
        for i in range(len(boundbox_msg.bounding_boxes)):
            oj = boundbox_msg.bounding_boxes[i]
            area = (oj.ymax - oj.ymin) * (oj.xmax - oj.xmin)
            # We are iterating through all objects that we have detecected in this frame and choosing the
            # biggest one for use in future calculation.
            if (oj.Class in self.object_names) and (area > best_area):
                cur_frame = oj
                best_area = area 
        if not self.detected:
            print('not detected and exit')
            return
        # Detected object in the frame, and we need to adjust our orientation and depth to make sure we are centering the object.
        else:
            self.xmin = cur_frame.xmin
            self.ymin = cur_frame.ymin
            self.xmax = cur_frame.xmax
            self.ymax = cur_frame.ymax
            name = cur_frame.Class
            self.command = Twist()
            adjusted_angular = self.adjust_orientation(self.xmin, self.xmax)
            if self.sampling_depth != None:
                goal_not_achieved = self.adjust_depth(self.sampling_depth)
        # These parameters are tested through experiment, when the turtlebot gets too close to the
        # object, the object stays at the bottom of the frame, so we can utilize this and stop once
        # the object is partially visible and in the bottom of the frame (which indicates that the turtlebot is close enough to the object). 
        if (self.ymax + self.ymin)/2 >= 396 and abs((self.xmin + self.xmax) / 2  - 320) <= 20:
            # When we exit we have reached the goal state and we need to kill the node and stop.
            print('should exit whole program')
            # The image robot seen will be shown to the viewer.
            os.system("rosrun image_view image_view image:=/camera/rgb/image_raw")
            nodes = os.popen("rosnode list").readlines()
            for i in range(len(nodes)):
                nodes[i] = nodes[i].replace("\n","")
            for node in nodes:
                if ('vel_control' in node) or ('darknet' in node) or ('searchMaster' in node):
                    os.system("rosnode kill "+ node)
        else:
            self.publish_cmd_vel(self.x, self.z)
        
    def depth_callback(self, depth_msg):
        self.depth_msg = depth_msg
        cv_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        depth_matrix = np.array(cv_image)
        xmin = self.xmin
        xmax = self.xmax
        ymin = self.ymin
        ymax = self.ymax
        #Calculate the depth average of the middle part of the searching object to determine its overall depth.
        center = [xmin + (xmax - xmin)/2, ymin + (ymax - ymin)/2]
        chord = 5
        new_xmin = center[0] - chord
        new_xmax = center[0] + chord
        new_ymin = center[1] - chord
        new_ymax = center[1] + chord
        sampling_matrix = depth_matrix[new_ymin:new_ymax, new_xmin:new_xmax]
        self.sampling_depth = np.nanmean(sampling_matrix)
        
    def adjust_orientation(self, xmin,xmax):
        #adjust the angular twist of the turtlebot to let object be in the center of the image
        xmid_point = xmin + (xmax - xmin)/2
        error = xmid_point - self.center_x
        if xmid_point > self.center_x and abs(xmid_point - self.center_x) > self.angular_tol:
            self.z = -self.angular_coefficient * error
            return True
        elif xmid_point < self.center_x and abs(xmid_point - self.center_x) > self.angular_tol:
            self.z = -self.angular_coefficient * error
            return True
        else:
            self.z = 0
            return False
    
    def adjust_depth(self, sampling_depth):
        #adjust the depth to let image be of a fixed size
        target_depth = 0.35
        tol = self.linear_tol
        if sampling_depth > target_depth and abs(sampling_depth - target_depth) > tol:
            self.x = self.linear_coefficient * (sampling_depth - target_depth)
            return True
        else:
            self.x = 0
            return False

if __name__ == "__main__":
    #Taking a string input telling us which object to track.
    objects = sys.argv[1]
    print(objects)
    rospy.init_node('searchMaster', anonymous=True)
    bot_search = searchMaster(objects)
    bot_search.loop()
    rospy.spin()