#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.
import rospy
import tf2_ros
import sys

from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
import numpy as np
import turtlebot_control

depth_buffer = [0,0,0]
mission_num = -1
#target_pos_in_frame: should be a 3*1 array that contain the xy positions and depth info of the object we are interested in
#mission: whether to do object following (0) or object searching (1)
#goal_object (string) :a string containing description/name of object

def turtlebot_main_function(goal_object):
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  try:
    rospy.init_node("turtlebot_controller")
  except rospy.exceptions.ROSException as e:
    print("Node has already been initialized, do nothing")
  rospy.Publisher('turtlebot_controller', Int8, queue_size=20)
  rospy.Subscriber("mission_num", Int8, mission_callback)
  if mission_num == 0: # Adjust and object following
    try:
        rospy.init_node("turtlebot_follower")
    except rospy.exceptions.ROSException as e:
        print("Node has already been initialized, do nothing")
    follow()

  rospy.spin()

#
def follow():
  #Create a publisher and a tf buffer, which is primed with a tf listener
  k1 = 0.1
  k2 = 1.0
  global mission_default
  angular_tolerance = 10
  translational_tolerance = 10
  image_centerX = 640
  image_centerY = 360
  pub_follow = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=20)
  rospy.Subscriber("target_pos_node", String, retrieve_callback)
  rospy.Subscriber("mission_num", Int8, mission_callback)
  r = rospy.Rate(20)
  while (mission_num == 0):
    command = Twist()
    diffX = target_pos_in_frame[0] - image_centerX
    diffY = target_pos_in_frame[1] - image_centerY
    if diffX >= angular_tolerance:
        angular_velocity = - k1 * diffX
    else:
        angular_velocity = 0

    depth_avg = depth_difference(depth_buffer)
    depth_diff = target_pos_in_frame[2] - depth_avg
    if (depth_diff >= translational_tolerance):
        translational_velocity = k2 * depth_diff
    else:
        translational_velocity = 0
        
    depth_buffer[0] = depth_buffer[1]
    depth_buffer[1] = depth_buffer[2]
    depth_buffer[2] = target_pos_in_frame[2]
    
    command.linear.x = translational_velocity
    command.angular.z = angular_velocity
    pub_follow.publish(command)
    r.sleep()

def mission_callback(int):
  global mission_num
  mission_num = int

def retrieve_callback(string):
  global target_pos_in_frame
  x,y,depth = string.split(",")
  x = float(x)
  y = float(y)
  depth = float(depth)
  target_pos_in_frame = [x,y,depth]

def depth_difference(depth_array):
  return 0.2 * depth_array[0] + 0.3 * depth_array[1] + 0.5 * depth_array[2]
  
#   controller_pub = rospy.Publisher('turtlebot_controller', Int8, queue_size=20)
#   mission_msg = Int8()
#   mission_msg.data = mission
#   controller_pub.publish(mission_msg)

    