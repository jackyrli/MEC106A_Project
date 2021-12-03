import numpy as np
import rospy
from geometry_msgs.msg import Twist

from darknet_ros_msgs.msgs import BoundingBoxes, BoundingBox, ObjectCount
import BoundingBox.msgs

class Following():
    
    
    def __init__(self, object_name):
        self.object_name = object_name
        self.rate = rospy.Rate(20)
        self.command = None
        #set up publisher and subscriber
        self.control_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
        
        '''TODO: define callback function'''
        self.bounding_boxes_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, bounding_boxes_callback)
        
        
        '''TODO: find out the center of the image'''
        self.center_x = 
        self.center_y = 
    
    def bounding_boxes_callback(self, msg):
        for i in range(len(msg.bounding_boxes)):
            if msg.bounding_boxes[i] == self.object_name:
                cur_frame = msg.bounding_boxes[i]
        xmin = cur_frame.xmin
        ymin = cur_frame.ymin
        xmax = cur_frame.xmax
        ymax = cur_frame.ymax
        name = cur_frame.Class
        self.command = Twist()
        adjusted_angular = self.adjust_orientation(xmin, ymin, xmax, ymax)
        if adjusted_angular == False:
            adjusted_depth = self.adjust_adjust_depth(xmin, ymin, xmax, ymax)
        self.control_pub.publish(self.command)
        print('angular: ', self.command.angular.z)
        print('linear: ', self.command.linear.x)
        
        
    def adjust_orientation(self, xmin, ymin, xmax, ymax):
        #adjust the angular twist of the turtlebot to let object be in the center of the image
        xmid_point = xmin + (xmin + xmax)/2
        tol = 10
        if xmid_point > self.center_x and abs(xmid_point - self.center_x) > tol:
            self.command.angular.z = 0.3
            return True
        return False
    
    def adjust_depth(self, xmin, ymin, xmax, ymax):
        #adjust the depth to let image be of a fixed size
        current_area = (xmax - xmin) * (ymax - ymin)
        desired_area = '''TODO: find desired area'''
        tol = 20
        if current_area < desired_area and abs(current_area - desired_area) > tol:
            self.command.linear.x = 0.3
            return True
        return False

if __name__ == "__main__":
    rospy.init_node("following_node")
    rospy.spin()