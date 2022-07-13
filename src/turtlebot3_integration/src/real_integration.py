#!/usr/bin/env python3
from email.errors import ObsoleteHeaderDefect
from statistics import mode
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from move_robot import MoveTurtlebot3
from darknet_ros_msgs.msg import BoundingBoxes

flag = 0
mode = 0 
stop = 0 
auto_nav_object = 0
stop_once = 1

class LineFollower(object):
    def __init__(self):
        global flag
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image/compressed",Image,self.camera_callback)
        self.stop_sign_subscriber = rospy.Subscriber('/darknet_ros/bounding_boxes' , BoundingBoxes, self.stop_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()

    def stop_callback(self,msg):
        global stop
        stop = 0
        if msg.bounding_boxes[len(msg.bounding_boxes)- 1].id == 11:
            stop = 1

    def camera_callback(self, data):
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            
        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        crop_img = cv_image[int((height/2)+100):int((height/2)+120)][1:int(width)]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    
        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([10,100,100])
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        global mode
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            mode = 1 
        except ZeroDivisionError:
            cx, cy = height/2, width/2
            
        # Draw the centroid in the resultant image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask,(int(cx), int(cy)), 7, (0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        # controller
        if mode == 1:
            err_x = cx - width/2
            twist_object = Twist()
            twist_object.linear.x = 0.1
            twist_object.angular.z = -err_x/300
            global stop
            global stop_once
            if stop == 1:
                if stop_once == 1:
                    rospy.sleep(6)
                    twist_object.linear.x = 0
                    twist_object.angular.z = 0
                    self.moveTurtlebot3_object.move_robot(twist_object)
                    rospy.sleep(3)
                    stop = 0
                    stop_once = 0

            # Make it start turning
            self.moveTurtlebot3_object.move_robot(twist_object)

        def clean_up(self):
            self.moveTurtlebot3_object.clean_class()
            cv2.destroyAllWindows()

def obstacle_avoidance_callback(laserscan): # defining a wall following function
    global err_front
    global err_side
    
    """ For lateral control """
    laserscan = list(laserscan.ranges[0:360]) # storing LiDAR data 
    
    right = laserscan[-90:-16]
    right_dist = sum(right)/len(right) # average distance of obstacles on the right 
    left = laserscan[16:90]
    left_dist = sum(left)/len(left) # average distance of obstacles on the left 
    err_side = left_dist-right_dist # estimating the error for P-Controller
    
    """ For longitudnal control """
    
    # # front_dist = min(min(i for i in laserscan[(len(laserscan)-15):len(laserscan)] if i>0), min(i for i in laserscan[0:15] if i>0)) # front distance 
    # if front_dist == None:
    #     front_dist = 10

    front_dist = []
    front_right = laserscan[-20:]
    front_left = laserscan[0:20]
    if(len(front_left)>0 and len(front_right)>0):
        for i in range(0,len(front_left)):
            if front_left[i] != 0:
                front_dist.append(front_left[i])
        for i in range(0,len(front_right)):
            if front_right[i] != 0:
                front_dist.append(front_right[i])

        front_dist = min(front_dist)
    
    else: 
        front_dist=10

    err_front = front_dist-0.2 # setting desired distance to be 0.2 for sim -- 0.35 for real-world
    kp_side = 1
    kp_front = 0.5

    """ Desired angular and linear velocity """
    move.angular.z = np.clip(err_side*kp_side,-1.2, 1.2)
    move.linear.x   = np.clip(err_front*kp_front,-0.1,0.2) # max linear vel to 0.3 for sim -- 0.4 for real-world 

def obstacle_avoidance():

    if move.linear.x < 0.01 and move.linear.x > 0:
        move.angular.z = 0.3
    vel_pub.publish(move)

def line_following():
    global auto_nav_object
    auto_nav_object = LineFollower()

    ctrl_c = False
    def shutdownhook():
        auto_nav_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)

if __name__ == '__main__':
    rospy.init_node('autonomous_navigation', anonymous=True)
    vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
    laser_sub = rospy.Subscriber("/scan",LaserScan, obstacle_avoidance_callback)
    move = Twist()
    global rate
    rate = rospy.Rate(5)
    
    if flag == 0:
        line_following()

    while mode==0:
        obstacle_avoidance()
        rate.sleep()
    rospy.spin()