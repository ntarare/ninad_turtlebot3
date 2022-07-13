#!/usr/bin/env python3
	
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
	
def PID_side(err_side, Kp_side=2.5): #defing a function for P-Controller
    return Kp_side*err_side 
    	
def PID_front(err_front, Kp_front=1): #defing a function for P-Controller
    return Kp_front*err_front 

def wallfollow(data): # defining a wall following function
    
    data = list(data.ranges[0:360]) # storing LiDAR data 

    """ For lateral control """
   
    right = data[-90:-16]
    right_dist = sum(right)/len(right) # average distance of obstacles on the right 

    left = data[16:90]
    left_dist = sum(left)/len(left) # average distance of obstacles on the left 

    err_side = left_dist-right_dist # estimating the error for P-Controller
	
    """ For longitudnal control """

    front_dist = min(max(data[0:15], data[(len(data)-15):len(data)])) # front distance 

    err_front = front_dist-0.35 # setting desired distance to be 0.2 for sim -- 0.35 for real-world
 
    """ Desired angular and linear velocity """
   
    move.angular.z = np.clip(PID_side(err_side), -1.5,1.5)
    move.linear.x   = np.clip(PID_front(err_front), -0.1, 0.4) # max linear vel to 0.3 for sim -- 0.4 for real-world 



    rospy.loginfo('')
    print("Current angular Velocity is: %s" % move.angular.z)
    print("Current linear Velocity is: %s" % move.linear.x)
    print("Distance from front wall in centimeters is: %s" % (front_dist*100))
    print("Distance from right wall in centimeters is: %s" % (right_dist*100))
    print("Distance from left wall in centimeters is: %s" % (left_dist*100))
    print('\n')

rospy.init_node('wall_follower')
move = Twist()
pub = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
sub = rospy.Subscriber("/scan",LaserScan, wallfollow)
while not rospy.is_shutdown():
    pub.publish(move)
    pass
rospy.spin()