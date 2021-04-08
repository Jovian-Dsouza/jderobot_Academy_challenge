#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import math
from math import pi
from drone_wrapper import DroneWrapper
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose
from math import atan2, sqrt

code_live_flag = False

def gui_play_stop_cb(msg):
	global code_live_flag, code_live_timer
	if msg.data == True:
		if not code_live_flag:
			code_live_flag = True
			code_live_timer = rospy.Timer(rospy.Duration(nsecs=50000000), execute)
	else:
		if code_live_flag:
			code_live_flag = False
			code_live_timer.shutdown()

def set_image_filtered(img):
	gui_filtered_img_pub.publish(drone.bridge.cv2_to_imgmsg(img))

def set_image_threshed(img):
	gui_threshed_img_pub.publish(drone.bridge.cv2_to_imgmsg(img))

################# Build your position control function here #####################
	
def position_control(x, y, z):
	global drone

	# Set Drone angle
	cur_x, cur_y, cur_z = drone.get_position()
	cur_x, cur_y = cur_y , -cur_x

	des_a = atan2(cur_y - y, cur_x - x) -pi 
	drone.set_cmd_pos(cur_x, cur_y, cur_z, des_a)
	# print("Drone Angle Set %0.3f" % des_a)
	rospy.sleep(1)

	while sqrt((cur_x-x)**2 + (cur_y-y)**2) > 0.4:
		drone.set_cmd_pos(x, y, z, des_a)
		cur_x, cur_y, cur_z = drone.get_position()
		cur_x, cur_y = cur_y , -cur_x
		# print("Distance %0.3f x=%0.3f y=%0.3f dx=%0.3f dy=%0.3f" % (sqrt((cur_x-x)**2 + (cur_y-y)**2), cur_x, cur_y, x, y) )
		rospy.sleep(0.1)

	rospy.loginfo("Waypoint Reached")
	
#################################################################################

def execute(event):
	global drone
	
	################# Insert your code here #################################
	# Waypoint list
	waypoint_list = [(8, -8, 2 ), 
					(-8, -8, 1.5),
					(-8, 8, 3.5),
					(-3, 8, 3.3),
					(1.5, 5.7, 2.5),
					(3.5 ,8, 2),
					(8, 8, 2)]

	for x, y, z in waypoint_list:
	    position_control(x, y, z)

	#########################################################################

if __name__ == "__main__":
	drone = DroneWrapper()
	
	drone.takeoff(4)
	waypoint_list = [(8, -8, 2 ), 
					(-8, -8, 1.5),
					(-8, 8, 3.5),
					(-3, 8, 3.3),
					(1.5, 5.7, 2.5),
					(3.5 ,8, 2),
					(8, 8, 2)]

	for x, y, z in waypoint_list:
	    position_control(x, y, z)
	
	position_control(0, 0, 2)
	drone.land()
	rospy.loginfo("Misson Complete")

	rospy.Subscriber('gui/play_stop', Bool, gui_play_stop_cb)
	gui_filtered_img_pub = rospy.Publisher('interface/filtered_img', Image, queue_size = 1)
	gui_threshed_img_pub = rospy.Publisher('interface/threshed_img', Image, queue_size = 1)
	code_live_flag = False
	code_live_timer = rospy.Timer(rospy.Duration(nsecs=50000000), execute)
	code_live_timer.shutdown()
	while not rospy.is_shutdown():
		rospy.spin()
