#!/usr/bin/env python

# Import necessary libraries: ROS, OpenCV and ROS-OpenCV Bridge

import cv2
import rospy
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

# Import communication datatypes

from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32

# Define important variables and constructs

bridge = CvBridge()
global cameraFrame
cameraFrame = None

global IMG_ROWS
IMG_ROWS = 480
global IMG_COLS
IMG_COLS = 640

global obj_loc
obj_loc = 'center_center'
global msg_cnt
msg_cnt = 0

# Function to detect circular object and its center (Hough Transform)

def findObjCenter(cv_img):

	# Convert to grayscale and apply slight Gaussian Blur

	gray_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2GRAY)
	gray_blur_img = cv2.GaussianBlur(gray_img, (5, 5), 0)

	# Hough Transform to detect circles

	circles = cv2.HoughCircles(image = gray_blur_img, 
							   method = cv2.HOUGH_GRADIENT, 
							   dp = 1, 
							   minDist = 15, 
							   param1 = 50, 
							   param2 = 30, 
							   minRadius = 0, 
							   maxRadius = 0)
	
	# Select first circle, draw on image and compute center

	if circles is not None:
		im = circles[0, 0]
		#cv2.circle(cv_img, (im[0], im[1]), im[2], (0, 255, 0), 2)
		#cv2.circle(cv_img, (im[0], im[1]), 2, (255, 0, 0), 3)
		c_col = im[0]; c_row = im[1]
	else:
		c_col = None; c_row = None

	center_dict = {'x' : c_col, 'y' : c_row}
	return center_dict, cv_img

# Function mapping the object center from pixel to (3 X 3) grid

def findGridLoc(center):

	global IMG_ROWS, IMG_COLS

	# Thresholds demarcating the (3 X 3) grid

	THRESH_X = IMG_COLS/3.0
	THRESH_Y = IMG_ROWS/3.0

	# Map center pixel to grid string

	if center['x'] == None or center['y'] == None:
		horz_tag = 'center'
		vert_tag = 'center'
	else:
		if center['y'] < THRESH_Y:
			vert_tag = 'up'
		elif center['y'] < 2*THRESH_Y:
			vert_tag = 'center'
		else:
			vert_tag = 'down'

		if center['x'] < THRESH_X:
			horz_tag = 'left'
		elif center['x'] < 2*THRESH_X:
			horz_tag = 'center'
		else:
			horz_tag = 'right'

	obj_loc = vert_tag + '_' + horz_tag
	return obj_loc

# Function to facilitate Arduino communication

def convertToInt(string):

	tag1, tag2 = string.split('_')

	if tag1 == 'up':
		num1 = 1
	elif tag1 == 'center':
		num1 = 2
	else:
		num1 = 3

	if tag2 == 'left':
		num2 = 1
	elif tag2 == 'center':
		num2 = 2
	else:
		num2 = 3

	return int(str(num1) + str(num2))

# Subscriber Callback function

def callback(ros_image):

	global msg_cnt, obj_loc, cameraFrame

	# Convert ROS Image into OpenCV RGB Image

	cv_image = bridge.imgmsg_to_cv2(ros_image, 'bgr8')
	b, g, r = cv2.split(cv_image)
	cv_image = cv2.merge([r, g, b])
	
	print 'Message ' + str(msg_cnt) + ': Image received'

	# Find the grid box corresponding to object center

	obj_cen, cv_img = findObjCenter(cv_image)
	obj_loc_str = findGridLoc(obj_cen)
	obj_loc = convertToInt(obj_loc_str)
	print obj_loc_str

	# Plot the OpenCV RGB Image with detected circles

	# if cameraFrame is None:
	# 	cameraFrame = plt.imshow(cv_img)
	# else:
	# 	cameraFrame.set_data(cv_img)
	# plt.pause(0.0001)
	# plt.draw()

# Main computer node function

def computer():

	global msg_cnt, obj_loc

	# Define node name and camera topic

	node_name = "computer_node"
	camera_topic = "/usb_cam/image_raw"

	# Start publisher and subscriber node

	rospy.init_node(node_name, anonymous=None)
	pub = rospy.Publisher('object_location', Int32, queue_size=10)
	rospy.Subscriber(camera_topic, Image, callback)

	# Publish to Arduino

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish(obj_loc)
		rate.sleep()
		msg_cnt += 1

if __name__ == '__main__':
	computer()
