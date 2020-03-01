#!/usr/bin/env python

'''
* Team Id:          2106
* Author:           Shweta Dalal, Laxmikant Suryavanshi
* Filename:         roi_detector.py
* Theme:            Survey and Rescue
* Functions:        detect_color_contour_centers
* Global Variables: None
'''

from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import SRInfo
from cv_bridge import CvBridge, CvBridgeError
import random
import json
import imutils
import copy
from time import sleep

class sr_determine_colors():

	def __init__(self):

		self.bridge = CvBridge()
		self.detect_pub = rospy.Publisher("/detection_info", SRInfo, queue_size=10)
 		self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, self.image_callback)
 		self.serviced_sub = rospy.Subscriber('/serviced_info', SRInfo, self.serviced_callback)
		self.img = None
		# event_msg: Detected event message to publish
		self.event_msg = SRInfo()
		# labelled_boxes: ROIs dictionary load from a json file
		self.labelled_boxes = dict()
		# PUBLISHED: If event detected the corresponding location is pushed into this dictionary
		# Hence next time detection isn't processed at this location
 		self.PUBLISHED = dict()


	'''
	* Function Name: load_rois
	* Input:         file_path path of a json file to load
	* Output:        None
	* Logic:         Use json.load method to read rect_info,json file of RoIs
	* Example:       obj.load_rois('file.json')
	'''
	def load_rois(self, file_path = '/home/yuks/eyrc_ws/src/survey_and_rescue/scripts/rect_info.json'):
		try:
			with open(file_path, 'rb') as input:
				self.labelled_boxes = json.load(input)
		except:
			print("File doesn't exist or is corrupted")


 	def image_callback(self, data):
 		try:
 			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
 		except CvBridgeError as e:
 			print(e)

	'''
	* Function Name: serviced_callback
	* call_back function for Subscriber to serviced_info
	'''
 	def serviced_callback(self, msg):
		# wait for sometime after msg recieved to avoid false detection
		# since it takes few time to turn off LED and then pop the location
		# from PUBLISHED dictionary so that it can be processed again
		sleep(.5)
		self.PUBLISHED.pop(msg.location, None)

	'''
	* Function Name: detect_color_contour_centers
	* Input:         None
	* Output:        None
	* Logic: ROIs of image recieved from /usb_cam/image_rect_color are
					 converted to HSV. Then this ROI cells are compared with
					 the HSV 'range' of colors to be detected, here, 'RED', 'GREEN',
					 and 'BLUE'. cv2.inRange function masks the ROIs pixels found
					 within that range. Pixels in this masked image are summed up
					 to check whether it crosses certain threshold.
					 If yes then respective color is detected.
	* Example Call: obj.detect_color_contour_centers()
	'''
	def detect_color_contour_centers(self):
		# image: create a deep copy of self.img
		image = self.img.copy()
		
		# loop over the ROI cells
		for coord in self.labelled_boxes:

			# if a beacon is already detected and published for this location
			# skip the processing that location again and proceed
			if coord in self.PUBLISHED:
				continue

			else:
				# color detection code
				# unpack ROI boundingRect's value
				x, y, w, h = self.labelled_boxes[coord]
				# ROI is specific portion of an image
				roi_cell = image[y:y+h, x:x+w]
				# convert cell from RGB to HSV
				roi_cell_hsv = cv2.cvtColor(roi_cell, cv2.COLOR_BGR2HSV)
				# kernel: used to dilate binary image obtained from cv2.inRange
				kernel = np.ones((5, 5), "uint8")

				# lower_red: lower bound of red color in HSV color space
				lower_red = np.array([136, 87, 150], np.uint8)
				# upper_red: upper bound of red color in HSV color space
				upper_red = np.array([200, 255, 200], np.uint8)

				# convert roi to binary by applying basic thresholding operation
				detect_color_red = cv2.inRange(roi_cell_hsv, lower_red, upper_red)
				# apply morphological enhancement operation on image
				detect_color_red = cv2.dilate(detect_color_red, kernel)

				# if sum of pixel values in binary image is greater than certain
				# threshold, respective color is considered to be present in cell
				if np.sum(detect_color_red) > 10000:
					self.event_msg.location = coord
					self.event_msg.info = "RESCUE"
					self.detect_pub.publish(self.event_msg)
					self.PUBLISHED[coord] = "RESCUE"
					continue

				# lower_green: lower bound of green color in HSV color space
				lower_green = np.array([51, 140, 55], np.uint8)
				# upper_green: upper bound of green color in HSV color space
				upper_green = np.array([70, 255, 198], np.uint8)

				# convert roi to binary by applying basic thresholding operation
				detect_color_green = cv2.inRange(roi_cell_hsv, lower_green, upper_green)
				# apply morphological enhancement operation on image
				detect_color_green = cv2.dilate(detect_color_green, kernel)

				# if sum of pixel values in binary image is greater than certain
				# threshold, respective color is considered to be present in cell
				if np.sum(detect_color_green) > 10000:
					self.event_msg.location = coord
					self.event_msg.info = "FOOD"
					self.detect_pub.publish(self.event_msg)
					self.PUBLISHED[coord] = "FOOD"
					continue

				# lower_blue: lower bound of blue color in HSV color space
				lower_blue = np.array([100, 184, 55], np.uint8)
				# upper_blue: upper bound of blue color in HSV color space
				upper_blue = np.array([255, 255, 100], np.uint8)

				# convert roi to binary by applying basic thresholding operation
				detect_color_blue = cv2.inRange(roi_cell_hsv, lower_blue, upper_blue)
				# apply morphological enhancement operation on image
				detect_color_blue = cv2.dilate(detect_color_blue, kernel)

				# if sum of pixel values in binary image is greater than certain
				# threshold, respective color is considered to be present in cell
				if np.sum(detect_color_blue) > 18000:
					self.event_msg.location = coord
					self.event_msg.info = "MEDICINE"
					self.detect_pub.publish(self.event_msg)
					self.PUBLISHED[coord] = "MEDICINE"
					continue



def main(args):
	
	try:
		rospy.init_node('sr_beacon_detector', anonymous=False)
		s = sr_determine_colors()
		rate = rospy.Rate(30)
		s.load_rois()
		while s.img is None:
			pass
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
	
	while not rospy.is_shutdown():
		try:
			s.detect_color_contour_centers()
			rate.sleep()
		except KeyboardInterrupt:
			cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)