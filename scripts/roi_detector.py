#!/usr/bin/env python

'''
* Team Id:          2106
* Author:           Laxmikant Suryavanshi
* Filename:         roi_detector.py
* Theme:            Survey and Rescue
* Functions:        detect_rois, sort_rois, save_rois, show_rois
* Global Variables: None
'''

from __future__ import print_function
import rospy
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import json
import itertools

class roi_detector():

  def __init__(self):

    # cv_bridge: used to convert images to cv's Mat object
    self.cv_bridge = CvBridge()
    # image_sub: subscriber to '/usb_cam/image_rect' for image data to find Region of Interest
    self.image_sub = rospy.Subscriber('/usb_cam/image_rect', Image, self.image_callback)
    # image: used to store image recieved from 'usb_cam'
    self.image = None
    
    # roi_list: save rois in list
    self.roi_list = list()
    # flag to check whether ROIs are sorted or not
    self.roi_sorted = False

  def image_callback(self, data):
    try:
      # try to convert image from usb_cam to cv'format (grayscale image)
      self.image = self.cv_bridge.imgmsg_to_cv2(data, 'mono8')
    except CvBridgeError as error:
      print('[ERROR] ', error)


  '''
  * Function Name: detect_rois
  * Input: `min_area` minimum area of an RoI
           `max_area` maximum area of an RoI
           `lower_thresh` threshold to convert grayscale image to binary image (default: 127)
           `max_val` upper bound of thresholding (default: 255)
  * Output: None
  * Logic: image recieved from usb_cam is stored in a local variable
           Then preprocessing is applied on the image to remove nois
           Resulted image is then thresholded to convert into a binary image
           Then on this binary image, contour detection is applied
           The area of contour is within the range [min_area, max_area],
           then the respective contour is considered as an RoI
  * Example Call: obj.detect_rois(4500, 8000, 127, 255)
  '''
  def detect_rois(self, min_area, max_area, lower_thresh=127, max_val=255):
    # image: create a deep copy of image for processing
    image = self.image.copy()
    # purge all elements in list before appending
    del self.roi_list[:]
    # set roi_sorted flag to False
    self.roi_sorted = False

    # Apply blur for image smoothening
    image = cv.GaussianBlur(image, (5, 5), 0)
    # Thresholding smoothened image to obtain binary image
    _, thresh = cv.threshold(image, lower_thresh, max_val, cv.THRESH_BINARY)
    # Find contours of binary image
    _, contours, __ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
      # @TODO testing behaviour rect: x, y, w, h
      x, y, w, h = cv.boundingRect(cnt)
      box = np.array([(x, y), (x, y+h), (x+w, y+h), (x+w, y)])
      area = cv.contourArea(box)

      # If area of contour is in range [min_area, max_area]
      if area => min_area and area <= max_area:
        # Consider as an ROI
        self.roi_list.append([x, y, w, h])

  '''
  * Function Name: rois_detected
  * Input:         None
  * Output:        True if 36 cell coordinates present in roi_list
  * Logic:         length of list is equal to 36
  * Example Call: obj.roi_detected()
  '''
  def rois_detected(self):
    return len(self.roi_list) == 36

  '''
  * Function Name: sort_rois
  * Input: None
  * Output: None
  * Logic: RoIs stored in a list are sorted by X and Y
           Each RoI is stored in the form of x, y, w, h
           At first RoIs are sorted in X axis using builtin python
           function sorted (key = X axis' coordinate)
           Then grouping RoIs in 6, sorted by Y axis' coordinate
  * Example Call: obj.sort_rois()
  '''
  def sort_rois(self):
    try:
      # sort in x axis
      self.roi_list = sorted(self.roi_list, key=lambda x: x[0])

      # sort in y axis by grouping 6 elements
      for i in range(0, 36, 6):
        self.roi_list[i:i+6] = sorted(self.roi_list[i:i+6], key=lambda x: x[1])

      self.roi_sorted = True
      return True
    except Exception as e:
      print('[ERROR] in sorting ROIs', e, sep='\n')

  '''
  * Function Name: save_rois
  * Input: None
  * Output: None
  * Logic: RoI list is dumped into a json file named 'rect_info.json' after labeling
           them as in task
  * Example Call: obj.save_rois()
  '''
  def save_rois(self):
    if self.roi_sorted:
      # labelled_rois: dictionary to store labelled RoIs temporarily
      labelled_rois = dict()
      # cell_names: itertools, a sequence `A1, A2..., A6, B1, B2..., F6`
      cell_names = itertools.product('ABCDEF', '123456')
      # each RoI in sorted list is labelled as respective cell in task
      for roi in self.roi_list:
        labelled_rois[''.join(next(cell_names))] = roi

      # The dictionary is then dumped into a json format file
      with open('/home/yuks/eyrc_ws/src/survey_and_rescue/scripts/rect_info.json', 'w+') as file:
        json.dump(labelled_rois, file)
      return True
    else:
      print('[ERROR] ROIs not sorted')
      return False

  '''
  * Function Name: show_rois
  * Input: None
  * Output: None
  * Logic: Image is converted to BGR format from GRAYSCALE and then RoIs are drawn on image
           The image is then displayed in a new window
  * Example Call: obj.show_rois()
  '''
  def show_rois(self):
    image = self.image.copy()
    image = cv.cvtColor(image, cv.COLOR_GRAY2BGR)

    for roi in self.roi_list:
      cv.rectangle(image, (roi[0], roi[1]), (roi[0]+roi[2], roi[1]+roi[3]), (0, 255, 0), thickness=2)
    cv.imshow('ROI', image)
    cv.waitKey(0)
    cv.destroyWindow('ROI')

  '''
  * Function Name: query_yes_no
  * Input: `question` query to ask user
           `default` if user enters no input this parameter is considered
  * Output: returns True if input is YES else return False
  * Logic: Question is displayed on the display and user query is taken from input
  * Exaple Call: obj.query_yes_no('What parameter do you want to change?')
  '''
  def query_yes_no(self, question, default=None):
    valid = {"yes": True, "y": True, "ye": True,"no": False, "n": False}

    while True:
      choice = raw_input(question+" [Y/N]:\t").lower()
      if default is not None and choice == '':
        return valid[default]
      elif choice in valid:
        return valid[choice]
      else:
        print('Please respond with yes or no\n')

'''
* Function Name: main
* Input: None
* Output: Returns true if RoIs written successfully else false
* Logic: find RoIs in usb_cam image and queries user about satisfaction or
         parameters to change
* Example Call: main()
'''
def main():
  # rd: create an RoI detector's instance
  rd = roi_detector()
  # min_area: Minimum area of an RoI
  min_area = 4500
  # max_area: Maximum area of an RoI
  max_area = 8000
  # lower_thresh: lower threshold to find binary image
  lower_thresh = 127
  # max_value: maximum value in cv's thresholding operation
  max_value = 255
  r = rospy.Rate(30)
  # wait till first image is recieved
  while rd.image == None:
    r.sleep()
  while True:
    # find RoIs in revieved image
    rd.detect_rois(min_area, max_area, lower_thresh, max_val=max_value)
    # display RoIs
    rd.show_rois()
    # If all RoIs are detected neatly
    if rd.rois_detected():
      # Query user
      satisfied_flag = int(raw_input('Are you satisfied? '))
      if satisfied_flag:
        # sort, label and save RoIs in a json file
        rd.sort_rois()
        rd.save_rois()
        return True
    else:
      param = int(raw_input('What param do you want to change? '))
      if param == 1:
        min_area = int(raw_input('Enter new min_area: '))
      elif param == 2:
        max_area = int(raw_input('Enter new max_area: '))
      elif param == 3:
        lower_thresh = int(raw_input('Enter new lower_thresh: '))
      else:
        max_value = int(raw_input('Enter new max value'))
      continue
  return False


if __name__ == "__main__":
  main()