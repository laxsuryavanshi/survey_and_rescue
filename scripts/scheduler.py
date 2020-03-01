#!/usr/bin/env python

'''
* Team Id: 2106
* Author:  Laxmikant Suryavanshi, Shweta Dalal
* Filename: scheduler.py
* Theme: Survey and Rescue
* Functions: distance, position, prior_food, prior_med,
             prior_rescue, decided_service, go_to_base,
             getBase, ISR
* Global Variables: None
'''

from __future__ import print_function
import rospy
from survey_and_rescue.msg import SRInfo
from geometry_msgs.msg import Point
import time
from std_msgs.msg import Float64
import numpy as np
from survey_and_rescue.msg import SRDroneStats
import csv

class sr_scheduler():

  def __init__(self):

    rospy.init_node('sr_scheduler', anonymous=False)

    # The scheduler gets information about the type of Beacon and location of
    # Beacon from the beacon_detector node by subscribing to the topic
    # '/detection_info'
    rospy.Subscriber('/detection_info', SRInfo, self.detection_callback)

    # subcribe to serviced_info published by monitor.pyc
    # @NOTE Use this info to avoid running seperate timer to check
    # hover time of the drone
    rospy.Subscriber('/serviced_info', SRInfo, self.serviced_callback)

    # subcribe to hover_time published by position_hold
    # time of drone service
    # plays key role in making decision servicing of rescue
    rospy.Subscriber('/hover_time', Float64, self.timer_callback)

    # subscribe to stats_sr published by monitor.pyc
    # to retrieve useful info such as foodOnboard and medOnboard
    rospy.Subscriber('/stats_sr', SRDroneStats, self.stats_sr_callback)

    # The scheduler publishes its decision on the topic '/decision_info'
    # publish: location and info
    # @TODO Use this topic to manipulate setpoint in pid script
    self.decision_pub = rospy.Publisher('/decision_info', SRInfo, queue_size=10)

    # decided_msg: event to be service
    self.decided_msg = SRInfo()

    # [rescue/food/med]_requests: event detected are stored in a dictionary
    # key = location of service
    # value = time of arrival
    self.rescue_requests = dict()
    self.food_requests = dict()
    self.med_requests = dict()

    # foodOnboard: store food supply available on drone
    self.foodOnboard = 3
    # medOnboard: store medicine supply available on drone
    self.medOnboard = 3
    # start_time: stores starting time of task
    self.start_time = time.time()
    # serviced_time: stores time of ongoing service
    self.serviced_time = Float64()
    # rescueOngoing: flag is set when a rescue service is ongoing
    self.rescueOngoing = False
    # serviceOngoing: flag is set when a food or medicine service is ongoing
    self.serviceOngoing = False
    # base: stores location of base. read from LED_OrgConfig.tsv
    self.base = None
    # flagContinueService: if true, continues the ongoing service even
    # if a rescue event is detected
    self.flagContinueService = False
    # rescueSuccess: flag is set when drone hovers successfully for 5 sec
    # on rescue location
    self.rescueSuccess = False
    # rate: controls the rate of loops in class methods
    self.rate = rospy.Rate(30)
    # wait for a while to let the node registered with ros server
    time.sleep(1)

    self.getBase()
    self.go_to_base()

  '''
  * Function Name: detection_callback
  * Input: `data` recieved from '/detection_info' subscriber
  * Output: None
  * Logic: Stores the arrival time of detected event in corresponding dictionary
  * Example Call: called automatically by ros when data recieved on topic
  '''
  def detection_callback(self, data):
    # Store the time of detected event in a corresponding dictionary
    print('Event detected at ', data.location, ': ', data.info)
    if data.info == "RESCUE":
      self.rescue_requests[data.location] = time.time()
    elif data.info == "FOOD":
      self.food_requests[data.location] = time.time()
    elif data.info == "MEDICINE":
      self.med_requests[data.location] = time.time()

  '''
  * Function Name: timer_callback
  * Input: `data` recieved from '/hover_time' subscriber
  * Output: None
  * Logic: Stores the serviced time of ongoing event
  * Example Call: called automatically by ros when data recieved on topic
  '''
  def timer_callback(self, data):
    self.serviced_time = data

  '''
  * Function Name: serviced_callback
  * Input: `data` recieved from '/serviced_info' subscriber
  * Output: None
  * Logic: if data is recieved on certain event, that means the event is
           either serviced successfully or not available to service anymore.
           Hence the event is removed from corresponding event's list and hence
           not considered for servicing on next decision.
           Also if data recieved is about current servicing event, the flags
           serviceOngoing and flagContinueService are cleared
  * Example Call: called automatically by ros when data recieved on topic
  '''
  def serviced_callback(self, data):
    # STATUS recieved on current servicing event
    print('Status recieved for ', data.location, ': ', data.info)
    if data.location == self.decided_msg.location:
      # clear the flags
      self.serviceOngoing = False
      self.flagContinueService = False
      if self.rescueOngoing == True:
        if data.info == 'SUCCESS':
          self.rescueSuccess = True
        else:
          self.rescueOngoing = False
    if data.location == self.base:
      self.rescueOngoing = False

    # REMOVE from dictionary
    if data.location in self.med_requests:
      # MEDICINE event status
      self.med_requests.pop(data.location, None)
    elif data.location in self.food_requests:
      # FOOD event status
      self.food_requests.pop(data.location, None)
    elif data.location in self.rescue_requests:
      # RESCUE event status
      self.rescue_requests.pop(data.location)

  '''
  * Function Name: stats_sr_callback
  * Input: `data` recieved from '/stats_sr' subscriber
  * Output: None
  * Logic: fetch info on foodOnboard and medOnboard
  * Example Call: called automatically by ros when data recieved on topic
  '''
  def stats_sr_callback(self, data):
    # UPDATE foodOnboard and medOnboard with latest values
    self.foodOnboard = data.foodOnboard
    self.medOnboard = data.medOnboard

  '''
  * Function Name: distance
  * Input: `location` of cell to find distance from current position of drone
  * Output: Chessboard or D8 distance of cell from current `position of drone`
  * Logic: Chessboard distance is given as:
           distance = max(|x1 - x2|, |y1 - y2|)
           where the locations are a string ('A1'), hence splitted and converted to
           integer to compute distance
           say: location1 = 'C3', location2 = 'E3'
           int(location1[1]) = 3 and int(location2[1]) = 3
           ord(location1[0]) = 67 and ord(location2[0]) = 69
           hence, distance = max(|3 - 3|, |67 - 69|) = `2`
  * Example Call: self.distance('F3')

  @NOTE nearer services are given high priority
  '''
  def distance(self, location):
    # RETURN chessboard or d8 distance from current position
    return max(abs(int(location[1]) - int(self.decided_msg.location[1])),
              abs(ord(location[0]) - ord(self.decided_msg.location[0])))

  '''
  * Function Name: position
  * Input: `location` of cell to find distance from `centre`
  * Output: Chessboard or D8 distance of cell from `centre` of arena
  * Logic: Chessboard distance is given as:
           distance = max(|x1 - x2|, |y1 - y2|)
           the centre of 6x6 grid is considered as (3.5, 3.5)
           hence the passed location is first converted to 2D point as:
           say, location = 'B2'
           x = ord(location[0]) - 64 ==> [ord('A') - 1] i.e. 'A' is given as 1
                                            ^
                                            65
           y = int(location[1])
           position = int(max(|3.5 - x|, |3.5 - y|) + .5)
           (.5 is added to round off the distance to `ceil` value)
  * Example Call: self.location('E2')

  @NOTE events nearer to centres are prioritized
  '''
  def position(self, location):
    # RETURN chessboard or d8 distance from centre
    return int(max(abs(int(location[1]) - 3.5), abs(ord(location[0]) - 3.5 - 64)) + .5)

  '''
  * Function Name: prior_food
  * Input: None
  * Output: highest priority food request in `food_requests` list
  * Logic: Factors considered while determinig priority are:
           1. Time of Arrival (ToA)
           2. distance from current position of drone
           3. position of event from centre
           Each factor is given some weightage (Wx) based on importance of factor
           Hence priority is given as
           (ToA * Wt + distance * Wd + position * Wp)
           The event is discarded if not serviced for certain amount of time (here 25 sec)
  * Example Call: self.prior_food()
  
  @NOTE priority is given to
  1. `least` recent service
  2. service nearer to drone's current postion
  3. service nearer from centre of arena
  (since fluctuations in reading are less in this zone)
  '''
  def prior_food(self):
    max_prior_service = None
    max_prior_value = -1
    for loc in self.food_requests.keys():
      ToA = time.time() - self.food_requests[loc]
      if ToA > 25.:
        # @TODO when there are no other events
        del self.food_requests[loc]
        continue
      position = 4 - self.position(loc)
      distance = 6 - self.distance(loc)
      priority = (ToA * .4 + position * 2.666667 + distance)
      if max_prior_service == None:
        max_prior_service = loc
        max_prior_value = priority
        continue
      else:
        if priority > max_prior_value:
          max_prior_value = priority
          max_prior_service = loc
    return max_prior_service, max_prior_value

  '''
  * Function Name: prior_food
  * Input: None
  * Output: highest priority medicine request in `med_requests` list
  * Logic: Factors considered while determinig priority are:
           1. Time of Arrival (ToA)
           2. distance from current position of drone
           3. position of event from centre
           Each factor is given some weightage (Wx) based on importance of factor
           Hence priority is given as
           (ToA * Wt + distance * Wd + position * Wp)
           The event is discarded if not serviced for certain amount of time (here 25 sec)
  * Example Call: self.prior_med()
  @NOTE priority is given to
  1. `least` recent service
  2. service nearer to drone's current postion
  3. service nearer from centre of arena
  (since fluctuations in reading are less in this zone)
  '''
  def prior_med(self):
    max_prior_service = None
    max_prior_value = -1
    for loc in self.med_requests.keys():
      ToA = time.time() - self.med_requests[loc]
      if ToA > 25.:
        # @TODO when there are no other events
        del self.med_requests[loc]
        continue
      position = 4 - self.position(loc)
      distance = 6 - self.distance(loc)
      priority = (ToA * .4 + position * 2.666667 + distance)
      if max_prior_service == None:
        max_prior_service = loc
        max_prior_value = priority
        continue
      else:
        if priority > max_prior_value:
          max_prior_value = priority
          max_prior_service = loc
    return max_prior_service, max_prior_value

  '''
  * Function Name: prior_rescue
  * Input: None
  * Output: highest priority rescue request if recieved multiple
  * Logic: Factors considered while dealing with multiple `rescue_requests`
           1. Time of Arrival (ToA)
           2. distance from current position
           3. position from centre
           Each factor is given some weightage (Wx) based on importance of factor
           Hence priority is given as
           (ToA * Wt + distance * Wd + position * Wp)
           The event is discarded if not serviced for certain amount of time (here 4 sec)
  * Example Call: self.prior_med()
  @NOTE priority is given to
  1. `most` recent service
  2. service nearer to drone's current postion
  3. service nearer from centre of arena
  (since fluctuations in reading are less in this zone)
  '''
  def prior_rescue(self):
    max_prior_rescue = None
    max_prior_value = -1
    for loc in self.rescue_requests.keys():
      ToA = time.time() - self.rescue_requests[loc]
      if ToA > 4.2:
        del self.rescue_requests[loc]
        continue
      time_priority = 5 - ToA
      distance = 6 - self.distance(loc)
      position = 4 - self.position(loc)
      priority = time_priority + (distance * 1.1) + position
      if max_prior_rescue == None:
        max_prior_rescue = loc
        max_prior_value = priority
        continue
      else:
        if priority > max_prior_value:
          max_prior_value = priority
          max_prior_rescue = loc
    return max_prior_rescue

  def ISR(self):
    pass

  '''
  * Function Name: getBase
  * Input: None
  * Output: None
  * Logic: read the tsv file and assign location of base to self.base
  * Example Call: self.getBase()
  '''
  def getBase(self):
    with open("/home/yuks/eyrc_ws/src/survey_and_rescue/scripts/LED_OrgConfig.tsv", 'r') as tsv:
      configFile = csv.reader(tsv, delimiter='\t')
      for line in configFile:
        if 'BASE' in line:
          self.base = line[0]
    print('BASE at: ', self.base)

  '''
  * Function Name: go_to_base
  * Input: None
  * Output: None
  * Logic: published base location on topic '/decision_info'
  * Example Call: self.go_to_base()
  '''
  def go_to_base(self):
    self.decided_msg.info = "BASE"
    self.decided_msg.location = self.base
    self.decision_pub.publish(self.decided_msg)
    print('Approaching BASE')

  '''
  * Function Name: decided_service
  * Input: None
  * Output: None
  * Logic: Various factors are taken into consideration before making a decision
           If there are no service ongoing, BASE location is publish to replenish supply
           If no rescue request is there, find the highest priority food and medicine
           request. compare the priority of either request and decide to serve the
           highest priority food or medicine request.
           If a rescue request is recieved, check the time of ongoing service,
           distance of rescue location and time of arrival of service.
           If time required to finish the ongoing task and time required to
           reach rescue source is less than certain threshold time (here 2 sec),
           then ongoing event serviced and then rescue event is serviced.
           Otherwise, ongoing service is aborted to serve rescue the rescue event.
  * Example Call: self.decided_service()
  '''
  def decided_service(self):
    if len(self.rescue_requests) == 0 and \
       len(self.med_requests) == 0 and \
       len(self.food_requests) == 0:
      # print('No requests queued')
      try:
        if self.medOnboard < 3 or self.foodOnboard < 3:
          # replenish supply
          print('Replenishing supplies')
          self.go_to_base()
      except:
        # medOnboard and foodOnboard is None
        pass

    else:
      if len(self.rescue_requests) != 0 and not self.rescueOngoing:
        # INTERRUPT arrived
        print('Interrupt Recieved')
        prior_rescue = self.prior_rescue()
        # If there is no rescue event that can be `served`
        # i.e. rescue event in queue for more than 4.2 seconds
        if not prior_rescue:
          return False
        # ToA: current time minus time of arrival (example 3 seconds before)
        ToA = time.time() - self.rescue_requests[prior_rescue]
        # distance: distance of location from current position
        distance = self.distance(prior_rescue)
        # hover_time: time remain to complete ongoing service
        hover_time = (3 - self.serviced_time.data) if self.serviced_time.data < 3 else 0
        if (ToA + distance + hover_time) < 1.5 and self.decided_msg.info != "BASE":
          # Continue with service
          print('Ongoing Service Continued')
          self.flagContinueService = True
          # flag is cleared in serviced_callback function
          while self.flagContinueService:
            self.rate.sleep()
          # ready to serve rescue event
          self.decided_msg.location = prior_rescue
          self.decided_msg.info = "RESCUE"
          self.decision_pub.publish(self.decided_msg)
          self.serviceOngoing = True
          self.rescueOngoing = True
          while self.serviceOngoing:
            self.rate.sleep()
          # if drone hover successfully on rescue location
          if self.rescueSuccess:
            print('RESCUE successful')
            self.go_to_base()
            self.rescueSuccess = False
        else:
          # Abort service
          print('Aborting ongoing service')
          self.decided_msg.location = prior_rescue
          self.decided_msg.info = "RESCUE"
          self.decision_pub.publish(self.decided_msg)
          self.rescueOngoing = True
          self.serviceOngoing = True
          while self.serviceOngoing:
            self.rate.sleep()
          if self.rescueSuccess:
            print('RESCUE successful')
            self.go_to_base()
            self.rescueSuccess = False
      # no service or rescue is ongoing
      elif not self.serviceOngoing and not self.rescueOngoing and len(self.rescue_requests) == 0:
        # check for available supplies
        time.sleep(.2)
        if self.foodOnboard > 0 and self.medOnboard > 0:
          # find high priority food and med requests
          prior_food, prior_valF = self.prior_food()
          prior_med, prior_valM = self.prior_med()

          if prior_food != None and prior_med != None:
            # compare priorities
            if prior_valF > prior_valM:
              # Service food
              self.decided_msg.info = "FOOD"
              self.decided_msg.location = prior_food
              self.decision_pub.publish(self.decided_msg)
              self.serviceOngoing = True
            else:
              # Service med
              self.decided_msg.info = "MEDICINE"
              self.decided_msg.location = prior_med
              self.decision_pub.publish(self.decided_msg)
              self.serviceOngoing = True
          # No med requests
          elif prior_food:
            # Service food
            self.decided_msg.info = "FOOD"
            self.decided_msg.location = prior_food
            self.decision_pub.publish(self.decided_msg)
            self.serviceOngoing = True
          # No food requests
          elif prior_med:
            # Service med
            self.decided_msg.info = "MEDICINE"
            self.decided_msg.location =prior_med
            self.decision_pub.publish(self.decided_msg)
            self.serviceOngoing = True
        # No Meds available. FOOD only
        elif self.foodOnboard > 0:
          prior_food, prior_valF = self.prior_food()
          if prior_food != None:
            # Service food
            self.decided_msg.info = "FOOD"
            self.decided_msg.location = prior_food
            self.decision_pub.publish(self.decided_msg)
            self.serviceOngoing = True
        # No food available. MEDICINE only
        elif self.medOnboard > 0:
          prior_med, prior_valM = self.prior_med()
          if prior_med != None:
            # Service med
            self.decided_msg.info = "MEDICINE"
            self.decided_msg.location = prior_med
            self.decision_pub.publish(self.decided_msg)
            self.serviceOngoing = True
        elif self.medOnboard == 0 and self.foodOnboard == 0:
          self.go_to_base()
  
  def shutdown_hook(self):
    pass


if __name__ == "__main__":
  try:
    scheduler = sr_scheduler()
    r = rospy.Rate(30)
  except Exception as error:
    print(error)
  while not rospy.is_shutdown():
    scheduler.decided_service()
    r.sleep()