#!/usr/bin/env python

'''
* Team Id:          2106
* Author:           Shweta Dalal, Yukta Behare,
                    Laxmikant Suryavanshi, Shruti Kulkarni
* Filename:         position_hold.py
* Theme:            Survey and Rescue
* Functions:        arm, disarm, roll_pid, pitch_pid, alt_pid, pid
* Global Variables: None
'''

from edrone_client.msg import edrone_msgs
from geometry_msgs.msg import PoseArray
from survey_and_rescue.msg import SRInfo
from std_msgs.msg import Float64
import rospy
import time
import json


class Edrone():

    def __init__(self):

        # initializing ros node with name drone_control
        rospy.init_node('drone_control', anonymous=True)

        # drone_position: Drone's current position in WhyCon frame
        # Initially set to 0.s
        self.drone_position = [0.0, 0.0, 0.0]

        # setpoint: drone should hover at this position
        self.setpoint = [None] * 3

        # cum_timer: cummulative timer when drone is within threshold
        self.cum_timer = Float64()
        # Declaring a cmd of message type edrone_msgs and initializing values
        # cmd: drone movement control command
        self.cmd = edrone_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        # Kp/Kd/Ki: PID constants
        self.Kp = [32.5, 35., 82.8]
        self.Kd = [898.7, 880.7, 1090.7]
        self.Ki = [.073, .073, .085]

        # prev_errors: used to store error value after computation of PID so it 
        # can be used to calculate PID for next time
        self.prev_errors = [0., 0., 0.]

        # min_values: minimum value of drone command in roll, pitch and altitude
        self.min_values = [1300, 1300, 1200]

        # max_values: maximum value of drone command in roll, pitch and altitude
        self.max_values = [1700, 1700, 2000]

        # Iterm: variable used to store Integral term in PID
        self.Iterm = [0., 0., 0.]

        # cell_coords: stores the coordinates of cell in dictinary format
        # key: cell name, value: coordinates
        # {'A1': [5.67, 1.23, -20.12]...}
        self.cell_coords = None

        # command_pub: Publishes drone movement control command
        self.command_pub = rospy.Publisher(
            '/drone_command', edrone_msgs, queue_size=1)
        
        # hover_time_pub: Publishes cummulative timer so scheduler can
        # make use to decide event servicing
        self.hover_time_pub = rospy.Publisher(
            '/hover_time', Float64, queue_size=10)
        
        # [roll/pitch/alt]_pub: publish error in roll, pitch and altitude
        self.roll_pub = rospy.Publisher(
            '/roll_error', Float64, queue_size=10)
        self.pitch_pub = rospy.Publisher(
            '/pitch_error', Float64, queue_size=10)
        self.alt_pub = rospy.Publisher(
            '/alt_error', Float64, queue_size=10)

        # subscribe to 'whycon/poses' to get the current position of drone
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
        
        # setpoint_info published by scheduler script
        # drone's setpoint is updated by this subscriber
        rospy.Subscriber('/decision_info', SRInfo, self.decision_callback)

        self.arm()  # ARMING THE DRONE

    '''
    * Function Name: decision_callback
    * Input: $data message published on topic '/decision_info'
    * Output: None
    * Logic: coordinates of location recieved in data is stored in new setpoint
    '''
    def decision_callback(self, data):
        # Reset cummulative timer for next setpoint
        self.cum_timer = 0
        self.setpoint[0] = self.cell_coords[data.location][0]
        self.setpoint[1] = self.cell_coords[data.location][1]
        self.setpoint[2] = self.cell_coords[data.location][2]
        # Integral term is zeroed out for new setpoint
        self.Iterm = [0., 0., 0.]
        print(self.setpoint)

    '''
    * Function Name: load_coords
    * Input: None
    * Output: None
    * Logic: loads coordinates of cells from cell_coords.json file
    '''
    def load_coords(self, file_path = '/home/yuks/eyrc_ws/src/survey_and_rescue/scripts/cell_coords.json'):
        try:
            with open(file_path, 'r') as file:
                self.cell_coords = json.load(file)
        except:
            print('file path does not exist')


    # Disarming condition of the drone
    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    # Arming condition of the drone : Best practise is to disarm and then arm the drone.
    def arm(self):
        # Disarm before arming the drone
        self.disarm()

        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)  # Publishing /drone_command
        rospy.sleep(1)

    # get the current position of the drone
    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    '''
    * Function Name: [roll/pitch/alt]_pid
    * Input:         None
    * Output:        $output calculated response from PID
    * Logic: Compute the response of a drone for certain error in position
             using PID algorithm. This response is then used to manipulate
             the control to settle the drone at specific point.
             error = position - setpoint
             P = Kp * error
             D = Kd * (error - previous_error)
             I = Ki * (error + previous_error) + I
    '''
    # [roll/pitch/alt]_pid functions are used to compute PID in
    # roll, pitch and altitude direction
    def roll_pid(self):
        error = self.drone_position[0] - self.setpoint[0]
        self.Iterm[0] += (error+self.prev_errors[0]) * self.Ki[0]
        output = self.Kp[0]*error + self.Kd[0] * \
            (error-self.prev_errors[0]) + self.Iterm[0]
        # store error value to use for next time computation
        self.prev_errors[0] = error
        # publish error
        self.roll_pub.publish(error)
        return output

    def pitch_pid(self):
        error = self.drone_position[1] - self.setpoint[1]
        self.Iterm[1] += (error+self.prev_errors[1]) * self.Ki[1]
        output = self.Kp[1]*error + self.Kd[1] * \
            (error-self.prev_errors[1]) + self.Iterm[1]
        # store error value to use for next time computation
        self.prev_errors[1] = error
        # publish error
        self.pitch_pub.publish(error)
        return output

    def alt_pid(self):
        error = self.drone_position[2] - self.setpoint[2]
        self.Iterm[2] += (error+self.prev_errors[2]) * self.Ki[2]
        output = self.Kp[2]*error + self.Kd[2] * \
            (error-self.prev_errors[2]) + self.Iterm[2]
        # store error value to use for next time computation
        self.prev_errors[2] = error
        # publish error
        self.alt_pub.publish(error)
        return output

    '''
    * Function Name: pid
    * Input:         None
    * Output:        None
    * Logic: Overall response in roll, pitch and altitude axis is calculated
             and published to '/drone_command' topic
    '''
    def pid(self):
        # Calculate overall PID and adjust control accordingly
        roll_out = self.roll_pid()
        self.cmd.rcRoll = 1500 - roll_out
        self.cmd.rcRoll = min(2000, self.cmd.rcRoll)
        self.cmd.rcRoll = max(1000, self.cmd.rcRoll)

        pitch_out = self.pitch_pid()
        self.cmd.rcPitch = 1500 + pitch_out
        self.cmd.rcPitch = min(2000, self.cmd.rcPitch)
        self.cmd.rcPitch = max(1000, self.cmd.rcPitch)

        alt_out = self.alt_pid()
        self.cmd.rcThrottle = 1500 + alt_out
        self.cmd.rcThrottle = min(2000, self.cmd.rcThrottle)
        self.cmd.rcThrottle = max(1000, self.cmd.rcThrottle)
        # publish control command
        self.command_pub.publish(self.cmd)

if __name__ == '__main__':
    
    e_drone = Edrone()
    e_drone.cum_timer = 0
    r = rospy.Rate(30)
    e_drone.load_coords()
    timer = 0
    # wait to recieve first setpoint to service
    while e_drone.setpoint[0] == None:
        pass
    while not rospy.is_shutdown():
        # run PID algorithm
        e_drone.pid()
        print('OUT: ', e_drone.prev_errors)
        # if drone is in threshold zone
        if e_drone.prev_errors[0] < .5 and \
           e_drone.prev_errors[1] < .5 and \
           e_drone.prev_errors[2] < 1:
            # timer: initialize timer to count service time
            timer = 0
            # start: mark the time of entering in threshold zone
            start = time.time()
            # while drone is within the threshold limits
            while e_drone.prev_errors[0] < .5 and \
                  e_drone.prev_errors[1] < .5 and \
                  e_drone.prev_errors[2] < 1:
                
                e_drone.pid()
                # print('IN: ', e_drone.prev_errors)

                # update timer while drone is within threshold limits
                timer = time.time() - start
                # publish the cummulative timer
                e_drone.hover_time_pub.publish(e_drone.cum_timer+timer)
                r.sleep()

        # update the cummulative timer with time of drone within thresholds
        # hence when next time drone enters the zone timer should start
        # from where it left off
        e_drone.cum_timer += timer
        r.sleep()
    print('DISARMING')
    e_drone.disarm()
