#!/usr/bin/env python
"""@package commutation.py

    Node to control the motor driving the central pillar. 
    Actuates according toe different modes of commutation.

    Manu S. Madhav
"""
import os,time
import rospy
import math
import random
import serial

from dome_common_msgs.msg import *
from dome_common_msgs.srv import *
from std_msgs.msg import *

from daq_interface.msg import *
from daq_interface.srv import *

import actionlib

class Commutation:
    def __init__(self):
        self.analogChannel = 0
        self.encoderAngle = 0.0
        self.ratHeadAngle = 0.0
        self.error_head = 0.0
        self.prevError_head = 0.0
        self.diffError_head = 0.0
        self.intError_head = 0.0
        self.ratBodyAngle = 0.0
        self.error_body = 0.0
        self.prevError_body = 0.0
        self.diffError_body = 0.0
        self.intError_body = 0.0
        self.P = 0.15
        self.I = 0.0
        self.D = 0.04
        self.setPoint = 15.0

        if rospy.has_param('/feedPin'):
            self.feedPin = rospy.get_param('/feedPin')
        else:
            self.feedPin = 0

        self.joystick_speed = 0
        self.joystick_throttle = 0

        rospy.set_param('/commutation/speed',0)

        try:
            rospy.wait_for_service('write_analog',2.0)
        except:
            rospy.logwarn('write_analog service not available')

        self.command_client = rospy.ServiceProxy('write_analog',WriteAnalog)

        # Publishers / Subscribers
        rospy.Subscriber('/rat_head_angle',Float64MultiArray,self.ratHeadAngleCallback)
        rospy.Subscriber('/rat_body_angle',Float64MultiArray,self.ratBodyAngleCallback)
        rospy.Subscriber('/encoder_angle',Angle,self.encoderAngleCallback)
        rospy.Subscriber('/joystick_event',DomeEvent,self.joystickEventCallback)
        self.feed_client = actionlib.SimpleActionClient('feed', PulseDigitalAction)

        self.heartbeat_client = rospy.ServiceProxy('write_digital',WriteDigital,persistent=True)
        self.heartbeatPin = 1
        self.heartbeatState = 0

        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.commutationLoop()
            r.sleep()
        else:
            self.command_client(self.analogChannel,0)

    def joystickEventCallback(self,event):
        """
        Callback to the joystick_events topic, that sets the 
        commutation parameters when the joystick is invoked to 
        actuate central pillar motor.
        """
        if event.name=='y':
            self.joystick_speed = -event.value*255
        elif event.name=='throttle':
            self.joystick_throttle = (-event.value+1)/2
        elif event.name=='trigger' and event.value==1:
            rospy.loginfo('trig')
            self.feed_client.cancel_goal()
            goal = PulseDigitalGoal()
            goal.channel = self.feedPin
            goal.duration = 1.5
            goal.persistent = False
            goal.interval = 0
            self.feed_client.send_goal(goal)

        return

    def updateParams(self):
        if rospy.has_param('/commutation/set_point'):
            self.setPoint = rospy.get_param('/commutation/set_point')

        if rospy.has_param('/commutation/mode'):
            self.commutationMode = rospy.get_param('/commutation/mode')

        else:
            rospy.set_param('/commutation/mode','joystick')
            self.commutationMode = 'joystick'

    def commutationLoop(self):
        """
        Main commutation loop that sets the speed and direction 
        of central pillar motor actuation according to commutation mode.
        """
        self.updateParams()

        # Send Heartbeat
        if self.heartbeatState == 0:
            self.heartbeat_client(self.heartbeatPin,1)
            self.heartbeatState = 1
        else:
            self.heartbeat_client(self.heartbeatPin,0)
            self.heartbeatState = 0

        if self.commutationMode == 'off':
            command = 0
        elif self.commutationMode == 'joystick':
            command = self.joystick_speed*self.joystick_throttle
        elif self.commutationMode == 'head_angle':
            command = self.P*self.error_head + self.D*self.diffError_head + self.I*self.intError_head
        elif self.commutationMode == 'body_angle':
            command = self.P*self.error_body + self.D*self.diffError_body + self.I*self.intError_body
        elif self.commutationMode == 'speed':
            command = rospy.get_param('/commutation/speed')*2.5/100
        else:
            rospy.loginfo('Unrecognized mode')
            command = 0

        print command

        if command>2.2:
            command=2.2
        elif command<-2.2:
            command=-2.2
        
        # Making sure arm does not go backwards
        if command<0:
            command=0

        self.command_client(self.analogChannel,2.5 + command)
        #rospy.loginfo('Command: %4d'%command)

        return

    def encoderAngleCallback(self,angle):
        """
        Callback to encoder_angle topic that computes wrapped 
        angle of the boom arm on the table.
        """
        self.encoderAngle = (angle.theta)%360
                
    def ratHeadAngleCallback(self,angle):
        """
        Callback to rat_head_angle topic that additionally computes the error 
        between it and the boom arm angle, its integral and derivative.
        """
        if abs(angle.data[0]) <= 180:
            self.prevError_head = self.error_head
            self.ratHeadAngle = (angle.data[0]) % 360

            self.error_head = (self.ratHeadAngle - self.encoderAngle - self.setPoint + 180) % 360 - 180

            if abs(self.error_head)>30:
                self.error_head = 0
            
            self.diffError_head = self.error_head - self.prevError_head
            self.intError_head = self.intError_head + self.error_head
        else:
            self.error_head = 0            

    def ratBodyAngleCallback(self,angle):
        """
        Callback to rat_body_angle topic that additionally computes the error 
        between it and the boom arm angle, its integral and derivative.
        """

        if abs(angle.data[0]) <= 180:
            self.prevError_body = self.error_body
            self.ratBodyAngle = (angle.data[0])  % 360

            self.error_body = (self.ratBodyAngle - self.encoderAngle - self.setPoint + 180) % 360 - 180

            if abs(self.error_body)>30:
                self.error_body = 0
            
            self.diffError_body = self.error_body - self.prevError_body
            self.intError_body = self.intError_body + self.error_body
        else:
            self.error_body = 0            
    

if __name__ == '__main__':
    rospy.init_node('commutation')

    Commutation()
    rospy.spin()
