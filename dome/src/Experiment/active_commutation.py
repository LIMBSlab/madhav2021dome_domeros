#!/usr/bin/env python
import os,time
import rospy
import math
import random
import serial

from dome_common_msgs.msg import *
from dome_common_msgs.srv import *

class ActiveCommutation:
    def __init__(self):
	self.internalEncoderAngle = 0.0

        self.error = 0.0
        self.prevError = 0.0
        self.diffError = 0.0
        self.intError = 0.0

	self.setPoint = 0.0
	self.P = 50.0
	self.I = 0.0
	self.D = 0.1
	self.motorAnalogPin = 0
        self.initialSet = 0

        # Initiate serial communication with Arduino
        self.ser = serial.Serial(port='/dev/ttyACM0',baudrate=115200,writeTimeout=0.1)
        if self.ser.is_open == False:
            rospy.logwarn('Serial communication to Arduino not open')

        # Publishers / Subscribers
        rospy.Subscriber('internal_encoder_angle',Angle,self.internalEncoderAngleCallback)
        
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.commutationLoop()
            r.sleep()

    def commutationLoop(self):
	command = round(self.P*self.error + self.D*self.diffError + self.I*self.intError)
        self.ser.write('%4d'%command)
        return

    def internalEncoderAngleCallback(self,angle):
        if self.initialSet==0:
            self.setPoint = angle.theta
            self.initialSet = 1

        self.prevError = self.error
        self.error = self.internalEncoderAngle - self.setPoint
        self.diffError = self.error - self.prevError
        self.intError = self.intError + self.error

        self.internalEncoderAngle = angle.theta

if __name__ == '__main__':
    rospy.init_node('active_commutation')

    ActiveCommutation()
    rospy.spin()
