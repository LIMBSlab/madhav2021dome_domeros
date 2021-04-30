#!/usr/bin/env python
import os,time
import rospy
import math
import random

from dome_common_msgs.msg import *
from dome_common_msgs.srv import *
from daq_interface.msg import *
from daq_interface.srv import *
from std_msgs.msg import *
from enum import Enum

import xml.etree.ElementTree as xml
from xml.dom import minidom

class EpochStatus(Enum):
    QUEUED = -1
    RUNNING = 0
    FINISHED = 1

class DomeObjectGroups(Enum):
    landmarks = 'landmarks'
    dots = 'dots'
    stripes = 'stripes'
    grid = 'grid'
    band = 'band'

class ExperimentController:
    def __init__(self):
        self.experimentStartTime = rospy.Time.now()
        self.experimentTime = rospy.Time.now()
        self.experimentStartAngle = 0.0
        self.experimentAngle = 0.0
        self.encoderAngle = 0.0
        self.ratHeadAngle = 0.0
        self.ratBodyAngle = 0.0
        self.modulationAngle = 0.0
        self.errorHead = 0.0
        self.errorBody = 0.0
        self.prevAngle = 0.0
        self.epochs = ()
        self.epoch = Epoch()
        self.epochIdx = -1
        self.epochStartAngle = 0.0
        self.epochEndAngle = 0.0
        self.epochTotalAngle = 0.0
        self.epochAngle = 0.0
        self.epochStartTime = rospy.Time.now()
        self.epochTime = rospy.Time.now()
        self.epochParams = {}
        self.gain = 0.0
        self.landmarkAngle = 0.0
        self.randomAngle = 0.0
        self.estimatedGain = 0.0
        self.gainError = 0.0
        self.integratedGainError = 0.0
        self.epochStartGain = 0.0
        self.epochStartEstGain = 0.0
        self.experimentXml = xml.Element('experiment')

        self.timestamp = 0.0;

        # Publishers / Subscribers
        self.visible_pub = rospy.Publisher('dome_visible',DomeVisible,queue_size=10)
        self.landmark_angle_pub = rospy.Publisher('landmark_angle',Float64MultiArray,queue_size=1,tcp_nodelay=True)
        self.experiment_status_pub = rospy.Publisher('experiment_status',ExperimentStatus,queue_size=10)
        self.epoch_progress_pub = rospy.Publisher('epoch_progress',EpochProgress,queue_size=10)
        self.modulation_angle_pub = rospy.Publisher('modulation_angle',Angle,queue_size=100)
        self.gain_pub = rospy.Publisher('gain',Gain,queue_size=100)
        self.event_pub = rospy.Publisher('dome_event',DomeEvent,queue_size=100)
        self.gain_desired_pub = rospy.Publisher('gain_desired',Gain,queue_size=100)

        # rospy.Subscriber('rat_head_angle',Float64,self.ratHeadAngleCallback)
        rospy.Subscriber('encoder_angle',Angle,self.encoderAngleCallback,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber('rat_head_angle',Float64MultiArray,self.ratHeadAngleCallback,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber('rat_body_angle',Float64MultiArray,self.ratBodyAngleCallback,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber('experiment_status',ExperimentStatus,self.experimentStatusCallback,queue_size=1)
        rospy.Subscriber('quit',DomeEvent,self.quitCallback,queue_size=1)
        rospy.Subscriber('landmark_angle',Float64MultiArray,self.landmarkAngleCallback,queue_size=1)
        rospy.Subscriber('gain',Gain,self.gainCallback,queue_size=1)
        rospy.Subscriber('estimated_gain',EstimatedGain,self.estGainCallback,queue_size=1)

        self.modulationAngleParam = 'encoder_angle'
        rospy.set_param('/experiment/modulation_angle','encoder_angle')

        try:
            rospy.wait_for_service('write_digital',2.0)
        except:
            rospy.logwarn('write_digital service not available')
        self.sync_client = rospy.ServiceProxy('write_digital',WriteDigital,persistent=True)

        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.runExperiment()
            r.sleep()

    # Return the angle on which the gain modulation is based, depending on the parameter setting
    def getModulationAngle(self):
        self.modulationAngleParam = rospy.get_param('/experiment/modulation_angle')

        if self.modulationAngleParam == 'encoder_angle':
            return self.encoderAngle
        elif self.modulationAngleParam == 'rat_head_angle':
            error_head = (((self.ratHeadAngle + 220 ) % 360 ) - (self.encoderAngle % 360) + 180 ) % 360 - 180
            if error_head > -15 and error_head < 45:
                self.errorHead = error_head
            return (self.encoderAngle + self.errorHead)
        elif self.modulationAngleParam == 'rat_body_angle':
            error_body = (((self.ratBodyAngle + 220 ) % 360 ) - (self.encoderAngle % 360) + 180 ) % 360 - 180
            if error_body > -15 and error_body < 45:
                self.errorBody = error_body
            return (self.encoderAngle + self.errorBody)

    def startExperiment(self):
        self.experimentStartAngle = self.getModulationAngle()
        self.experimentStartTime = rospy.Time.now()

        # Determine DAQ pin
        if rospy.has_param('/expBeginSyncPin'):
            self.expBeginSyncPin = rospy.get_param('/expBeginSyncPin')
        else:
            self.expBeginSyncPin = 7

        rospy.loginfo('Experiment started')

        # exp_begin event and pulse on DAQ
        self.event_pub.publish('exp_begin',1)
        self.sync_client(self.expBeginSyncPin,1)
        rospy.sleep(1.0)
        self.sync_client(self.expBeginSyncPin,0)
        self.event_pub.publish('exp_begin',0)

        # Re-initialize Xml tags
        self.experimentXml = xml.Element('experiment')

    def endExperiment(self):
        # Determine DAQ pin
        if rospy.has_param('/expEndSyncPin'):
            self.expEndSyncPin = rospy.get_param('/expEndSyncPin')
        else:
            self.expEndSyncPin = 2

        rospy.loginfo('Experiment ended')
        self.epochIdx = -1

        # exp_end event and pulse on DAQ
        self.event_pub.publish('exp_end',1)
        self.sync_client(self.expEndSyncPin,1)
        rospy.sleep(1.0)
        self.sync_client(self.expEndSyncPin,0)
        self.event_pub.publish('exp_end',0)

        # Write experiment xml file
        xmlStr = minidom.parseString(xml.tostring(self.experimentXml)).toprettyxml(indent="   ")
        xmlDir = os.path.expanduser(os.path.join('~','experiment','xmls'))
        xmlFile = os.path.join(xmlDir,time.strftime('experiment_%Y-%m-%d-%H-%M-%S.xml'))
        with open(xmlFile, "w") as f:
            f.write(xmlStr)

    def startNextEpoch(self):
        self.epochIdx += 1

        # Set state of previous epoch 
        if self.epochIdx>0:
            self.epochs[self.epochIdx-1].status = EpochStatus.FINISHED.value

        if self.epochIdx == len(self.epochs):
            self.endExperiment()
        else:
            # Make dictionary from param/values
            self.epoch = self.epochs[self.epochIdx]
            for p,v in zip(self.epoch.params,self.epoch.values):
                self.epochParams[p] = v

            # Set running parameters
            self.epochs[self.epochIdx].status = EpochStatus.RUNNING.value
            self.epochStartAngle = self.getModulationAngle()
            self.epochTotalAngle = self.epochParams['laps']*360
            self.epochEndAngle = self.epochStartAngle + self.epochTotalAngle
            self.epochStartTime = rospy.Time.now()
            self.epochStartGain = self.gain
            self.epochStartEstGain = self.estimatedGain
            self.randomAngle = 0.0
            self.gainError = 0.0
            self.integratedGainError = 0.0

            # Publish visibility
            for group in DomeObjectGroups:
                if group.value in self.epochParams:
                    self.visible_pub.publish(group.name,self.epochParams[group.value])

            if self.epochIdx == 0:
                self.startExperiment()

            # Publish dome event
            self.event_pub.publish('epoch',self.epochIdx+1)
            # Add epoch to xml
            self.addEpochToXml()

    def runExperiment(self):
        # Get and publish the current modulation angle
        self.modulationAngle = self.getModulationAngle()
        self.modulation_angle_pub.publish(self.modulationAngle)        
        self.epochAngle = self.modulationAngle-self.epochStartAngle
        self.epochTime = rospy.Time.now() - self.epochStartTime
        self.experimentAngle = self.modulationAngle-self.experimentStartAngle
        self.experimentTime = rospy.Time.now() - self.experimentStartTime
        deltaAngle = self.modulationAngle-self.prevAngle

        if self.epochIdx>=0:
            # Do experimenty stuff with the current epoch
            
            if self.epochAngle>self.epochTotalAngle:
                rospy.loginfo('Ending epoch %d',self.epochIdx)
                self.startNextEpoch()
                rospy.loginfo('Starting epoch %d',self.epochIdx)
                self.experiment_status_pub.publish(self.epochs)

            # This is the default control mode
            controlMode = 'gain' # Options are 'gain' or 'landmarks'

            # Epoch type: constant; No change in gain
            if self.epoch.type == 'constant':
                self.gain = self.epochParams['gain']
                controlMode = 'gain'

            # Epoch type: ramp; Linear change in gain
            elif self.epoch.type == 'ramp':
                self.gain = self.epochParams['gain0'] + self.epochAngle/self.epochTotalAngle * (self.epochParams['gain1']-self.epochParams['gain0'])
                controlMode = 'gain'

            # Epoch type: hyperramp; Hyperbolic change in gain
            elif self.epoch.type == 'hyperramp':
                self.gain = self.dehyper(self.hyper(self.epochParams['gain0']) + self.epochAngle/self.epochTotalAngle * (self.hyper(self.epochParams['gain1'])-self.hyper(self.epochParams['gain0'])))
                controlMode = 'gain'

            # Epoch type: sine; Sinusoidal change in gain / landmark location
            elif self.epoch.type =='sine':
                self.gain = self.epochParams['offset'] + self.epochParams['amp'] * math.sin(self.epochParams['freq']*self.epochAngle * math.pi/180 + self.epochParams['phase'])
                controlMode = 'gain'
                #controlMode = 'landmarks'

            # Epoch type: chirp; Sinusoid with changing frequency in gain / landmark location
            elif self.epoch.type == 'chirp':
                self.gain =  self.epochParams['offset'] + self.epochParams['amp'] * math.sin(self.epochParams['freq0']*self.epochTotalAngle* (math.pow(self.epochParams['freq1']/self.epochParams['freq0'],self.epochAngle/self.epochTotalAngle)-1) / math.log(self.epochParams['freq1']/self.epochParams['freq0']) * math.pi/180 + self.epochParams['phase'])
                controlMode = 'gain'
                #controlMode = 'landmarks'

            # Epoch type: jump; Random jump in landmark location
            elif self.epoch.type == 'jump':
                if self.epochAngle >= self.randomAngle:
                    jump = random.randrange(self.epochParams['jumpmin'],self.epochParams['jumpmax'],self.epochParams['jumpstep'])
                    self.landmarkAngle += jump
                    self.event_pub.publish('jump',jump)
                    self.randomAngle += random.randrange(self.epochParams['intervalmin'],self.epochParams['intervalmax'],self.epochParams['intervalstep'])
                controlMode = 'landmarks'
            
            elif self.epoch.type == 'constant_from_prev':
                self.gain = self.epochStartGain
                controlMode = 'gain'

            elif self.epoch.type == 'constant_from_prev_estGain':
                self.gain = self.epochStartEstGain
                controlMode = 'gain'

            elif self.epoch.type == 'ramp_from_prev_change':
                self.gain = self.epochStartGain + self.epochAngle/self.epochTotalAngle * self.epochParams['gainchange']
                controlMode = 'gain'

            elif self.epoch.type == 'ramp_from_prev_estGain_change':
                self.gain = self.epochStartEstGain + self.epochAngle/self.epochTotalAngle * self.epochParams['gainchange']
                controlMode = 'gain'

            elif self.epoch.type == 'ramp_from_prev_to':
                self.gain = self.epochStartGain + self.epochAngle/self.epochTotalAngle * (self.epochParams['gain1']-self.epochStartGain)
                controlMode = 'gain'

            elif self.epoch.type == 'ramp_from_prev_estGain_to':
                self.gain = self.epochStartEstGain + self.epochAngle/self.epochTotalAngle * (self.epochParams['gain1']-self.epochStartEstGain)
                controlMode = 'gain'

            elif self.epoch.type == 'sine_from_prev':
                self.gain = self.epochStartGain + self.epochParams['offset'] + self.epochParams['amp'] * math.sin(self.epochParams['freq']*self.epochAngle * math.pi/180 + self.epochParams['phase'])
                controlMode = 'gain'

            elif self.epoch.type == 'sine_from_prev_estGain':
                self.gain = self.epochStartEstGain + self.epochParams['offset'] + self.epochParams['amp'] * math.sin(self.epochParams['freq']*self.epochAngle * math.pi/180 + self.epochParams['phase'])
                controlMode = 'gain'

            elif self.epoch.type == 'gain_control_match':
                self.gainError = (self.estimatedGain - self.gain)*deltaAngle/360.0

                if abs(self.gainError) < 1.0:
                    self.integratedGainError = self.integratedGainError + self.gainError
                    self.gain = self.gain + self.epochParams['P']*self.gainError + self.epochParams['I']*self.integratedGainError

                if self.gain<-3:
                    self.gain = -3
                elif self.gain > 0.9:
                    self.gain = 0.9

                controlMode = 'gain'

            # Epoch type: gain_control_const; Closed-loop control to maintain constant internal gain
            elif self.epoch.type == 'gain_control_const':
                self.gain_desired_pub.publish(self.epochParams['gain'])
                self.gainError = self.epochParams['gain']-self.estimatedGain
                if abs(self.gainError) < 1.0:
                    integratedGainError = self.integratedGainError + self.gainError*deltaAngle/360.0
                else:
                    integratedGainError = self.integratedGainError

                self.gain = self.epochStartGain + self.epochParams['K'] * integratedGainError

                if self.gain<-3:
                    self.gain = -3
                elif self.gain > 0.9:
                    self.gain = 0.9
                else:
                    self.integratedGainError = integratedGainError
                controlMode = 'gain'

            # Epoch type: gain_control_const; Closed-loop control to maintain constant internal gain
            elif self.epoch.type == 'gain_control_delta':
                gain_desired = self.epochStartEstGain + self.epochParams['gain']
                self.gain_desired_pub.publish(gain_desired)
                self.gainError = gain_desired-self.estimatedGain
                if abs(self.gainError) < 1.0:
                    integratedGainError = self.integratedGainError + self.gainError*deltaAngle/360.0
                else:
                    integratedGainError = self.integratedGainError

                self.gain = self.epochStartGain + self.epochParams['K'] * integratedGainError

                if self.gain<-3:
                    self.gain = -3
                elif self.gain > 0.9:
                    self.gain = 0.9
                else:
                    self.integratedGainError = integratedGainError
                controlMode = 'gain'

            # Epoch type: gain_control_hyperbolic_const; Closed-loop hyperbolic control to maintain constant internal gain
            elif self.epoch.type == 'gain_control_hyperbolic_const':
                self.gainError = self.hyper(self.epochParams['gain'])-self.hyper(self.estimatedGain)
                integratedGainError = self.integratedGainError + self.gainError*deltaAngle/360.0
                self.gain = self.hyper(self.epochStartGain) + self.epochParams['K'] * integratedGainError
                self.gain = self.dehyper(self.gain)
                if self.gain<-2:
                    self.gain = -2
                else:
                    self.integratedGainError = integratedGainError
                controlMode = 'gain'

            # Publish gain and landmark_angle
            self.prevAngle = self.modulationAngle
            if controlMode == 'gain':
                self.gain_pub.publish(self.gain)
                landmark_angle_to_pub = Float64MultiArray(data=[self.landmarkAngle + self.gain*deltaAngle, self.timestamp])
                self.landmark_angle_pub.publish(landmark_angle_to_pub)
            elif controlMode == 'landmarks':
                self.gain_pub.publish(0.0)
                landmark_angle_to_pub = Float64MultiArray(data=[self.landmarkAngle, self.timestamp])
                self.landmark_angle_pub.publish(landmark_angle_to_pub)
        
        # Publish epoch progress for GUI
        self.epoch_progress_pub.publish(self.experimentTime,self.experimentAngle,self.epoch,self.epochIdx,self.epochTime,self.epochStartAngle,self.epochEndAngle,self.epochAngle)

    def hyper(self,g):
        return 1/(1-g)

    def dehyper(self,h):
        return 1-(1/h)

    def quitCallback(self,quitEvent):
        rospy.signal_shutdown('Manual quit')

    def experimentStatusCallback(self,experimentStatus):
        epochs = experimentStatus.epochs
        
        epochIdx = next((idx for idx,ep in enumerate(epochs) if ep.status==EpochStatus.RUNNING.value),len(epochs))

        if epochIdx==(self.epochIdx+1):
            self.startNextEpoch()
        elif self.epochIdx>=0 and epochIdx==len(epochs):
            self.endExperiment()

        self.epochs = epochs

    def estGainCallback(self,estGain):
        self.estimatedGain = 1-estGain.gain 

    def encoderAngleCallback(self,angle):
        self.encoderAngle = angle.theta

    def ratHeadAngleCallback(self,angle):
        if self.modulationAngleParam == 'rat_head_angle':
	    	self.timestamp = angle.data[1]
	    	# self.timestamp = rospy.Time.now().to_nsec()
        self.ratHeadAngle = (angle.data[0]) % 360

    def ratBodyAngleCallback(self,angle):
        if self.modulationAngleParam == 'rat_body_angle':
	    	self.timestamp = angle.data[1]
	    	# self.timestamp = rospy.Time.now().to_nsec()
        self.ratBodyAngle = (angle.data[0]) % 360

    def landmarkAngleCallback(self,angle):
        # self.landmarkAngle = angle.theta
        self.landmarkAngle = angle.data[0]

    def gainCallback(self,gain):
        self.gain = gain.value

    def addEpochToXml(self):
        e = xml.SubElement(self.experimentXml,'epoch',{'name':self.epoch.name,'type':self.epoch.type})
        for p,v in zip(self.epoch.params,self.epoch.values):
            x = xml.SubElement(e,p)
            x.text = str(v)

if __name__ == '__main__':
    rospy.init_node('experiment_controller')
    ExperimentController()
    rospy.spin()

