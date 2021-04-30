import os,sys,time
import roslib
import rospy
import subprocess,signal
import math
import xml.etree.ElementTree as xml

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *

from python_qt_binding import loadUi
from python_qt_binding import QtOpenGL 

from dome.srv import *
from dome.msg import *
from daq_interface.msg import *
from daq_interface.srv import *
from std_msgs.msg import *

import actionlib
import sensor_msgs.msg
from enum import Enum

class EpochStatus(Enum):
    """Enumeration of Epoch Status

    These states are shared between the GUI and Experiment Controller and used during experiment run
    """
    QUEUED = -1
    RUNNING = 0
    FINISHED = 1

class DomeDashboard(Plugin):
    """Main GUI Class
    
    Defines functions which provide slots for Qt Signals, as well as callbacks for ROS messages to which the GUI subscribes. Available signals from the GUI rqt_dome_interface.ui is used for interface elements. Communication between ROS callbacks and UI elements is done through custom signals.
    """

    """Custom signals"""
    encoderAngleChanged = Signal(float)
    ratHeadAngleChanged = Signal(float)
    ratBodyAngleChanged = Signal(float)
    landmarkAngleChanged = Signal(float)
    modulationAngleChanged = Signal(float)
    domeVisibleChanged = Signal(DomeVisible)
    gainChanged = Signal(float)
    queueUpdated = Signal(list)
    epochsUpdated = Signal(list)
    experimentStarted = Signal()
    experimentEnded = Signal()
    epochProgressed = Signal(float,float,int,float,float,float)
    feedStarted = Signal()
    feedStopped = Signal()

    def __init__(self, context):
        super(DomeDashboard, self).__init__(context)

        self._joint_sub = None

        ## Give QObjects reasonable names
        self.setObjectName('DomeDashboard')

        ## Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                dest="quiet",
                help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        #ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'rqt_dome_interface_simple.ui')
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'rqt_dome_interface.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names

        # Setup graphicsview
        self._widget.tableView.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.scene=QGraphicsScene()
        self._widget.tableView.setScene(self.scene)
        self.scene.setSceneRect(0,0,350,350)
        self._widget.tableView.setTransformationAnchor(0)

        # This makes the view OpenGL-accelerated. Usually makes
        # things much faster, but it *is* optional.
        self._widget.tableView.setViewport(QtOpenGL.QGLWidget())
        self._widget.tableView.update()

        # Service clients
        self.whats_in_dome = rospy.ServiceProxy('whats_in_dome', WhatsInDome,persistent=True)
        self.write_digital = rospy.ServiceProxy('write_digital', WriteDigital,persistent=True)
        
        # Populate dome rendering
        self.guiOffset = 135; # offset is difference between camera x-axis and lab x-axis
        self.populate()
        self._widget.tableView.centerOn(self.table)

        self.experimentRunning = False

        rospy.loginfo('Dome GUI started')

        self._widget.setObjectName('DomeDashboardPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Figure out pins
        if rospy.has_param('/feedPin'):
            self.feedPin = rospy.get_param('/feedPin')
        else:
            self.feedPin = 0

        if rospy.has_param('/expBeginSyncPin'):
            self.expBeginSyncPin = rospy.get_param('/expBeginSyncPin')
        else:
            self.expBeginSyncPin = 7

        if rospy.has_param('/expEndSyncPin'):
            self.expEndSyncPin = rospy.get_param('/expEndSyncPin')
        else:
            self.expEndSyncPin = 2

        # Subscriber instances
        self.encoderCount = 0
        self.landmarkCount = 0
        self.progressCount = 0
        self.gainCount = 0
        self.landmarkAngle = 0
        self.encoderAngle = 0
        self.ratHeadAngle = 0
        self.ratBodyAngle = 0
        self.sessionStartTime = rospy.Time.now()
        self.sessionStartAngle = 0.0
        rospy.Subscriber("encoder_angle", Angle, self.ros_encoderCallback)
        rospy.Subscriber("gain", Gain, self.ros_gainCallback)
        rospy.Subscriber("landmark_angle", Angle, self.ros_landmarkAngleCallback)
        rospy.Subscriber('experiment_status',ExperimentStatus,self.ros_experimentStatusCallback)
        rospy.Subscriber('epoch_progress',EpochProgress,self.ros_epochProgressCallback)
        rospy.Subscriber('rat_head_angle',Float64,self.ros_ratHeadAngleCallback)
        rospy.Subscriber('rat_body_angle',Float64,self.ros_ratBodyAngleCallback)
        rospy.Subscriber('dome_visible',DomeVisible,self.ros_domeVisibleCallback)

        # Publisher instances
        self.quit_pub = rospy.Publisher('quit',DomeEvent,queue_size=1);
        self.gain_pub = rospy.Publisher('gain',Gain,queue_size=10);
        self.event_pub = rospy.Publisher('dome_event',DomeEvent,queue_size=10);
        self.visible_pub = rospy.Publisher('dome_visible',DomeVisible,queue_size=10);
        self.landmark_angle_pub = rospy.Publisher("landmark_angle", Angle,queue_size=10);
        self.experiment_status_pub = rospy.Publisher('experiment_status',ExperimentStatus,queue_size=10)
        self.videoTrackingReset_pub = rospy.Publisher('tracking_reinitialize',String,queue_size=10)

        if rospy.has_param('/commutation/mode'):
            self.commutationMode = rospy.get_param('/commutation/mode')
            self.commutationModeSet(self.commutationMode)
        else:
            self.commutationModeSet('off')
            rospy.set_param('/commutation/mode','off')

        # Action clients
        self.feed_client = actionlib.SimpleActionClient('feed', PulseDigitalAction)

        #if not self.feed_client.wait_for_server(2.0):
        #    rospy.logerr('Feed Server not found');

        # Signal connections
        self._widget.xmlLoadBtn.clicked[bool].connect(self._handle_xmlLoadBtn_clicked)
        self._widget.quickFeedBtn.clicked[bool].connect(self._handle_quickFeedBtn_clicked)
        self._widget.quickGainBtn.clicked[bool].connect(self._handle_quickGainBtn_clicked)
        self._widget.quickLandmarkBtn.clicked[bool].connect(self._handle_quickLandmarkBtn_clicked)
        self._widget.startExperimentBtn.clicked[bool].connect(self._handle_startExperimentBtn_clicked)
        self._widget.cancelExperimentBtn.clicked[bool].connect(self._handle_cancelExperimentBtn_clicked)
        self._widget.nextEpochBtn.clicked[bool].connect(self._handle_nextEpochBtn_clicked)
        self._widget.postEventBtn.clicked[bool].connect(self._handle_postEventBtn_clicked)
        self._widget.sessionResetBtn.clicked[bool].connect(self._handle_sessionResetBtn_clicked)
        self._widget.quitBtn.clicked[bool].connect(self._handle_quitBtn_clicked)
        self._widget.eventEdit.returnPressed.connect(self._handle_eventEdit_returnPressed)
        self._widget.feedBtn.clicked[bool].connect(self._handle_feedBtn_clicked)
        self._widget.xmlBrowseBtn.clicked[bool].connect(self._handle_xmlBrowseBtn_clicked)
        self._widget.bagBrowseBtn.clicked[bool].connect(self._handle_bagBrowseBtn_clicked)
        self._widget.bagRecordBtn.clicked[bool].connect(self._handle_bagRecordBtn_clicked)
        self._widget.landmarksVisibleCheck.stateChanged[int].connect(self._handle_landmarksVisibleCheck_stateChanged)
        self._widget.landmarksVisibleCheck.clicked[bool].connect(self._handle_landmarksVisibleCheck_clicked)
        self._widget.dotsVisibleCheck.stateChanged[int].connect(self._handle_dotsVisibleCheck_stateChanged)
        self._widget.dotsVisibleCheck.clicked[bool].connect(self._handle_dotsVisibleCheck_clicked)
        self._widget.stripesVisibleCheck.stateChanged[int].connect(self._handle_stripesVisibleCheck_stateChanged)
        self._widget.stripesVisibleCheck.clicked[bool].connect(self._handle_stripesVisibleCheck_clicked)
        self._widget.gridVisibleCheck.stateChanged[int].connect(self._handle_gridVisibleCheck_stateChanged)
        self._widget.gridVisibleCheck.clicked[bool].connect(self._handle_gridVisibleCheck_clicked)
        self._widget.bandVisibleCheck.stateChanged[int].connect(self._handle_bandVisibleCheck_stateChanged)
        self._widget.bandVisibleCheck.clicked[bool].connect(self._handle_bandVisibleCheck_clicked)
        self._widget.clearEpochListBtn.clicked[bool].connect(self._handle_clearEpochListBtn_clicked)
        self._widget.clearQueueListBtn.clicked[bool].connect(self._handle_clearQueueListBtn_clicked)
        self._widget.commutationOffRadioBtn.toggled.connect(self.commutationModeGet)
        self._widget.commutationJoystickRadioBtn.toggled.connect(self.commutationModeGet)
        self._widget.commutationSpeedRadioBtn.toggled.connect(self.commutationModeGet)
        self._widget.commutationHeadAngleRadioBtn.toggled.connect(self.commutationModeGet)
        self._widget.commutationBodyAngleRadioBtn.toggled.connect(self.commutationModeGet)
        self._widget.commutationSpeedSlider.valueChanged.connect(self._handle_commutationSpeedSliderChanged)
        self._widget.videoTrackingResetBtn.clicked[bool].connect(self._handle_videoTrackingResetBtnClicked)
        self._widget.commutationSetPointBox.valueChanged[int].connect(self._handle_commutationSetPointBoxValueChanged)
        self._widget.gainManipulationSourceList.currentIndexChanged[str].connect(self._handle_gainManipulationSourceListChanged)

        # Context menu
        self._widget.epochList.setContextMenuPolicy(Qt.CustomContextMenu)
        self._widget.epochList.customContextMenuRequested[QPoint].connect(self._handle_epochListShowContextMenu)

        self._widget.queueList.setContextMenuPolicy(Qt.CustomContextMenu)
        self._widget.queueList.customContextMenuRequested[QPoint].connect(self._handle_queueListShowContextMenu)

        # Handle whenever queueList is edited
        model = self._widget.queueList.model() 
        model.rowsInserted.connect(self._handle_queueEdited)
        model.rowsRemoved.connect(self._handle_queueEdited)
        model.rowsMoved.connect(self._handle_queueEdited)

        #self._widget.queueList.setSortingEnabled()
        self.queueListEditFlag = False

        # Custom signal connections
        self.encoderAngleChanged[float].connect(self._handle_encoderAngleChanged)
        self.ratHeadAngleChanged[float].connect(self._handle_ratHeadAngleChanged)
        self.ratBodyAngleChanged[float].connect(self._handle_ratBodyAngleChanged)
        self.landmarkAngleChanged[float].connect(self._handle_landmarkAngleChanged)
        self.modulationAngleChanged[float].connect(self._handle_modulationAngleChanged)
        self.gainChanged[float].connect(self._handle_gainChanged)
        self.queueUpdated[list].connect(self._handle_queueUpdated)
        self.epochsUpdated[list].connect(self._handle_epochsUpdated)
        self.experimentStarted.connect(self._handle_experimentStarted)
        self.experimentEnded.connect(self._handle_experimentEnded)
        self.epochProgressed.connect(self._handle_epochProgressed)
        self.feedStarted.connect(self._handle_feedStarted)
        self.feedStopped.connect(self._handle_feedStopped)
        self.domeVisibleChanged.connect(self._handle_domeVisibleChanged)

        # Checkbox states
        try:
            self._widget.landmarksVisibleCheck.setChecked(any(obj.event != 'off' for obj in self.whats_in_dome('landmarks').objects)*2)
        except:
            self._widget.landmarksVisibleCheck.setChecked(False)

        try:
            self._widget.gridVisibleCheck.setChecked(any(obj.event != 'off' for obj in self.whats_in_dome('grid').objects)*2)
        except:
            self._widget.gridVisibleCheck.setChecked(False)

        try:
            self._widget.dotsVisibleCheck.setChecked(any(obj.event != 'off' for obj in self.whats_in_dome('dots').objects)*2)
        except:
            self._widget.dotsVisibleCheck.setChecked(False)

        try:
            self._widget.stripesVisibleCheck.setChecked(any(obj.event != 'off' for obj in self.whats_in_dome('stripes').objects)*2)
        except:
            self._widget.stripesVisibleCheck.setChecked(False)

        try:
            self._widget.bandVisibleCheck.setChecked(any(obj.event != 'off' for obj in self.whats_in_dome('band').objects)*2)
        except:
            self._widget.bandVisibleCheck.setChecked(False)

        # Default texts
        #self._widget.bagDirEdit.setText(os.path.expanduser(os.path.join('~','experiment','bags')))
        self._widget.bagDirEdit.setText(os.path.expanduser(os.path.join('/media','domeros','DomeDataSSD','experiment','bags')))
	
        self._widget.gainManipulationSourceList.addItem('Boom Arm');
        self._widget.gainManipulationSourceList.addItem('Rat Head');
        self._widget.gainManipulationSourceList.addItem('Rat Body');
        self._widget.gainManipulationSourceList.setCurrentIndex(0);
        rospy.set_param('/experiment/modulation_angle','encoder_angle')

        rospy.set_param('/commutation/set_point',self._widget.commutationSetPointBox.value())

        # Button colors
        self._widget.startExperimentBtn.setStyleSheet("background-color: green; color: yellow")
        self._widget.cancelExperimentBtn.setStyleSheet("background-color: red; color: yellow")

        self.isRecording = self.is_recording_on()
        self.isFeeding = self.is_feeding_on()
        if self.isRecording:
            self._widget.bagRecordBtn.setStyleSheet("background-color: red; color: yellow")
            self._widget.bagRecordBtn.setText("Stop")
        else:
            self._widget.bagRecordBtn.setStyleSheet("background-color: green; color: yellow")
            self._widget.bagRecordBtn.setText("Record")

        if self.isFeeding:
            self.feedStarted.emit()
        else:
            self.feedStopped.emit()
        
        # Set GUI update timer
        self.updateTimer = QTimer()
        self.updateTimer.timeout.connect(self._handle_update)
        self.updateTimer.start(1000)

    # Qt callback handling functions
    # Any GUI updates MUST be made from these callbacks, and not from any other 
    # functions or especially ROS callbacks

    def _handle_gainManipulationSourceListChanged(self,text):
        if text=='Boom Arm':
            rospy.set_param('/experiment/modulation_angle','encoder_angle')
        elif text=='Rat Head':
            rospy.set_param('/experiment/modulation_angle','rat_head_angle')
        elif text=='Rat Body':
            rospy.set_param('/experiment/modulation_angle','rat_body_angle')

    def _handle_commutationSetPointBoxValueChanged(self,value):
        rospy.set_param('/commutation/set_point',value)

    def _handle_videoTrackingResetBtnClicked(self,checked):
        self.videoTrackingReset_pub.publish('Reset')

    def _handle_commutationSpeedSliderChanged(self,value):
        rospy.set_param('/commutation/speed',value)

    def _handle_update(self):
        isFeedingOn = self.is_feeding_on()
        if isFeedingOn and not self.isFeeding:
            self.feedStarted.emit()
            self.isFeeding = 1
        elif self.isFeeding and not isFeedingOn:
            self.feedStopped.emit()
            self.isFeeding = 0

    # Qt handle called whenever the encoder angle is to be updated in the GUI
    def _handle_modulationAngleChanged(self,angle):
	pass

    def _handle_ratHeadAngleChanged(self,angle):
        self._widget.videoTrackingHeadAngleDisplay.setText("%.2f"%angle)

    def _handle_ratBodyAngleChanged(self,angle):
        self._widget.videoTrackingBodyAngleDisplay.setText("%.2f"%angle)

    def _handle_encoderAngleChanged(self,angle):
        sessionLaps = (angle-self.sessionStartAngle)/360
        sessionTime = rospy.Time.now()-self.sessionStartTime

        self._widget.ratAngleDisplay.setText("%.2f"%angle)
        self._widget.sessionLapsDisplay.setText("%.1f"%sessionLaps)
        self._widget.sessionTimeDisplay.setText("%.1f"%sessionTime.to_sec())

        self.boomGroup.setRotation(-angle + self.guiOffset)
        self._widget.tableView.centerOn(self.table)
        self._widget.tableView.update()

    def _handle_domeVisibleChanged(self,vis):
        if vis.type == 'landmarks':
            self._widget.landmarksVisibleCheck.setChecked(vis.visible)
        elif vis.type == 'grid':
            self._widget.gridVisibleCheck.setChecked(vis.visible)
        elif vis.type == 'band':
            self._widget.bandVisibleCheck.setChecked(vis.visible)
        elif vis.type == 'stripes':
            self._widget.stripesVisibleCheck.setChecked(vis.visible)
        elif vis.type == 'dots':
            self._widget.dotsVisibleCheck.setChecked(vis.visible)

    # Qt handle called whenever the landmarks angle is to be updated in the GUI
    def _handle_landmarkAngleChanged(self,angle):
        self._widget.landmarkAngleDisplay.setText("%.2f"%angle)

        self.landmarksGroup.setRotation(-angle + self.guiOffset)
        self._widget.tableView.centerOn(self.table)
        self._widget.tableView.update()

    # Qt handle called whenever the gain is to be updated in the GUI
    def _handle_gainChanged(self,gain):
        self._widget.gainDisplay.setText("%.2f"%gain)

    def _handle_clearEpochListBtn_clicked(self,checked):
        availableEpochs = []
        self.epochsUpdated.emit(availableEpochs)

    def _handle_clearQueueListBtn_clicked(self,checked):
        queuedEpochs = []
        self.queueUpdated.emit(queuedEpochs)

    # Qt handle called whenever the epoch list is updated
    def _handle_epochsUpdated(self,epochs):
        self._widget.epochList.clear()
        for ep in epochs:
            self._widget.epochList.addItem(self.qItemFromEpoch(ep))

        # Refresh mini-display
        self.populate()

    # Qt handle called whenever the queued epochs are updated, 
    # the GUI should then proceed to update the queue accordingly
    def _handle_queueUpdated(self,epochs):
        self.queueListEditFlag = True
        self._widget.queueList.clear()
        for ep in epochs:
            self._widget.queueList.addItem(self.qItemFromEpoch(ep))
        self.queueListEditFlag = False
        
    def ros_experimentStatusCallback(self,experimentStatus):
        # Update the GUI
        self.queueUpdated.emit(experimentStatus.epochs)
        # If last epoch is finished, end the experiment
        if experimentStatus.epochs[-1].status==EpochStatus.FINISHED.value:
            self.experimentEnded.emit()
        return

    def ros_epochProgressCallback(self,prg):
        if self.progressCount<20:
            self.progressCount += 1
        else:
            epochLaps = prg.epochAngle/360
            epochTotalLaps = (prg.epochEndAngle-prg.epochStartAngle)/360
            experimentLaps = prg.experimentAngle/360
            self.epochProgressed.emit(experimentLaps,prg.experimentTime.to_sec(),prg.epochIdx,epochLaps,epochTotalLaps,prg.epochTime.to_sec())
            self.progressCount = 0

    def ros_domeVisibleCallback(self,vis):
        self.domeVisibleChanged.emit(vis)

    def ros_ratHeadAngleCallback(self,angle):
        self.ratHeadAngle = angle.data % 360
        self.ratHeadAngleChanged.emit(self.ratHeadAngle)

    def ros_ratBodyAngleCallback(self,angle):
        self.ratBodyAngle = angle.data % 360
        self.ratBodyAngleChanged.emit(self.ratBodyAngle)

    def ros_encoderCallback(self,angle):
        self.encoderAngle = angle.theta; 

        if self.encoderCount<20:
            self.encoderCount += 1
        else:
            self.encoderAngleChanged.emit(self.encoderAngle)
            self.encoderCount = 0
        
    def ros_gainCallback(self,gain):
        if self.gainCount<20:
            self.gainCount += 1
        else:
            self.gainChanged.emit(gain.value)
            self.gainCount = 0

    def ros_landmarkAngleCallback(self,angle):
        self.landmarkAngle = angle.theta

        if self.landmarkCount<20:
            self.landmarkCount += 1
        else:
            self.landmarkAngleChanged.emit(angle.theta)
            self.landmarkCount = 0

    def _handle_epochProgressed(self,experimentLaps,experimentTime,epochIdx,epochLaps,epochTotalLaps,epochTime):
        self._widget.experimentLapsDisplay.setText("%.1f"%(experimentLaps))
        self._widget.experimentTimeDisplay.setText("%.1f"%(experimentTime))
        self._widget.epochIdxDisplay.setText("%d"%(epochIdx+1))
        self._widget.epochLapsDisplay.setText("%.1f/%.1f"%(epochLaps,epochTotalLaps))
        self._widget.epochTimeDisplay.setText("%.1f"%(epochTime))

    def commutationModeSet(self,mode):
        if mode == 'off':
            self._widget.commutationOffRadioBtn.setChecked(True)
        elif mode == 'joystick':
            self._widget.commutationJoystickRadioBtn.setChecked(True)
        elif mode == 'speed':
            self._widget.commutationSpeedRadioBtn.setChecked(True)
        elif mode == 'head_angle':
            self._widget.commutationHeadAngleRadioBtn.setChecked(True)
        elif mode == 'body_angle':
            self._widget.commutationBodyAngleRadioBtn.setChecked(True)
        else:
            rospy.logerr('Commutation mode not recognized')

    def commutationModeGet(self):
        if self._widget.commutationOffRadioBtn.isChecked():
            mode = 'off'
        elif self._widget.commutationJoystickRadioBtn.isChecked():  
            mode = 'joystick'
        elif self._widget.commutationSpeedRadioBtn.isChecked(): 
            mode = 'speed'
        elif self._widget.commutationHeadAngleRadioBtn.isChecked(): 
            mode = 'head_angle'
        elif self._widget.commutationBodyAngleRadioBtn.isChecked(): 
            mode = 'body_angle'
        else:
            mode = 'off'
        
        rospy.set_param('/commutation/mode',mode)
        rospy.set_param('/commutation/speed',self._widget.commutationSpeedSlider.value()) 
        
    def populate(self):
        W = self.scene.width()
        H = self.scene.height()
        cX = W/2;
        cY = H/2;

        # Boom arm
        boomWidth = 0.02*H;
        boomLength = 0.35*W;
        self.boom = QGraphicsRectItem(0,-boomWidth/2,boomLength,boomWidth)
        self.boom.setBrush(Qt.black)

        # Rat
        ratHeadWidth = 0.05*W;
        ratHeadLength = 0.07*W;
        ratBodyWidth = 0.07*W;
        ratBodyLength = 0.1*W;
        self.ratBody = QGraphicsEllipseItem(boomLength-ratBodyWidth,0,ratBodyWidth,ratBodyLength)
        self.ratHead = QGraphicsEllipseItem(boomLength-ratBodyWidth/2-ratHeadWidth/2,-ratHeadLength,ratHeadWidth,ratHeadLength)
        self.ratBody.setBrush(Qt.darkGray)
        self.ratHead.setBrush(Qt.darkGray)

        # Table
        self.table = QGraphicsEllipseItem(-0.45*W,-0.45*H,0.9*W,0.9*H)
        self.table.setBrush(Qt.lightGray)

        # Define standard landmarks and create dictionary
        pizza_lm = QGraphicsPolygonItem(QPolygonF([QPointF(0.4*W,-0.05*H), QPointF(0.4*W,0.05*H), QPointF(0.5*W,0)]))
        pizza_lm.setBrush(Qt.darkGreen)

        ellipse_lm = QGraphicsEllipseItem(0.4*W,-0.05*H,0.05*W,0.1*H)
        ellipse_lm.setBrush(Qt.darkRed)

        vertical_bar_lm = QGraphicsRectItem(0.4*W,-0.025*H,0.1*W,0.05*H)
        vertical_bar_lm.setBrush(Qt.darkBlue)

        other_lm = QGraphicsSimpleTextItem('L')
        other_lm.setBrush(Qt.black)

        landmarksList = {
            'pizza': pizza_lm,
            'ellipse': ellipse_lm,
            'vertbar': vertical_bar_lm,
            'other': other_lm
        }

        # Populate displyed landmarks from list
        self.landmarks = [];
        try:
            landmarkObjs = self.whats_in_dome('landmark').objects
            for obj in landmarkObjs:
                qitem = landmarksList.get(obj.name,landmarksList['other'])
                qitem.setRotation(-obj.theta*180/math.pi + self.guiOffset)
                self.landmarks.append(qitem)
            if not len(self.landmarks):
                self.landmarks.append(landmarksList['other'])
        except:
            self.landmarks.append(landmarksList['other'])

        # Experiment start marker
        self.startExperimentMarker = QGraphicsEllipseItem(0.4*W,-0.025*H,0.05*W,0.05*H)
        self.startExperimentMarker.setBrush(Qt.black)

        # Create groups of items for easier handling
        self.boomGroup = QGraphicsItemGroup()
        self.boomGroup.addToGroup(self.ratHead)
        self.boomGroup.addToGroup(self.ratBody)
        self.boomGroup.addToGroup(self.boom)
              
        self.landmarksGroup = QGraphicsItemGroup()
        for i in range(len(self.landmarks)):
            self.landmarksGroup.addToGroup(self.landmarks[i])
        
        # Add all items to scene in order
        self.scene.addItem(self.table)
        self.table.setPos(cX,cY);

        self.scene.addItem(self.boomGroup)
        self.boomGroup.setPos(cX,cY)
        
        self.scene.addItem(self.startExperimentMarker)
        self.startExperimentMarker.setPos(cX,cY)
        self.startExperimentMarker.hide()

        self.scene.addItem(self.landmarksGroup)
        self.landmarksGroup.setPos(cX,cY)

    def parseXML(self):
        # TODO_V2: Param type checking, warnings, errors
        filename = self._widget.xmlFileEdit.text()
        exp = xml.parse(filename).getroot()

        if exp.tag!='experiment':
            rospy.logerr("experiment tag is not root in %s",filename)
            return 0

        self.epochs = {}
        
        availableEpochs = []
        for child in exp:
            if child.tag != 'epoch':
               rospy.logerr("epoch tag not found in %s",filename)
               return 0
            if 'type' not in child.attrib:
               rospy.logerr("type not specified for epoch")
               return 0
            if 'name' not in child.attrib:
               rospy.logerr("name not specified for epoch")
               return 0

            epochName = child.attrib['name']
            epochType = child.attrib['type']

            params = ()
            values = ()
            for child2 in child:
               params = params+(child2.tag,)
               values = values+(float(child2.text),)

            # Create epoch
            availableEpochs.append(Epoch(epochName,epochType,params,values,-1))
            # Add to epoch list
                
        self.epochsUpdated.emit(availableEpochs)
            
        self.totalLaps = 0
        for laps in exp.iter('laps'):
            self.totalLaps = self.totalLaps + int(float(laps.text))
        rospy.loginfo("%d laps in total",self.totalLaps)

    def qItemFromEpoch(self,epoch):
        listItem = QListWidgetItem(epoch.name)

        # Set tooltip text to epoch description
        toolTipText = "type: " + epoch.type + "\n"
        for p,v in zip(epoch.params,epoch.values):
            toolTipText += p + " : " + str(v) + "\n"
        listItem.setToolTip(toolTipText)

        # Set data to epoch
        listItem.setData(Qt.UserRole,epoch)

        # Set font and flags according to epoch status
        normalFlags = Qt.ItemIsEnabled | Qt.ItemIsDragEnabled | Qt.ItemIsDropEnabled | Qt.ItemIsSelectable
        disabledFlags = Qt.NoItemFlags
        runningFlags = Qt.ItemIsEnabled | Qt.ItemIsSelectable
        font = listItem.font()
        if epoch.status == EpochStatus.FINISHED.value:
            listItem.setFlags(disabledFlags)
            font.setBold(False)
        elif epoch.status == EpochStatus.RUNNING.value:
            listItem.setFlags(runningFlags)
            font.setBold(True)
        else:
            listItem.setFlags(normalFlags)
            font.setBold(False)
        listItem.setFont(font)

        return listItem
    
    def _handle_queueEdited(self):
        if not self.queueListEditFlag:
            queueEpochs = []
            for k in range(self._widget.queueList.count()):
                # This is a workaround since in the specific case of drag/drop 
                # from epochList to queueList, the item is somehow not accessible 
                # in the rowInserted callback
                if self._widget.queueList.item(k).text()=="":
                    queueEpochs.append(self._widget.epochList.currentItem().data(Qt.UserRole))
                else:
                    queueEpochs.append(self._widget.queueList.item(k).data(Qt.UserRole))

            # Sort by status, i.e. make sure that the order is QUEUED-->RUNNING-->FINISHED
            queueEpochsSorted = sorted(queueEpochs,key = lambda x: x.status,reverse=True)
            if queueEpochs != queueEpochsSorted:
                self.queueUpdated.emit(queueEpochsSorted)
            
            self.experiment_status_pub.publish(queueEpochsSorted)

    def _handle_epochListShowContextMenu(self,pos):
        menu = QMenu(self._widget.epochList)
        copyToQueueAction = menu.addAction("Copy to Queue")
        action = menu.exec_(self._widget.epochList.mapToGlobal(pos))
        
        if action==copyToQueueAction:
            self._widget.queueList.addItem(self._widget.epochList.currentItem().clone())

    def _handle_queueListShowContextMenu(self,pos):
        menu = QMenu(self._widget.queueList)
        deleteAction = menu.addAction("Delete")
        action = menu.exec_(self._widget.queueList.mapToGlobal(pos))
        
        if action==deleteAction:
            self._widget.queueList.takeItem(self._widget.queueList.currentRow())

    def _handle_landmarksVisibleCheck_stateChanged(self,state):
        if state:
            rospy.loginfo('Turning on landmarks')
        else:
            rospy.loginfo('Turning off landmarks')
        
    def _handle_landmarksVisibleCheck_clicked(self,state):
        self.visible_pub.publish("landmarks",state)

    def _handle_dotsVisibleCheck_stateChanged(self,state):
        if state:
            rospy.loginfo('Turning on dots')
        else:
            rospy.loginfo('Turning off dots')

    def _handle_dotsVisibleCheck_clicked(self,state):
        self.visible_pub.publish("dots",state)

    def _handle_stripesVisibleCheck_stateChanged(self,state):
        if state:
            rospy.loginfo('Turning on stripes')
        else:
            rospy.loginfo('Turning off stripes')

    def _handle_stripesVisibleCheck_clicked(self,state):
        self.visible_pub.publish("stripes",state)

    def _handle_gridVisibleCheck_stateChanged(self,state):
        if state:
            rospy.loginfo('Turning on grid')
        else:
            rospy.loginfo('Turning off grid')

    def _handle_gridVisibleCheck_clicked(self,state):
        self.visible_pub.publish("grid",state)

    def _handle_bandVisibleCheck_stateChanged(self,state):
        if state:
            rospy.loginfo('Turning on band')
        else:
            rospy.loginfo('Turning off band')

    def _handle_bandVisibleCheck_clicked(self,state):
        self.visible_pub.publish("band",state)
        
    def _handle_postEventBtn_clicked(self,checked):
        self.event_pub.publish(self._widget.eventEdit.text(),999)
        rospy.loginfo('Posted event')

    def _handle_sessionResetBtn_clicked(self,checked):
        self.sessionStartAngle = self.encoderAngle
        self.sessionStartTime = rospy.Time.now()

    def _handle_quitBtn_clicked(self,checked):
        # Send instruction for most nodes to quit
        self.quit_pub.publish("quit",1)
        # Wait a bit
        time.sleep(1)

        # Kill all other active nodes
        os.system("rosnode kill -a")
        # Wait a bit
        time.sleep(1)

        # GUI will have to be murdered explicitly
        os.system("pkill -f rqt_dome_interface")
    
    def _handle_eventEdit_returnPressed(self):
        self.event_pub.publish(self._widget.eventEdit.text(),999)
        rospy.loginfo('Posted event')

    def _handle_bagBrowseBtn_clicked(self,checked):
        bagDir = os.path.expanduser(os.path.join('~','experiment','bags'))
        res = QFileDialog.getExistingDirectory(None,"Select directory for recording",bagDir,QFileDialog.ShowDirsOnly)
        self._widget.bagDirEdit.setText(res)
        rospy.loginfo('Selected %s',res)

    def _handle_feedBtn_clicked(self,checked):
        if not self.isFeeding:
            goal = PulseDigitalGoal()
            goal.channel = self.feedPin
            goal.duration = self._widget.expFeedDurationBox.value()
            goal.persistent = True
            goal.interval = self._widget.expFeedIntervalBox.value()
            self.feed_client.send_goal(goal)

            self.feedStarted.emit()
            rospy.loginfo('Started feeding every %.1f degrees',self._widget.expFeedIntervalBox.value())
            self.isFeeding = 1
        else:
            self.feed_client.cancel_goal()
            self.feedStopped.emit()
            rospy.loginfo('Stopping feeding')
            self.isFeeding = 0

    def _handle_bagRecordBtn_clicked(self,checked):
        if not self.isRecording:
            if not os.path.isdir(self._widget.bagDirEdit.text()):
                self._widget.bagDirEdit.setText(os.path.expanduser('~/experiment/bags'))

            #command = "rosbag record -a -o domeExperimentData -x \/camera/.* \/camera_info"
            command = "roslaunch dome record_all_except_camera.launch"
            self.recordDataPid = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self._widget.bagDirEdit.text())

            command = "rosbag record -b 1024 -o domeExperimentCamera /camera/image_mono/compressed /camera_info"
            command = "roslaunch dome record_camera.launch"
            self.recordCameraPid = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self._widget.bagDirEdit.text())

            rospy.sleep(3)
            # Force publishing of landmark objects to /dome_objects topic
            self.whats_in_dome('landmark')
            
            self._widget.bagRecordBtn.setStyleSheet("background-color: red; color: yellow")
            self._widget.bagRecordBtn.setText("Stop")
            rospy.loginfo('Recording bag files')
            self.isRecording = 1

        else:
            self.terminate_ros_node("/record")
            self._widget.bagRecordBtn.setStyleSheet("background-color: green; color: yellow")
            self._widget.bagRecordBtn.setText("Record")
            rospy.loginfo('Stopping recording')

            self.isRecording = 0

    def is_recording_on(self):
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        ret = 0
        for str in list_output.split("\n"):
            if (str.startswith("/record")):
                ret = 1
        return ret

    def is_feeding_on(self):
        return self.feed_client.simple_state != actionlib.SimpleGoalState.DONE
   
    # Credit: schadlerm
    # http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
    def terminate_ros_node(self,s):
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)

    # Credit: sergey_alexandrov
    # http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
    def terminate_process_and_children(self,p):
        ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
                os.kill(int(pid_str), signal.SIGINT)
        p.terminate()
        p.kill()

    def _handle_xmlLoadBtn_clicked(self,checked):
        # TODO_V2: Add file checking, warnings, errors
	if os.path.exists(self._widget.xmlFileEdit.text()):
	    self.xmlFilePath,self.xmlFileName = os.path.split(self._widget.xmlFileEdit.text())
	    self.parseXML()
	    rospy.loginfo('Loaded and parsed %s',self.xmlFileName)

    def _handle_xmlBrowseBtn_clicked(self,checked):
        xmlDir = os.path.expanduser(os.path.join('~','experiment','xmls'))
        res = QFileDialog.getOpenFileName(None,"Select experiment file",xmlDir,"XML files (*.xml)")
        self._widget.xmlFileEdit.setText(os.path.join(res[1],res[0]))
        rospy.loginfo('Selected %s',res[0])

    # Handle all the GUI transitions when an experiment ends (or is cancelled)
    def _handle_experimentEnded(self):
        rospy.loginfo('Experiment Done')

        self.startExperimentMarker.hide()
        
        # Set button states
        self._widget.startExperimentBtn.setEnabled(True)
        self._widget.cancelExperimentBtn.setEnabled(False)
        self._widget.startExperimentBtn.setStyleSheet("background-color: green; color: yellow")
        self._widget.cancelExperimentBtn.setStyleSheet("background-color: red; color: yellow")

        #self._widget.epochIdxDisplay.setText("0")
        #self._widget.epochLapsDisplay.setText("0")
        #self._widget.epochTimeDisplay.setText("0")

        #self._widget.experimentLapsDisplay.setText("0")
        #self._widget.experimentTimeDisplay.setText("0")

        self.experimentRunning = False

    # Handle all the GUI transitions when an experiment starts
    def _handle_experimentStarted(self):
        self.startExperimentMarker.setRotation(-self.encoderAngle + self.guiOffset)
        self.startExperimentMarker.show()
        
        # Set button states
        self._widget.startExperimentBtn.setEnabled(False)
        self._widget.cancelExperimentBtn.setEnabled(True)
        self._widget.startExperimentBtn.setStyleSheet("background-color: red; color: yellow")
        self._widget.cancelExperimentBtn.setStyleSheet("background-color: green; color: yellow")

        self.experimentRunning = True

    def _handle_feedStarted(self):
        self._widget.feedBtn.setStyleSheet("background-color: red; color: yellow")
        self._widget.feedBtn.setText("Stop Feeding")

    def _handle_feedStopped(self):
        self._widget.feedBtn.setStyleSheet("background-color: green; color: yellow")
        self._widget.feedBtn.setText("Start Feeding")

    def _handle_nextEpochBtn_clicked(self,checked):
        if self.experimentRunning:
            nQueuedEpochs = self._widget.queueList.count()
            nextEpochIdx = next((k for k in range(nQueuedEpochs) \
            if self._widget.queueList.item(k).data(Qt.UserRole).status==EpochStatus.QUEUED.value),nQueuedEpochs)

            if nextEpochIdx < nQueuedEpochs:
                self._widget.queueList.item(nextEpochIdx).data(Qt.UserRole).status = EpochStatus.RUNNING.value
                rospy.loginfo("Next epoch started")
            else:
                rospy.loginfo("No more epochs queued")

            if nextEpochIdx > 0:
                self._widget.queueList.item(nextEpochIdx-1).data(Qt.UserRole).status = EpochStatus.FINISHED.value

            # Publish queue
            queueEpochs = []
            for k in range(self._widget.queueList.count()):
                queueEpochs.append(self._widget.queueList.item(k).data(Qt.UserRole))
            self.experiment_status_pub.publish(queueEpochs)

    def _handle_cancelExperimentBtn_clicked(self,checked):
        if self.experimentRunning:
            rospy.loginfo('Cancelling Experiment')

            # Set all epochs to ran
            for k in range(self._widget.queueList.count()):
                self._widget.queueList.item(k).data(Qt.UserRole).status = EpochStatus.FINISHED.value

            # Publish queue
            queueEpochs = []
            for k in range(self._widget.queueList.count()):
                queueEpochs.append(self._widget.queueList.item(k).data(Qt.UserRole))
            self.experiment_status_pub.publish(queueEpochs)

            rospy.loginfo('Cancelled Experiment')

    def _handle_startExperimentBtn_clicked(self,checked):
        if not self.experimentRunning and self._widget.queueList.count():
            rospy.loginfo('Starting Experiment')

            # Set all epochs to not run except for the first one
            for k in range(1,self._widget.queueList.count()):
                self._widget.queueList.item(k).data(Qt.UserRole).status = EpochStatus.QUEUED.value
            # Set first epoch to run
            self._widget.queueList.item(0).data(Qt.UserRole).status = EpochStatus.RUNNING.value

            # Publish queue
            queueEpochs = []
            for k in range(self._widget.queueList.count()):
                queueEpochs.append(self._widget.queueList.item(k).data(Qt.UserRole))
            self.experiment_status_pub.publish(queueEpochs)

            self.experimentStarted.emit()

    def _handle_quickGainBtn_clicked(self,checked):
        self.gain_pub.publish(self._widget.quickGainBox.value())
        rospy.loginfo('Gain set to %.1f',self._widget.quickGainBox.value())

    def _handle_quickFeedBtn_clicked(self,checked):
        goal = PulseDigitalGoal()
        goal.channel = self.feedPin
        goal.duration = self._widget.quickFeedDurationBox.value()
        goal.persistent = False
        goal.interval = 0
        self.feed_client.send_goal(goal)
        rospy.loginfo('Feeding now for %.1f seconds',self._widget.quickFeedDurationBox.value())

    def _handle_quickLandmarkBtn_clicked(self,checked):
        self.landmark_angle_pub.publish(self._widget.quickLandmarkBox.value())
        rospy.loginfo('Landmark Angle set to %d degrees',self._widget.quickLandmarkBox.value())
