#!/bin/bash

if [[ $(cat /proc/analogy/devices | grep pcie-6259 | wc -l) = 1 ]]; then
	echo "DAQ device already linked"
else
	echo "Linking DAQ device..."
	analogy_config analogy0 analogy_ni_pcimio
fi

if [[ $(rostopic list 2>&1 | grep /rosout | wc -l) != 0 ]]; then
	echo "roscore already running"
else
	echo "Starting roscore..."
	gnome-terminal -x roscore
	sleep 2
fi

gnome-terminal -x roslaunch dome daq_interface.launch
sleep 2
gnome-terminal -x roslaunch dome peripherals.launch
sleep 2
gnome-terminal -x roslaunch dome visual.launch
sleep 2
rosrun rqt_dome_interface rqt_dome_interface
