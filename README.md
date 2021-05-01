# madhav2021dome_domeros
Code that runs within the [Robot Operating System (ROS)](http://www.ros.org) framework to operate the virtual reality Dome apparatus for investigating the cognitive map in rats.

Accompanying code for 
### The Dome: A virtual reality apparatus to create multimodal conflict during navigation in rodents
#### Manu S. Madhav, Ravikrishnan P. Jayakumar, Shahin G. Lashkari, Francesco Savelli, Hugh T. Blair, James J. Knierim, Noah J. Cowan\

## General Description

Most directories in this repositories contain a ROS 'package'. Each package is a collection of 'nodes', i.e. a ROS program that can be deployed standalone.
Each node performs a particular function and communicates to other nodes by passing 'messages' through a ROS 'master' program.
Designed to run on a real-time linux distribution (last tested on Ubuntu 16.04 + Xenomai 3.1.0). Real-time kernel is only a requirement for the data acquisition system node.
If a different, non-real time acquisition system is used, a vanilla Linux kernel should work.

### Package descriptions

#### dome

Package containing experimental control and visual cue generation nodes

#### daq_interface

Package containing a ROS interface to the National Instruments PCIe-6259 Data Acquisition System and nodes to use this interface to count encoder angles

#### dome_common_msgs

Package containing definitions for messages that are common between nodes

#### rqt_dome_interface

Package containing GUI node - GUI is written in Qt and operated under the [rqt framework](http://wiki.ros.org/rqt) to communicate with ROS.

### Other repositories

We also use the following package repositories in order to operate peripherals and other functionality

#### [pointgrey_camera_driver](https://github.com/ros-drivers/pointgrey_camera_driver)

### Non-node folders

#### scripts

#### arduino_commutation_dcmotor
