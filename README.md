# madhav2021dome_domeros
Code that runs within the [Robot Operating System (ROS)](http://www.ros.org) framework to operate the virtual reality Dome apparatus for investigating the cognitive map in rats.

Accompanying code for 
### The Dome: A virtual reality apparatus to create multimodal conflict during navigation in rodents
#### Manu S. Madhav, Ravikrishnan P. Jayakumar, Shahin G. Lashkari, Francesco Savelli, Hugh T. Blair, James J. Knierim, Noah J. Cowan\

## General Description

Each folder in the directory refers to a 'node', i.e. a ROS program that can be deployed standalone.
Each node performs a particular function and communicates to other nodes by passing 'messages' through a ROS 'master' program.
Designed to run on a real-time linux distribution (last tested on Ubuntu 16.04 + Xenomai 3.1.0). Real-time kernel is only a requirement for the data acquisition system node.
If a different, non-real time acquisition system is used, a vanilla Linux kernel should work.

### Node descriptions

#### dome

#### daq_interface

#### dome_common_msgs

#### pointgrey_camera_driver

#### rqt_dome_interface

### Non-node folders

#### scripts

#### arduino_commutation_dcmotor
