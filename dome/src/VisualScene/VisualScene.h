#pragma once

#include <iostream>
#include <fstream>
#include <streambuf>
#include <ros/ros.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <vector>
#include <array>
#include <string>
#include <climits>
#include "DomeShapeLibrary.h"
#include "helper.h"
#include "textfile.h"
#include "dome_common_msgs/DomeEvent.h"
#include "dome_common_msgs/DomeObj.h"
#include "dome_common_msgs/DomeVisible.h"
#include "dome_common_msgs/WhatsInDome.h"
#include <chrono>
#include <XmlRpcValue.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

#define PI 3.1415926535897932384626433832795

// Defining types to increase readability
typedef XmlRpc::XmlRpcValue XmlRpcValue;
typedef map<string, XmlRpcValue> CueParamMap;
typedef map<string, CueParamMap> CueGroupParamMap;
typedef map<string, CueGroupParamMap> CueConstellationParamMap;

typedef map<string, DomeShape*> CueGroup;
typedef map<string, CueGroup> CueConstellation;


/***** PROTOTYPES *****/
void reshape(int, int);
void keyPressed (unsigned char, int, int);
void makeCues(CueConstellationParamMap&);
void toggleFullscreen(void);
bool whatsInDome(dome_common_msgs::WhatsInDome::Request&, dome_common_msgs::WhatsInDome::Response&);
dome_common_msgs::DomeObj objectFromCue(DomeShape*, double);
void quitVisualScene(void);
void mainDisplay(void);
void setShaders(void);
void landmark_angle_callback(const std_msgs::Float64MultiArray::ConstPtr&);
// void landmark_angle_callback(const dome_common_msgs::Angle::ConstPtr&);
void quit_callback(const dome_common_msgs::DomeEvent::ConstPtr&);
int main(int argc, char** argv);
void visible_callback(const dome_common_msgs::DomeVisible::ConstPtr&);
