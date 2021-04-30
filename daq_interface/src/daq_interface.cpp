/*
 * Manu S. Madhav
 * 16-May-2014
 */

#include "ros/ros.h"
#include "ros/console.h"
#include "daqapi.h"
#include "daq_interface/ReadEncoder.h"
#include "daq_interface/ReadDigital.h"
#include "daq_interface/WriteDigital.h"
#include "daq_interface/WriteAnalog.h"
#include "daq_interface/ReadAnalog.h"
#include "daq_interface/DaqEvent.h"
#include <xenomai/init.h>

ros::Publisher daq_event_pub;
daq_interface::DaqEvent doEvent,diEvent,aoEvent,aiEvent;
//daqapi daq6259("analogy0");
daqapi *daq6259;

/*
void quit_callback(void)
{
    ros::shutdown();
}
*/

bool readEncoder(daq_interface::ReadEncoder::Request  &req,
        daq_interface::ReadEncoder::Response &res)
{
    if (req.mode==1) {
        res.theta = -(daq6259->readCounter(0)*360)/(20000*82/20);
    }
    else if (req.mode==2) {
        res.theta = (daq6259->readCounter(1)*360/8192);
    }
    else {
        res.theta = -(daq6259->readCounter(0)*360)/(20000*82/20) + (daq6259->readCounter(1)*360/8192);// - 65.38; // The offset is so that /encoder_angle 0 is always at lab x-axis
    }
    return true;
}

bool readDigital(daq_interface::ReadDigital::Request  &req,
        daq_interface::ReadDigital::Response &res)
{
    ros::Time before = ros::Time::now();
    res.value = daq6259->digitalRead(req.channel);

    if (res.value<0) {
        ROS_ERROR("Error reading digital channel %d",req.channel);}
    else if (res.value != req.prev) {
        diEvent.channel = req.channel;
        diEvent.value = res.value;
        diEvent.before = before;
        daq_event_pub.publish(diEvent);
    }

    return true;
}

bool writeDigital(daq_interface::WriteDigital::Request  &req,
        daq_interface::WriteDigital::Response &res)
{
    doEvent.channel = req.channel;
    doEvent.value = req.value;

    ros::Time before = ros::Time::now();
    daq6259->digitalWrite(req.channel,req.value);
    
    doEvent.before = before;

    if (req.channel != 1) // Bad way to avoid publishing events for the motor heartbeat signal
    {
        daq_event_pub.publish(doEvent);
    }

    return true;
}

bool readAnalog(daq_interface::ReadAnalog::Request  &req,
        daq_interface::ReadAnalog::Response &res)
{
    //ros::Time before = ros::Time::now();
    res.value = daq6259->analogRead(req.channel);

    //aiEvent.channel = req.channel;
    //aiEvent.avalue = res.value;
    //aiEvent.before = before;
    //daq_event_pub.publish(aiEvent);

    return true;
}

bool writeAnalog(daq_interface::WriteAnalog::Request  &req,
        daq_interface::WriteAnalog::Response &res)
{
    //aoEvent.channel = req.channel;
    //aoEvent.avalue = req.value;

    //ros::Time before = ros::Time::now();
    daq6259->analogWrite(req.channel,req.value);
    
    //aoEvent.before = before;
    //daq_event_pub.publish(aoEvent);

    return true;
}

int main (int argc, char **argv)
{
    char *const *argv2 = argv;
    xenomai_init(&argc, &argv2);

    ros::init(argc, argv, "daq_interface");
    if (!ros::Time::isValid()) ros::Time::init();
    ros::NodeHandle n;

    daq6259 = new daqapi("analogy0");

    for (int i=0; i<32; i++) daq6259->digitalWrite(i,0); // Initialize digital outs

    int pfi1A = 3, pfi1B = 11;
    if (ros::param::has("/intEncoderAPFI")) ros::param::get("/intEncoderAPFI",pfi1A);
    if (ros::param::has("/intEncoderBPFI")) ros::param::get("/intEncoderBPFI",pfi1B);
    daq6259->setupCounter(1,pfi1A,pfi1B,-1);

    int pfi0A = 0, pfi0B = 10, pfi0Z = 9;
    if (ros::param::has("/extEncoderAPFI")) ros::param::get("/extEncoderAPFI",pfi0A);
    if (ros::param::has("/extEncoderBPFI")) ros::param::get("/extEncoderBPFI",pfi0B);
    if (ros::param::has("/hallZPFI")) ros::param::get("/hallZPFI",pfi0Z);
    daq6259->setupCounter(0,pfi0A,pfi0B,pfi0Z);
    //daq6259->setupCounter(0,pfi0A,pfi0B,-1);
    
    ros::ServiceServer read_encoder_service = n.advertiseService("read_encoder",readEncoder);
    ros::ServiceServer read_digital_service = n.advertiseService("read_digital",readDigital);
    ros::ServiceServer write_digital_service = n.advertiseService("write_digital",writeDigital);
    ros::ServiceServer write_analog_service = n.advertiseService("write_analog",writeAnalog);
    //ros::ServiceServer read_analog_service = n.advertiseService("read_analog",readAnalog);

    daq_event_pub = n.advertise<daq_interface::DaqEvent>("daq_event",10000);
    doEvent.name = "DO";
    doEvent.avalue = 0.0;
    diEvent.name = "DI";
    diEvent.avalue = 0.0;
    aoEvent.name = "AO";
    aoEvent.value = 0;
    aiEvent.name = "AI";
    aiEvent.value = 0;

    //ros::Subscriber quit_sub = n.subscribe("quit",1,quit_callback);

    ros::spin();

    return 0;
}
