/*
 * Manu S. Madhav
 * 03-Oct-2014
 */
#include "ros/ros.h"
#include "ros/console.h"
#include "dome_common_msgs/DomeEvent.h"
#include "daq_interface/ReadDigital.h"

#include "daq_interface/ReadEncoder.h"
#include "dome_common_msgs/Drift.h"

using namespace std;

void quit_callback(const dome_common_msgs::DomeEvent::ConstPtr& quit)
{
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ZListener");
    ros::NodeHandle n;
    ros::Rate loop_rate(2000);

    int hallZPin = 6;
    if (ros::param::has("/hallZPin")) ros::param::get("/hallZPin",hallZPin);

    ros::ServiceClient read_client = n.serviceClient<daq_interface::ReadDigital>("read_digital",true);
    daq_interface::ReadDigital readHallZ;
    readHallZ.request.channel = hallZPin;

    ros::Publisher event_pub = n.advertise<dome_common_msgs::DomeEvent>("dome_event", 1000);
    dome_common_msgs::DomeEvent event_hall_z;
    event_hall_z.name = "hall_z";

    //
    ros::ServiceClient encoderClient = n.serviceClient<daq_interface::ReadEncoder>("read_encoder",true);
    daq_interface::ReadEncoder encoder;
    encoder.request.mode=3;
    ros::Subscriber quit_sub = n.subscribe("quit",1,quit_callback);

    ros::Publisher drift_pub = n.advertise<dome_common_msgs::Drift>("drift", 1000);

    dome_common_msgs::Drift drift;

    int nLaps = -1;
    //

    int hall_zVal = 0;
    if (read_client.call(readHallZ)) {
        readHallZ.request.prev = readHallZ.response.value;
    }
    
    while (ros::ok())
    {
        if (read_client.call(readHallZ)) {
            if (readHallZ.response.value != readHallZ.request.prev) {
                readHallZ.request.prev = readHallZ.response.value;
                event_hall_z.value = readHallZ.response.value;
                event_pub.publish(event_hall_z);
                //ROS_INFO("Hall_Z");

                //
                if (event_hall_z.value) {
                    nLaps+=1;
                    if (encoderClient.call(encoder)) 
                    {
                    	if (nLaps > 1)
                    	{
                    		drift.valuePrevLap = drift.value;
                    	}
                    	else
                    	{
                    		drift.valuePrevLap = 0;
                    	}

                        drift.value = encoder.response.theta - nLaps*360;
                        drift_pub.publish(drift);
                    }
                }
                //
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
