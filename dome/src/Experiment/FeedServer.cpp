/*
 * Manu S. Madhav
 * 16-May-2014
 */
#include "ros/ros.h"
#include "ros/console.h"
#include "dome_common_msgs/Angle.h"
#include "dome_common_msgs/DomeEvent.h"
#include "daq_interface/WriteDigital.h"
#include "daq_interface/ReadEncoder.h"
#include "daq_interface/PulseDigitalAction.h"
#include "actionlib/server/simple_action_server.h"
#include <cstdlib>

typedef actionlib::SimpleActionServer<daq_interface::PulseDigitalAction> PulseServer;
ros::ServiceClient write_client;
ros::Publisher event_pub;
//ros::ServiceClient encoderClient;
dome_common_msgs::DomeEvent event;
ros::Subscriber encoder_sub;
float startTheta = 0, currentTheta = 0, nextTheta = 0;
ros::Time feedStopTime;
daq_interface::WriteDigital feed;

void quit_callback(const dome_common_msgs::DomeEvent::ConstPtr& quit)
{
    ros::shutdown();
}

void encoder_angle_callback(const dome_common_msgs::Angle::ConstPtr& ang)
{
    currentTheta = ang -> theta;// - startTheta;
}

void execute_feed(const daq_interface::PulseDigitalGoalConstPtr& goal, PulseServer* as)
{
    //ros::NodeHandle n;
    ros::Rate loop_rate(10);
    feed.request.channel = goal->channel;

    if (!goal->persistent) { // One-time feed for a certain duration
        feedStopTime = ros::Time::now() + ros::Duration(goal->duration);

        feed.request.value = 1;
        write_client.call(feed);

        event.value = currentTheta;
        event_pub.publish(event);

        while (ros::ok()) {
            if (as->isPreemptRequested()) {
                as->setPreempted();
                break;
            }

            if (ros::Time::now() >= feedStopTime) {
                as->setSucceeded();
                break;
            }

            loop_rate.sleep();
        }

        feed.request.value = 0;
        write_client.call(feed);
    }
    else { // Feed at a fixed angle interval

        // Explicit initialization of starting angle
        //daq_interface::ReadEncoder encoder;
        //encoder.request.mode=3;
        //if (encoderClient.call(encoder)) startTheta = encoder.response.theta - startTheta;
        //nextTheta = 0;
        nextTheta = currentTheta;
        
        while (ros::ok())
        {
            if (as->isPreemptRequested()) {
                as->setPreempted();
                break;
            }

            if (currentTheta>=nextTheta) {
                feedStopTime = ros::Time::now() + ros::Duration(goal->duration);


                feed.request.value = 1;
                write_client.call(feed);

                nextTheta = currentTheta + goal->interval/2 + rand()%(int(goal->interval));
                //ROS_INFO("Current angle: %f, Next angle: %f",currentTheta, nextTheta);

                event.value = currentTheta;//encoder.response.theta;
                event_pub.publish(event);
            }

            if (ros::Time::now() >= feedStopTime) {
                feed.request.value = 0;
                write_client.call(feed);
            }

            loop_rate.sleep();
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "FeedServer");
    if (!ros::Time::isValid()) ros::Time::init();
    ros::NodeHandle n;
    write_client = n.serviceClient<daq_interface::WriteDigital>("write_digital",true);
    encoder_sub = n.subscribe("encoder_angle", 100, encoder_angle_callback);
    event_pub = n.advertise<dome_common_msgs::DomeEvent>("dome_event", 100);
    event.name = "feed";
    ros::Subscriber quit_sub = n.subscribe("quit",1,quit_callback);

    PulseServer pulseServer(n, "feed", boost::bind(&execute_feed, _1, &pulseServer), false);
    pulseServer.start();

    ros::spin();
    return 0;
}
