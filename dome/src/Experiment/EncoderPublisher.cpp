/*
 * Manu S. Madhav
 * 16-May-2014
 */
#include "ros/ros.h"
#include "ros/console.h"
#include "daq_interface/ReadEncoder.h"
#include "dome_common_msgs/Angle.h"
#include "dome_common_msgs/DomeEvent.h"
#include "dome_common_msgs/Drift.h"

float driftValue_currentLap, driftValue_prevLap;

void quit_callback(const dome_common_msgs::DomeEvent::ConstPtr& quit)
{
    ros::shutdown();
}

void drift_handle_callback(const dome_common_msgs::Drift::ConstPtr& latestDrift)
{
    driftValue_currentLap = latestDrift->value;
    driftValue_prevLap = latestDrift->valuePrevLap;
    std::cout<<"Compensated drift in the last lap : "<<static_cast<int>(1000*(driftValue_currentLap-driftValue_prevLap))<<" mdeg"<<std::endl;
    std::cout<<"Compensated cumulative drift : "<<static_cast<int>(driftValue_currentLap)<< " deg"<<std::endl;
}

int main(int argc, char **argv)
{
    driftValue_currentLap = 0;
    driftValue_prevLap = 0;
    ros::init(argc, argv, "EncoderPublisher");
    ros::NodeHandle n;

    ros::Publisher encoder_pub = n.advertise<dome_common_msgs::Angle>("encoder_angle", 1000);
    ros::Publisher internal_encoder_pub = n.advertise<dome_common_msgs::Angle>("internal_encoder_angle", 1000);
    ros::ServiceClient client = n.serviceClient<daq_interface::ReadEncoder>("read_encoder",true);

    ros::Subscriber drift_sub = n.subscribe("drift", 1000, drift_handle_callback);
    
    daq_interface::ReadEncoder encoder,internalEncoder;
    encoder.request.mode = 1;
    internalEncoder.request.mode=2;
    ros::Subscriber quit_sub = n.subscribe("quit",1,quit_callback);

    ros::Rate loop_rate(100);

    dome_common_msgs::Angle ang;
    ang.theta = 0;

    while (ros::ok())
    {
        if (client.call(encoder))
        {
        ang.theta = encoder.response.theta - driftValue_currentLap;
        encoder_pub.publish(ang);
        }

        if (client.call(internalEncoder))
        {
        ang.theta = internalEncoder.response.theta;
        internal_encoder_pub.publish(ang);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}