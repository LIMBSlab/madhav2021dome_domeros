/*
 * Node to create "reg-sync" pulses at random intervals, 
 * but with one pulse of width <width> every <interval> seconds.
 *
 * Manu S. Madhav
 * 03-Oct-2014
 */
#include "ros/ros.h"
#include "ros/console.h"
#include "dome_common_msgs/DomeEvent.h"
#include "daq_interface/WriteDigital.h"
#include <cstdlib>

using namespace std;

double interval = 10;
double width = 1;
double randInterval;
ros::Publisher event_pub;
ros::ServiceClient sync_client; 
daq_interface::WriteDigital sync_obj;
dome_common_msgs::DomeEvent event;

void quit_callback(const dome_common_msgs::DomeEvent::ConstPtr& quit)
{
    ros::shutdown();
}

void syncCallback(const ros::TimerEvent&)
{
    randInterval = ((double) rand())/((double) RAND_MAX) * (interval-width);
    ros::Duration(randInterval).sleep();
    
    event.value = 1;
    event_pub.publish(event);
    
    sync_obj.request.value = 1;
    sync_client.call(sync_obj);
    ros::Duration(width).sleep();
    sync_obj.request.value = 0;
    sync_client.call(sync_obj);

    event.value = 0;
    event_pub.publish(event);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "SyncSender");
    if (!ros::Time::isValid()) ros::Time::init();
    ros::NodeHandle n;

    event_pub = n.advertise<dome_common_msgs::DomeEvent>("dome_event", 1000);
    event.name = "reg_sync";
    ros::Subscriber quit_sub = n.subscribe("quit",1,quit_callback);

    sync_client = n.serviceClient<daq_interface::WriteDigital>("write_digital",true);

    int regSyncPin = 3;
    if (ros::param::has("/regSyncPin")) ros::param::get("/regSyncPin",regSyncPin);
    sync_obj.request.channel = regSyncPin;

    // Publisher is somehow not ready immediately, and so the following line. Is a race condition and is thus undesirable.
    ros::Duration(1).sleep();

    ros::Timer sync_timer = n.createTimer(ros::Duration(interval), syncCallback);
    ros::spin();

    return 0;
}
