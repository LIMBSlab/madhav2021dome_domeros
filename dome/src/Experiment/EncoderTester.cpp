/* EncoderTester
 * Code to simulate encoder_angle events at a constant velocity, for testing purposes
 *
 * Manu S. Madhav
 * 04-Feb-2015
 */
#include "ros/ros.h"
#include "ros/console.h"
#include "dome_common_msgs/Angle.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EncoderTester");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    ros::Publisher modulation_pub = n.advertise<dome_common_msgs::Angle>("encoder_angle", 1000);

    ros::Rate loop_rate(100);
    dome_common_msgs::Angle ang;
    ang.theta = 0;

    double vel = 30; // degrees / sec
    nh.getParam("vel",vel);
    double now,start;
    start = ros::Time::now().toSec();

    while (ros::ok())
    {
        now = ros::Time::now().toSec();
        
        ang.theta = vel*(now-start);
        modulation_pub.publish(ang);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
