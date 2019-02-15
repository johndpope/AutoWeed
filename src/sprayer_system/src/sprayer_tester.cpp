#include "ros/ros.h"
#include "std_msgs/String.h"
#include "custom_messages/SprayerInfo.h"
#include <sstream>


int main(int argc, char **argv)
{

    ros::init(argc, argv, "sprayer_message_tester");
    ros::NodeHandle nh;
    ros::Publisher sprayer_publisher = nh.advertise<custom_messages::SprayerInfo>("sprayers", 100);

    ros::Rate loop_rate(10);
    /**
    * A count of how many messages we have sent. This is used to create
    * a unique string for each message.
    */
    //int count = 0;



    while (ros::ok()) {
        ROS_INFO("Input: s<sprayers>,<msec>e [enter](ex: s3,1000e [enter]): ctrl-c 'e' [enter] to quit");

        int bufferIndex = 0;
        char rxChar = '0';
        int intsScanned = 0;
        char rxBuff[128];
        int sprayersVal = 0;
        int sprayTime = 0;
        bool processBuffer = false;

        int c = getchar();   // call your non-blocking input function
        bufferIndex = 0;
        while (c != 'e'){
            if(c != 's'){
                rxBuff[bufferIndex] = c;
                ++bufferIndex;
                processBuffer = true;
            } else {
                break;
            }

            c = getchar();
        }

        if(processBuffer){
            processBuffer = false;
            rxBuff[bufferIndex] = 0;

            intsScanned = sscanf(rxBuff, "%d,%d", &sprayersVal, &sprayTime);

            custom_messages::SprayerInfo sprayer_msg;
            sprayer_msg.sprayerIDs = sprayersVal;
            sprayer_msg.sprayerTime = sprayTime;
            sprayer_publisher.publish(sprayer_msg);

            ROS_INFO("SprayId: %d, SprayTime: %d", sprayer_msg.sprayerIDs, sprayer_msg.sprayerTime);

        }

        loop_rate.sleep();

    }

    return 0;
}
