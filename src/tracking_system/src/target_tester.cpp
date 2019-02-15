#include "ros/ros.h"
#include "std_msgs/String.h"
#include "custom_messages/TargetInfo.h"
#include "custom_messages/SprayerInfo.h"
#include "sensor_msgs/NavSatFix.h"
#include <nav_msgs/Odometry.h>
#include <sstream>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

/*
 * Get a character from the terminal without blocking
 */
// int getch()
// {
//   static struct termios oldt, newt;
//   tcgetattr( STDIN_FILENO, &oldt);           // save old settings
//   newt = oldt;
//   newt.c_lflag &= ~(ICANON);                 // disable buffering
//   tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
//
//   int c = getchar();  // read character (non-blocking)
//
//   tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
//   return c;
// }

sensor_msgs::NavSatFix currentNavSatFix;
nav_msgs::Odometry currentOdom;
/*
 * Updates the local position variables
 */
void navSatCallback(const sensor_msgs::NavSatFixConstPtr& msg){
    currentNavSatFix = *msg;
}

void positionCallback(const nav_msgs::OdometryConstPtr& msg){
    currentOdom = *msg;
}

uint16_t targetCountId = 0;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
    ros::init(argc, argv, "target_identification");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle nh;

    /**
    * The advertise() function is how you tell ROS that you want to
    * publish on a given topic name. This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing. After this advertise() call is made, the master
    * node will notify anyone who is trying to subscribe to this topic name,
    * and they will in turn negotiate a peer-to-peer connection with this
    * node.  advertise() returns a Publisher object which allows you to
    * publish messages on that topic through a call to publish().  Once
    * all copies of the returned Publisher object are destroyed, the topic
    * will be automatically unadvertised.
    *
    * The second parameter to advertise() is the size of the message queue
    * used for publishing messages.  If messages are published more quickly
    * than we can send them, the number here specifies how many messages to
    * buffer up before throwing some away.
    */
    ros::Subscriber sub = nh.subscribe("an_device/NavSatFix", 10, navSatCallback);
    ros::Subscriber Odom_sub = nh.subscribe("an_device/Odom", 100, positionCallback);
    ros::Publisher target_publisher = nh.advertise<custom_messages::TargetInfo>("targets", 100);

    ros::Rate loop_rate(10);

    int bufferIndex = 0;
    char rxChar = '0';
    int intsScanned = 0;
    char rxBuff[128];
    char camChar;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    std::string cameras[4] = {"camera_1_frame", "camera_2_frame", "camera_3_frame", "camera_4_frame"};

    while (ros::ok()) {
        ros::spinOnce();

        ROS_INFO("Enter: s1e or s2e or s3e or s4e: ctrl-c 'e' [enter] to quit");

        int c = getchar();   // call your non-blocking input function
        bufferIndex = 0;
        while (c != 'e'){
            if(c != 's'){
                camChar = c;
                // rxBuff[bufferIndex] = c;
                // ++bufferIndex;
            }
            c = getchar();
        }
        // Uncomment if statement to make it listen for t only
        //if (c == 't'){
        if(camChar == '1' | camChar == '2' | camChar == '3' | camChar == '4'){

            int camIndex = (camChar - '0') - 1;

            geometry_msgs::TransformStamped cameraTransform;

            try{
                cameraTransform = tfBuffer.lookupTransform(cameras[camIndex], "odom", ros::Time(0), ros::Duration(1.0));
                //sprayerPosition[i] = tfBuffer.lookupTransform("odom", sprayerId[i], ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                //ros::Duration(0.05).sleep();
                continue;
            }

            // multiply 2 transforms together to get the result of both
            // https://answers.ros.org/question/289144/apply-transformation-matrix-to-existing-frame/


            custom_messages::TargetInfo newTarget;
            newTarget.header.stamp = ros::Time::now();
            newTarget.header.frame_id = "odom";
            newTarget.targetId = targetCountId;
            newTarget.cameraId = (camChar - '0');
            newTarget.xPosition = 0;
            newTarget.yPosition = 0;
            newTarget.cameraTransform = cameraTransform;
            newTarget.targetTransform = cameraTransform;
            newTarget.navSatFix = currentNavSatFix;
            newTarget.odom = currentOdom;
            newTarget.boxHeight = 100;
            newTarget.boxWidth = 100;
            newTarget.width = 0.2; // Meters
            newTarget.height = 0.2; // Meters

            target_publisher.publish(newTarget);
            ROS_INFO("Target Published");
            targetCountId++;
            c = 'b';
        }

        loop_rate.sleep();

    }

    return 0;
}
