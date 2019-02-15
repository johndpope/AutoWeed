#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <nav_msgs/Odometry.h>

#include "custom_messages/TargetInfo.h"
#include "custom_messages/SprayerInfo.h"


#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Constants.hpp>

using namespace GeographicLib;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh;

    ros::Publisher sprayer_publisher = nh.advertise<custom_messages::SprayerInfo>("sprayers", 100);


    ros::Rate loop_rate(1000);




    custom_messages::SprayerInfo sprayer_msg;

    // double lat1 = -19.3295030895;
    // double lon1 = 146.759567896;
    // double lat2 = -19.3295160463;
    // double lon2 = 146.759566044;
    // double s12;
    //
    // GeographicLib::Geodesic geod(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    //
    // geod.Inverse(lat1, lon1, lat1, lon2, s12);
    // ROS_INFO("SW Direction: %f", s12);
    // geod.Inverse(lat1, lon1, lat2, lon1, s12);
    // ROS_INFO("NE Direction: %f", s12);
    // while(ros::ok()){
    //     bool targetToBeSprayed = false;
    //     bool publishSprayerMsg = false;
    //
    //     std::list<custom_messages::TargetInfo> targetInfoList;
    //
    //     custom_messages::TargetInfo target1;
    //     target1.targetId = 1;
    //     custom_messages::TargetInfo target2;
    //     target2.targetId = 2;
    //     custom_messages::TargetInfo target3;
    //     target3.targetId = 3;
    //
    //     targetInfoList.push_back(target1);
    //     targetInfoList.push_back(target2);
    //     targetInfoList.push_back(target3);
    //
    //     for (std::list<custom_messages::TargetInfo>::iterator target=targetInfoList.begin(); target != targetInfoList.end(); ){
    //
    //         ROS_INFO("Target: %d", (*target).targetId);
    //
    //         bool targetToBeSprayed = false;
    //
    //         if((*target).targetId == 2){
    //
    //             sprayer_msg.targetInfo = *target;
    //
    //             publishSprayerMsg = true;
    //             targetToBeSprayed = true;
    //
    //             ROS_INFO("FINISHED single spray check");
    //         }
    //
    //         if(targetToBeSprayed){
    //             // If target to be sprayed, remove from the list
    //             target = targetInfoList.erase(target);
    //             ROS_INFO("Removed target");
    //         } else {
    //             ++target;
    //         }
    //     }
    //
    //     if(publishSprayerMsg) {
    //         ROS_INFO("About to transmit");
    //         sprayer_publisher.publish(sprayer_msg);
    //         publishSprayerMsg = false;
    //         ROS_INFO("Published Spray Message");
    //     }
    //     loop_rate.sleep();
    // }

    ROS_INFO("Camera 1: %d", ('1' - '0'));

    ros::spin();

    return 0;
}
