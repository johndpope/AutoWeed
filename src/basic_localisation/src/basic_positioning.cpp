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
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <nav_msgs/Odometry.h>

#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Constants.hpp>


// POSE MAPPING VARIABLES
/*************************/
bool originReset = true;
double previousPoseX = 0.0;
double previousPoseY = 0.0;
double currentPoseX = 0.0;
double currentPoseY = 0.0;
double previousLatitude = 0.0;
double previousLongitude = 0.0;
double currentLatitude = 0.0;
double currentLongitude = 0.0;
double currentOriginLatitude = 0.0;
double currentOriginLongitude = 0.0;
double maxSpeed = 28.0; // m/s ~= 100 km/hr

// ODOMETRY VARIABLES
/*********************/
nav_msgs::Odometry odom_msg;


bool receivedOdom = false;
bool receivedTwist = false;
bool receivedImu = false;


GeographicLib::Geodesic geod(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());




void positionCallback(const sensor_msgs::NavSatFixConstPtr& msg){

    // Update GPS Coordinates
    /*************************/
    previousLatitude = currentLatitude;
    previousLongitude = currentLongitude;
    currentLatitude = msg->latitude;
    currentLongitude = msg->longitude;

    // ROS_INFO("Previous Lat : %f", previousLatitude);
    // ROS_INFO("Previous Long: %f", previousLongitude);
    // ROS_INFO("Current  Lat : %f", currentLatitude);
    // ROS_INFO("Current  Long: %f", currentLongitude);


    // TODO: GPS updating "jump" mitigation HERE
    // http://wiki.ros.org/roscpp/Overview/Time
    // http://docs.ros.org/latest/api/rostime/html/classros_1_1TimeBase.html
    // http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html

    // POSE MAPPING
    /***************/
    /* This mapping has 3 functions:
        DONE - Calculate change in x and y position from GPS coordinates
        DONE - Outputting the current position
        TODO - Catching and mitigating jumps due to sudden GPS updates. */

    // Mapping current position to origin if the originReset flag is true.
    if (originReset) {
        // Reset the origin position
        currentOriginLatitude = currentLatitude;
        currentOriginLongitude = currentLongitude;
        currentPoseX = 0.0;
        currentPoseY = 0.0;
        originReset = false;

    } else {
        // Compute changes in X and Y
        /*****************************/
        /* Calculate the distance between pair of GPS Coordinates */

        double xDistance; // x distance "adjacent"
        double yDistance; // y distance "opposite"
        double zDistance;
        geod.Inverse(previousLatitude, previousLongitude, previousLatitude, currentLongitude, xDistance);
        geod.Inverse(previousLatitude, currentLongitude,currentLatitude, currentLongitude, yDistance);

        // Debug Testing Code
        /*********************/
        // Calculate difference in Longitude for X value
        // double xDistance_a; // x distance "adjacent"
        // double xDistance_b; // x distance "inverse adjacent"
        // Calculate X distance (Same Latitude, difference in Longitude)
        // geod.Inverse(lat1, lon1, lat2, lon2, s12);
        // ros::Time timeVar = ros::Time::now();
        // ROS_INFO("Start Time: X : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());
        // geod.Inverse(previousLatitude, previousLongitude, previousLatitude, currentLongitude, xDistance_a);
        // ROS_INFO("x_a: [%f]", xDistance_a*100);
        // timeVar = ros::Time::now();
        // ROS_INFO("Step  Time: X : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());
        // geod.Inverse(currentLatitude, previousLongitude, currentLatitude, currentLongitude, xDistance_b);
        // ROS_INFO("x_b: [%f]", xDistance_b*100);
        // timeVar = ros::Time::now();
        // ROS_INFO("Stop  Time: X : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());
        //
        // Calculate difference in Latitude for Y value
        // double yDistance_a; // y distance "opposite"
        // double yDistance_b; // y distance "inverse opposite"
        // Calculate Y distance (Same Longitude, difference in Latitude)
        // geod.Inverse(lat1, lon1, lat2, lon2, s12);
        // timeVar = ros::Time::now();
        // ROS_INFO("Start Time: Y : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());
        // geod.Inverse(previousLatitude, currentLongitude,currentLatitude, currentLongitude, yDistance_a);
        // ROS_INFO("y_a: [%f]", yDistance_a*100);
        // timeVar = ros::Time::now();
        // ROS_INFO("Step  Time: Y : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());
        // geod.Inverse(previousLatitude, previousLongitude, currentLatitude, previousLongitude, yDistance_b);
        // ROS_INFO("y_b: [%f]", yDistance_b*100);
        // timeVar = ros::Time::now();
        // ROS_INFO("Stop  Time: Y : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());
        //
        // Calculate Direct Distance
        // double directDistance;
        // timeVar = ros::Time::now();
        // ROS_INFO("Start Time: direct : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());
        // geod.Inverse(previousLatitude, previousLongitude, currentLatitude, currentLongitude, directDistance);
        // ROS_INFO("x_a: [%f]", directDistance*100);
        // timeVar = ros::Time::now();
        // ROS_INFO("Stop  Time: Y : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());

        // Update Pose
        /**************/
        if((currentLongitude - previousLongitude) >= 0.0){
            // Change in X is Positive
            // ROS_INFO("Positive X");
            currentPoseX += xDistance;
        } else {
            // Change in X is Negative
            // ROS_INFO("Negative X");
            currentPoseX -= xDistance;
        }

        if((currentLatitude - previousLatitude) >= 0.0){
            // Change in Y is Positive
            // ROS_INFO("Positive Y");
            currentPoseY += yDistance;
        } else {
            // Change in Y is Negative
            // ROS_INFO("Negative Y");
            currentPoseY -= yDistance;
        }
    }

    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.pose.pose.position.x = currentPoseX;
    odom_msg.pose.pose.position.y = currentPoseY;
    odom_msg.pose.pose.position.z = 0;

    receivedOdom = true;
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg){
    odom_msg.pose.pose.orientation.x = msg->orientation.x;
    odom_msg.pose.pose.orientation.y = msg->orientation.y;
    odom_msg.pose.pose.orientation.z = msg->orientation.z;
    odom_msg.pose.pose.orientation.w = msg->orientation.w;

    receivedImu = true;
}

void twistCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg){
    odom_msg.twist.twist.linear.x = msg->twist.twist.linear.x;
    odom_msg.twist.twist.linear.y = msg->twist.twist.linear.y;
    odom_msg.twist.twist.linear.z = msg->twist.twist.linear.z;
    odom_msg.twist.twist.angular.x = msg->twist.twist.angular.x;
    odom_msg.twist.twist.angular.y = msg->twist.twist.angular.y;
    odom_msg.twist.twist.angular.z = msg->twist.twist.angular.z;

    receivedTwist = true;
}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "basic_positioning_node");
    ros::NodeHandle nh;
    ros::Subscriber nav_subscriber = nh.subscribe("an_device/NavSatFix", 100, positionCallback);
    ros::Subscriber imu_sub = nh.subscribe("an_device/Imu", 100, imuCallback);
    ros::Subscriber twist_sub = nh.subscribe("an_device/Twist", 100, twistCallback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("Odometry", 10);

    odom_msg.header.stamp.sec = 0;
    odom_msg.header.stamp.nsec = 0;
    odom_msg.header.frame_id = "adnav_frame";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = 0;
    odom_msg.pose.pose.position.y = 0;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = 0;
    odom_msg.pose.pose.orientation.w = 0;
    odom_msg.pose.covariance = {  	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
    							  	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
    								0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
    								0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
    								0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
    								0.0,	0.0,	0.0,	0.0,	0.0,	0.0  }; // CURRENTLY UNUSED
    odom_msg.twist.twist.linear.x=0.0;
    odom_msg.twist.twist.linear.y=0.0;
    odom_msg.twist.twist.linear.z=0.0;
    odom_msg.twist.twist.angular.x=0.0;
    odom_msg.twist.twist.angular.y=0.0;
    odom_msg.twist.twist.angular.z=0.0;
    odom_msg.twist.covariance = {  	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
    							  	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
    								0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
    								0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
    								0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
    								0.0,	0.0,	0.0,	0.0,	0.0,	0.0  }; // CURRENTLY UNUSED

    // Testing Code
    /***************/
    // previousLatitude = -19.3295347143;
    // previousLongitude = 146.759635879;
    // currentLatitude = -19.3295348155;
    // currentLongitude = 146.759636526;
    //
    //
    // // Calculate difference in Longitude for X value
    // double xDistance_a; // x distance "adjacent"
    // double xDistance_b; // x distance "inverse adjacent"
    // // Calculate X distance (Same Latitude, difference in Longitude)
    // // geod.Inverse(lat1, lon1, lat2, lon2, s12);
    // ros::Time timeVar = ros::Time::now();
    // ROS_INFO("Start Time: X : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());
    // geod.Inverse(previousLatitude, previousLongitude, previousLatitude, currentLongitude, xDistance_a);
    // ROS_INFO("x_a: [%f]", xDistance_a);
    //
    // timeVar = ros::Time::now();
    // ROS_INFO("Step  Time: X : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());
    // geod.Inverse(currentLatitude, previousLongitude, currentLatitude, currentLongitude, xDistance_b);
    // ROS_INFO("x_b: [%f]", xDistance_b);
    //
    // if((currentLongitude - previousLongitude) >= 0.0){
    //     ROS_INFO("Positive X");
    // } else {
    //     ROS_INFO("Negative X");
    // }
    //
    // timeVar = ros::Time::now();
    // ROS_INFO("Stop  Time: X : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());
    //
    // // Calculate difference in Latitude for Y value
    // double yDistance_a; // y distance "opposite"
    // double yDistance_b; // y distance "inverse opposite"
    // // Calculate Y distance (Same Longitude, difference in Latitude)
    // // geod.Inverse(lat1, lon1, lat2, lon2, s12);
    // timeVar = ros::Time::now();
    // ROS_INFO("Start Time: Y : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());
    // geod.Inverse(previousLatitude, currentLongitude,currentLatitude, currentLongitude, yDistance_a);
    // ROS_INFO("y_a: [%f]", yDistance_a);
    //
    // timeVar = ros::Time::now();
    // ROS_INFO("Step  Time: Y : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());
    // geod.Inverse(previousLatitude, previousLongitude, currentLatitude, previousLongitude, yDistance_b);
    // ROS_INFO("y_b: [%f]", yDistance_b);
    //
    // if((currentLatitude - previousLatitude) >= 0.0){
    //     ROS_INFO("Positive Y");
    // } else {
    //     ROS_INFO("Negative Y");
    // }
    //
    // timeVar = ros::Time::now();
    // ROS_INFO("Stop  Time: Y : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());
    //
    // // Calculate Direct Distance
    // double directDistance;
    //
    // timeVar = ros::Time::now();
    // ROS_INFO("Start Time: direct : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());
    // geod.Inverse(previousLatitude, previousLongitude, currentLatitude, currentLongitude, directDistance);
    // ROS_INFO("x_a: [%f]", directDistance);
    // timeVar = ros::Time::now();
    // ROS_INFO("Stop  Time: Y : [%f]:[%lu]", timeVar.toSec(), timeVar.toNSec());

    while(ros::ok()){

        if(receivedOdom and receivedImu and receivedOdom){
            odom_pub.publish(odom_msg);
            receivedOdom = false;
            receivedTwist = false;
            receivedImu = false;
        }

        ros::spinOnce();
    }
    return 0;
}
