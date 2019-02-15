#include "ros/ros.h"
#include "std_msgs/String.h"
#include <fstream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

void odomFilteredCallback(const nav_msgs::OdometryConstPtr& msg)
{
    std::fstream fs;
    fs.open ("/home/jakey/autoweed_mannedsystem/logs/odomFiltered.csv", std::fstream::in | std::fstream::out | std::fstream::app);

    fs << msg->header.seq << ","
        << msg->header.stamp.sec << ","
        << msg->header.stamp.nsec << ","
        << msg->pose.pose.position.x << ","
        << msg->pose.pose.position.y << ","
        << msg->pose.pose.position.z << ","
        << msg->pose.pose.orientation.x << ","
        << msg->pose.pose.orientation.y << ","
        << msg->pose.pose.orientation.z << ","
        << msg->pose.pose.orientation.w << ",";

    for(int i = 0; i < 35; ++i){
        fs << msg->pose.covariance[i] << ",";
    }

    fs << msg->pose.covariance[35] << ",";

    fs << msg->twist.twist.linear.x << ","
        << msg->twist.twist.linear.y << ","
        << msg->twist.twist.linear.z << ","
        << msg->twist.twist.angular.x << ","
        << msg->twist.twist.angular.y << ","
        << msg->twist.twist.angular.z << ",";

    for(int i = 0; i < 35; ++i){
        fs << msg->twist.covariance[i] << ",";
    }

    fs << msg->twist.covariance[35] << "\n";

    fs.close();
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    std::fstream fs;
    fs.open ("/home/jakey/autoweed_mannedsystem/logs/odom.csv", std::fstream::in | std::fstream::out | std::fstream::app);

    fs << msg->header.seq << ","
        << msg->header.stamp.sec << ","
        << msg->header.stamp.nsec << ","
        << msg->pose.pose.position.x << ","
        << msg->pose.pose.position.y << ","
        << msg->pose.pose.position.z << ","
        << msg->pose.pose.orientation.x << ","
        << msg->pose.pose.orientation.y << ","
        << msg->pose.pose.orientation.z << ","
        << msg->pose.pose.orientation.w << ",";

    // for(int i = 0; i < 35; ++i){
    //     fs << msg->pose.covariance[i] << ",";
    // }
    //
    // fs << msg->pose.covariance[35] << ",";

    fs << msg->twist.twist.linear.x << ","
        << msg->twist.twist.linear.y << ","
        << msg->twist.twist.linear.z << ","
        << msg->twist.twist.angular.x << ","
        << msg->twist.twist.angular.y << ","
        << msg->twist.twist.angular.z << "\n";

    // fs << msg->twist.twist.linear.x << ","
    //     << msg->twist.twist.linear.y << ","
    //     << msg->twist.twist.linear.z << ","
    //     << msg->twist.twist.angular.x << ","
    //     << msg->twist.twist.angular.y << ","
    //     << msg->twist.twist.angular.z << ",";
    // for(int i = 0; i < 35; ++i){
    //     fs << msg->twist.covariance[i] << ",";
    // }
    //
    // fs << msg->twist.covariance[35] << "\n";

    fs.close();
}

void odom2Callback(const nav_msgs::OdometryConstPtr& msg)
{
    std::fstream fs;
    fs.open ("/home/jakey/autoweed_mannedsystem/logs/odom2.csv", std::fstream::in | std::fstream::out | std::fstream::app);

    fs << msg->header.seq << ","
        << msg->header.stamp.sec << ","
        << msg->header.stamp.nsec << ","
        << msg->pose.pose.position.x << ","
        << msg->pose.pose.position.y << ","
        << msg->pose.pose.position.z << "\n";
        // << msg->pose.pose.orientation.x << ","
        // << msg->pose.pose.orientation.y << ","
        // << msg->pose.pose.orientation.z << ","
        // << msg->pose.pose.orientation.w << ",";

    // for(int i = 0; i < 35; ++i){
    //     fs << msg->pose.covariance[i] << ",";
    // }
    //
    // fs << msg->pose.covariance[35] << ",";

    // fs << msg->twist.twist.linear.x << ","
    //     << msg->twist.twist.linear.y << ","
    //     << msg->twist.twist.linear.z << ","
    //     << msg->twist.twist.angular.x << ","
    //     << msg->twist.twist.angular.y << ","
    //     << msg->twist.twist.angular.z << "\n";

    // fs << msg->twist.twist.linear.x << ","
    //     << msg->twist.twist.linear.y << ","
    //     << msg->twist.twist.linear.z << ","
    //     << msg->twist.twist.angular.x << ","
    //     << msg->twist.twist.angular.y << ","
    //     << msg->twist.twist.angular.z << ",";
    // for(int i = 0; i < 35; ++i){
    //     fs << msg->twist.covariance[i] << ",";
    // }
    //
    // fs << msg->twist.covariance[35] << "\n";

    fs.close();
}

void twistCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg)
{
    std::fstream fs;
    fs.open ("/home/jakey/autoweed_mannedsystem/logs/twist.csv", std::fstream::in | std::fstream::out | std::fstream::app);

    fs << msg->header.seq << ","
        << msg->header.stamp.sec << ","
        << msg->header.stamp.nsec << ",";

    fs << msg->twist.twist.linear.x << ","
        << msg->twist.twist.linear.y << ","
        << msg->twist.twist.linear.z << ","
        << msg->twist.twist.angular.x << ","
        << msg->twist.twist.angular.y << ","
        << msg->twist.twist.angular.z << ",";

    for(int i = 0; i < 35; ++i){
        fs << msg->twist.covariance[i] << ",";
    }

    fs << msg->twist.covariance[35] << "\n";

    fs.close();
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
    std::fstream fs;
    fs.open ("/home/jakey/autoweed_mannedsystem/logs/imu.csv", std::fstream::in | std::fstream::out | std::fstream::app);

    fs << msg->header.seq << ","
        << msg->header.stamp.sec << ","
        << msg->header.stamp.nsec << ",";

    fs << msg->orientation.x << ","
        << msg->orientation.y << ","
        << msg->orientation.z << ","
        << msg->orientation.w;

    for(int i=0; i < 7; ++i){
        fs << msg->orientation_covariance[i] << ",";
    }

    fs << msg->orientation_covariance[8] << ",";

    fs << msg->angular_velocity.x << ","
        << msg->angular_velocity.y << ","
        << msg->angular_velocity.z << ",";

    for(int i=0; i < 7; ++i){
        fs << msg->angular_velocity_covariance[i] << ",";
    }

    fs << msg->angular_velocity_covariance[8] << ",";

    fs << msg->linear_acceleration.x << ","
        << msg->linear_acceleration.y << ","
        << msg->linear_acceleration.z << ",";

    for(int i=0; i < 7; ++i){
        fs << msg->linear_acceleration_covariance[i] << ",";
    }

    fs << msg->linear_acceleration_covariance[8] << "\n";

    fs.close();
}

void navSatCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
    std::fstream fs;
    fs.open ("/home/jakey/autoweed_mannedsystem/logs/NavSatFix.csv", std::fstream::in | std::fstream::out | std::fstream::app);


    fs << msg->header.seq << ","
        << msg->header.stamp.sec << ","
        << msg->header.stamp.nsec << ",";

    fs << msg->latitude << ","
        << msg->longitude << ","
        << msg->altitude;

    for(int i=0; i < 7; ++i){
        fs << msg->position_covariance[i] << ",";
    }

    fs << msg->position_covariance[8] << "\n";

    fs.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "logger");
  ros::NodeHandle n;

  ros::Subscriber odom_filtered = n.subscribe("odometry/filtered", 1000, odomFilteredCallback);
  ros::Subscriber adnav_imu = n.subscribe("an_device/Imu", 1000, imuCallback);
  ros::Subscriber adnav_twist = n.subscribe("an_device/Twist", 1000, twistCallback);
  ros::Subscriber adnav_odom = n.subscribe("an_device/Odom", 1000, odomCallback);
  ros::Subscriber adnav_odom2 = n.subscribe("an_device/Odom2", 1000, odom2Callback);
  ros::Subscriber adnav_navsat = n.subscribe("an_device/NavSatFix", 1000, navSatCallback);

  std::fstream fs;

  // odomFiltered File Header
  fs.open ("/home/jakey/autoweed_mannedsystem/logs/odomFiltered.csv", std::fstream::in | std::fstream::out | std::fstream::app);
  fs << "seq,sec,nsec,x,y,z,Qx,Qy,Qz,Qw,pCov0,pCov1,pCov2,pCov3,pCov4,pCov5,pCov6,pCov7,pCov8,pCov9,pCov10,pCov11,pCov12,pCov13,pCov14,pCov15,pCov16,pCov17,pCov18,pCov19,pCov20,pCov21,pCov22,pCov23,pCov24,pCov25,pCov26,pCov27,pCov28,pCov29,pCov30,pCov31,pCov32,pCov33,pCov34,pCov35,linearx,lineary,linearz,angularx,angulary,angularz,tCov0,tCov1,tCov2,tCov3,tCov4,tCov5,tCov6,tCov7,tCov8,tCov9,tCov10,tCov11,tCov12,tCov13,tCov14,tCov15,tCov16,tCov17,tCov18,tCov19,tCov20,tCov21,tCov22,tCov23,tCov24,tCov25,tCov26,tCov27,tCov28,tCov29,tCov30,tCov31,tCov32,tCov33,tCov34,tCov35\n";
  fs.close();

  // imu File Header
  fs.open ("/home/jakey/autoweed_mannedsystem/logs/odom.csv", std::fstream::in | std::fstream::out | std::fstream::app);
  fs << "seq,sec,nsec,PoseX,PoseY,PoseZ,Qx,Qy,Qz,Qw,linearx,lineary,linearz,angularx,angulary,angularz\n";
  fs.close();

  // odom File Header
  fs.open ("/home/jakey/autoweed_mannedsystem/logs/odom2.csv", std::fstream::in | std::fstream::out | std::fstream::app);
  // fs << "seq,sec,nsec,x,y,z,Qx,Qy,Qz,Qw,pCov0,pCov1,pCov2,pCov3,pCov4,pCov5,pCov6,pCov7,pCov8,pCov9,pCov10,pCov11,pCov12,pCov13,pCov14,pCov15,pCov16,pCov17,pCov18,pCov19,pCov20,pCov21,pCov22,pCov23,pCov24,pCov25,pCov26,pCov27,pCov28,pCov29,pCov30,pCov31,pCov32,pCov33,pCov34,pCov35,linearx,lineary,linearz,angularx,angulary,angularz,tCov0,tCov1,tCov2,tCov3,tCov4,tCov5,tCov6,tCov7,tCov8,tCov9,tCov10,tCov11,tCov12,tCov13,tCov14,tCov15,tCov16,tCov17,tCov18,tCov19,tCov20,tCov21,tCov22,tCov23,tCov24,tCov25,tCov26,tCov27,tCov28,tCov29,tCov30,tCov31,tCov32,tCov33,tCov34,tCov35\n";
  fs << "seq,sec,nsec,PoseX,PoseY,PoseZ,Qx,Qy,Qz,Qw,linearx,lineary,linearz,angularx,angulary,angularz\n";
  fs.close();

  // twist File Header
  fs.open ("/home/jakey/autoweed_mannedsystem/logs/twist.csv", std::fstream::in | std::fstream::out | std::fstream::app);
  fs << "seq,sec,nsec,linearx,lineary,linearz,angularx,angulary,angularz,tCov0,tCov1,tCov2,tCov3,tCov4,tCov5,tCov6,tCov7,tCov8,tCov9,tCov10,tCov11,tCov12,tCov13,tCov14,tCov15,tCov16,tCov17,tCov18,tCov19,tCov20,tCov21,tCov22,tCov23,tCov24,tCov25,tCov26,tCov27,tCov28,tCov29,tCov30,tCov31,tCov32,tCov33,tCov34,tCov35\n";
  fs.close();

  // IMU File Header
  fs.open ("/home/jakey/autoweed_mannedsystem/logs/imu.csv", std::fstream::in | std::fstream::out | std::fstream::app);
  fs << "seq,sec,nsec,Qx,Qy,Qz,Qw,Qcov0,Qcov1,,Qcov2,Qcov3,Qcov4,Qcov5,Qcov6,Qcov7,Qcov8,angVx,angVy,angVz,angCov0,angCov1,angCov2,angCov3,angCov4,angCov5,angCov6,angCov7,angCov8,linearAx,linearAy,linearAz,lAcov0,lAcov1,lAcov2,lAcov3,lAcov4,lAcov5,lAcov6,lAcov7,lAcov8\n";
  fs.close();

  // NavSatFix File Header
  fs.open ("/home/jakey/autoweed_mannedsystem/logs/NavSatFix.csv", std::fstream::in | std::fstream::out | std::fstream::app);
  fs << "seq,sec,nsec,latitude,longitude,altitude,posCov0,posCov1,posCov2,posCov3,posCov4,posCov5,posCov6,posCov7,posCov8\n";
  fs.close();



  ros::spin();

  return 0;
}
