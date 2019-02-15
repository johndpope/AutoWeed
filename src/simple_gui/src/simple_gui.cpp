#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

void imageCallback0(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        imshow("Camera0", cv_bridge::toCvShare(msg, "bgr8")->image);
        waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        imshow("Camera1", cv_bridge::toCvShare(msg, "bgr8")->image);
        waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        imshow("Camera2", cv_bridge::toCvShare(msg, "bgr8")->image);
        waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void imageCallback3(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        imshow("Camera3", cv_bridge::toCvShare(msg, "bgr8")->image);
        waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_gui");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    
    namedWindow("Camera0");
    namedWindow("Camera1");
    namedWindow("Camera2");
    namedWindow("Camera3");
    startWindowThread();
    
    image_transport::Subscriber sub0 = it.subscribe("/Camera0/processed_image", 1, imageCallback0);
    image_transport::Subscriber sub1 = it.subscribe("/Camera1/processed_image", 1, imageCallback1);
    image_transport::Subscriber sub2 = it.subscribe("/Camera2/processed_image", 1, imageCallback2);
    image_transport::Subscriber sub3 = it.subscribe("/Camera3/processed_image", 1, imageCallback3);
    
    ros::spin();
    
    destroyWindow("Camera0");
    destroyWindow("Camera1");
    destroyWindow("Camera2");
    destroyWindow("Camera3");
}