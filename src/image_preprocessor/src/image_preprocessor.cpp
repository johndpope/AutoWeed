#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <algorithm>
#include <cstdarg>
#include <unistd.h>
#include <cstdlib>
#include <list>
#include <sys/stat.h>
#include "custom_messages/ImageInfo.h"

using namespace cv;
using namespace std;

ros::Publisher imagePublisher;
int camera_id;
bool display_enabled;
bool save;
int fps;
int imageHeight = 300;
int imageWidth = 480;
char runDirectory[128];

uint8_t INPUT_H = 224;
uint8_t INPUT_W = 224;

// Frame counters
int frame_counts[] = {0, 0, 0, 0};

void imageCallback(const sensor_msgs::ImageConstPtr& img)
{
    try
    {
        // Extract ROS timestamp
        ros::Time timestamp = ros::Time::now(); // Don't have the camera timestamp because we are emulating images from file

        // Extract image
        Mat image = cv_bridge::toCvShare(img, "bgr8")->image;
        
        // Increment specific frame count
        frame_counts[camera_id]++;
        
        // Save image
        if (save) {
            if (frame_counts[camera_id] % (24 / fps) == 0) {
                // Save every other image according to desired frame rate
                // Note 24fps is max frame rate for low res cameras
                char filename[256];
                sprintf(filename, "%s/Frame%06dCamera%d.jpg", runDirectory, frame_counts[camera_id], camera_id);
                imwrite(filename, image);
            }
        }

		// Downsample to 224x224 with nearest neighbout
		cv::resize(image, image, cv::Size(INPUT_H, INPUT_W), 0, 0, cv::INTER_NEAREST);
		// Convert from HWC to CHW format
		cv::Size size = image.size();
		cv::Size newsize(size.width, size.height * 3);
		cv::Mat destination(newsize, CV_8U);
		for (int i = 0; i < image.channels(); ++i)
		{
			cv::extractChannel(
				image,
				cv::Mat(
					size.height,
					size.width,
					CV_8U,
					&(destination.at<uint8_t>(size.height*size.width*i))),
				2 - i); // BGR 2 RGB
		}
		// Convert to uint8_t data array
		int size_ = destination.total() * destination.elemSize();

		// Generate ImageInfo message
		custom_messages::ImageInfo imageInfo;
		imageInfo.header.stamp = timestamp;
		imageInfo.cameraID = camera_id;
		imageInfo.frameCount = frame_counts[camera_id];
		memcpy(&imageInfo.data, destination.data, size_ * sizeof(uint8_t));

		// Publish preprocessed image data
		imagePublisher.publish(imageInfo);

		// Display image
        if (display_enabled) {
            Mat resizedImage;
            resize(image, resizedImage, Size(), 0.5, 0.5, cv::INTER_NEAREST);
            char windowName [50];
            sprintf(windowName, "AutoWeed-%i", camera_id);
            cv::imshow(windowName, resizedImage);
            cv::waitKey(1);
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_preprocessor");
    ros::NodeHandle nh;

    // Parse command line arguments
    if(argc != 6)
    {
        printf("\nCannot start - not enough commnand line arguments. \nUsage: rosrun image_processor image_processor {target_type} {camera_id}. \nTry: rosrun image_processor image_processor 0 0\n");
        printf("Number of command line arguments detected: %i\n",argc);
        exit(EXIT_FAILURE);
    }
    printf("\nYour image preprocessor is running\nClose the Terminal window when done.\n");

    // Set up target type and camera id
    camera_id = atoi(argv[1]);
    display_enabled = atoi(argv[2]) == 1 ? true : false;
    fps = atoi(argv[3]);
    save = atoi(argv[4]) == 1 ? true : false;
    
    // Create directory
    if (save) {
        sprintf(runDirectory, "/home/nvidia/AutoWeed/%s/", argv[5]);
        const int dir_err = mkdir(runDirectory, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    ROS_INFO("You selected camera ID %d", camera_id);

    char windowName [50];
    sprintf(windowName, "AutoWeed-%i", camera_id);
    if (display_enabled) {
        cv::namedWindow(windowName);
        cv::startWindowThread();
    }

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);
	imagePublisher = nh.advertise<custom_messages::ImageInfo>("/preprocessed_images", 1);

    while(ros::ok()){

        ros::spinOnce();

    }


    if (display_enabled) {
        cv::destroyWindow(windowName);
    }
}
