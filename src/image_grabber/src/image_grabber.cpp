#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <ros/package.h>

char resourceDirectory[256];
std::string TEST_IMAGES[4] = { "0d6773ff2.jpg", "1a6540288.jpg", "00f37e71d.jpg", "0b81fee2f.jpg" };

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("image", 4);

	// Parse command line arguments
	if (argc != 3)
	{
		printf("\nCannot start - not enough commnand line arguments.\n");
		printf("Number of command line arguments detected: %i\n", argc);
		exit(EXIT_FAILURE);
	}
	printf("\nYour image grabber is running\nClose the Terminal window when done.\n");

	// Location of resource files
	int cameraID = atoi(argv[1]);
	std::string packagePath = ros::package::getPath("image_grabber");
	sprintf(resourceDirectory, "%s/resources/", packagePath.c_str());
	char imageFilePath[256];
	sprintf(imageFilePath, "%s/%s", resourceDirectory, TEST_IMAGES[cameraID].c_str());

	cv::Mat image = cv::imread(imageFilePath, CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	int fps = atoi(argv[2]);
	ros::Rate loop_rate(fps);

	while (nh.ok()) {
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
}