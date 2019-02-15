#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("image", 5);

	// Parse command line arguments
	if (argc != 2)
	{
		printf("\nCannot start - not enough commnand line arguments.\n");
		printf("Number of command line arguments detected: %i\n", argc);
		exit(EXIT_FAILURE);
	}
	printf("\nYour image grabber is running\nClose the Terminal window when done.\n");

	cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

	ros::Rate loop_rate(1);
	while (nh.ok()) {
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
}