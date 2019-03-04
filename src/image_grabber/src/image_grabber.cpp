#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <ros/package.h>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>

char resourceDirectory[256];
char imageDirectory[256];
char labelFilepath[256];
char imageFilepath[256];

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("image_raw", 4);

	// Parse command line arguments
	if (argc != 4)
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
	sprintf(imageDirectory, "%s", argv[3]);

	int fps = atoi(argv[2]);
	ros::Rate loop_rate(fps);

	// Read labels and filenames
	std::string line;
	sprintf(labelFilepath, "%s/labels.csv", imageDirectory);
	std::ifstream labelsFile(labelFilepath);
	std::vector<std::string> filenames;
	std::vector<int> labels;
	if (labelsFile.is_open())
	{
		getline(labelsFile, line); // Skip header
		while (getline(labelsFile, line))
		{
			int tokenCount = 0;
			size_t pos = 0;
			std::string token;
			while ((pos = line.find(",")) != std::string::npos)
			{
				token = line.substr(0, pos);
				if (tokenCount == 0) {
					filenames.push_back(token);
				}
				else if (tokenCount == 1)
					labels.push_back(0);//std::stoi(token));
				line.erase(0, pos + 1);
				tokenCount = tokenCount + 1;
			}
		}
		labelsFile.close();
	}
	printf("Read all %i labels.\n", filenames.size());

	int i = cameraID;
	while (nh.ok()) {

		if (i > filenames.size())
			break;

		sprintf(imageFilepath, "%s/%s", imageDirectory, filenames[i].c_str());
		ROS_INFO("Loading image: %s", imageFilepath);
		cv::Mat image = cv::imread(imageFilepath, CV_LOAD_IMAGE_COLOR);
		cv::imshow("test", image);
		cv::waitKey(1);
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

		i = i + 4;
	}
}