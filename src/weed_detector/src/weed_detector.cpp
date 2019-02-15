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
#include "custom_messages/TargetInfo.h"
#include <list>
#include <sys/stat.h>
#include "Target.h"

using namespace cv;
using namespace std;

int camera_id;
bool find_targets;
bool display_enabled;
bool save;
int fps;
int imageHeight = 300;
int imageWidth = 480;
int targetSize = 120; // Square size threshold
char runDirectory[128];

// Frame counters
int frame_counts[] = {0, 0, 0, 0};

bool publishTargets = false;
std::list<custom_messages::TargetInfo> targetInfoList;

void imageCallback(const sensor_msgs::ImageConstPtr& img)
{
    try
    {
        // Extract ROS timestamp
        ros::Time timestamp;
        //timestamp = ci->header.stamp;

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

        // Find targets?
        if (find_targets)
        {
            // Colour/contrast normalisation
            Mat bgr[3];
            split(image, bgr);
            Ptr<CLAHE> clahe = createCLAHE();
            clahe->setClipLimit(2.0);
            clahe->setTilesGridSize(Size(8,8));
            clahe->apply(bgr[0], bgr[0]);
            clahe->apply(bgr[1], bgr[1]);
            clahe->apply(bgr[2], bgr[2]);
            merge(bgr, 3, image);

            // Convert to HSV
            Mat hsv;
            cvtColor(image, hsv, COLOR_BGR2HSV);

            // Threshold yellow hue
            Mat binary;
            inRange(hsv, Scalar(5,0,0), Scalar(30,255,255), binary);

            // Morphologically select small squares
            morphologyEx(binary, binary, CV_MOP_OPEN, getStructuringElement(MORPH_RECT, Size(5, 5)));
            morphologyEx(binary, binary, CV_MOP_CLOSE, getStructuringElement(MORPH_RECT, Size(5, 5)));

            // Find contours
            vector< vector<Point> > contours;
            vector<Vec4i> heirarchy;
            findContours(binary, contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

            // Approximate contours to polygons and get bounding rectangles
            vector< vector<Point> > contours_poly(contours.size());
            vector<Rect> boundRect(contours.size());
            for (int i = 0; i < contours.size(); i++) {
                cv::approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
                boundRect[i] = boundingRect(Mat(contours_poly[i]));
            }

            // Create targets based on bounding rectangle size
            unsigned int x, y, height, width;
            vector<Target> t;
            int targetCount = 0;
            int maxTargetsPerImage = imageHeight / targetSize * imageWidth / targetSize;
            t.reserve(maxTargetsPerImage);

			/*
            for (int i = 0; i < contours.size(); i++) {
                Rect rect = boundRect[i];
                x = rect.x + rect.width / 2;
                y = rect.y + rect.height / 2;
                height = rect.height;
                width = rect.width;
				
                // If area > 100x100 square pixels, large enough to be target
                if (height * width >= targetSize*targetSize) {
                    //Target target(x, y, height, width, timestamp, camera_id, imageHeight, imageWidth);


                    // Generate TargetInfo Message
                    custom_messages::TargetInfo newTarget;
                    //newTarget.header.stamp = timestamp;
                    newTarget.targetId = targetCount;
                    newTarget.cameraId = camera_id + 1; // Map from IDs 0-3 to IDs 1-4
                    newTarget.xPixelLocation = x;
                    newTarget.yPixelLocation = y;
                    newTarget.boxHeight = height;
                    newTarget.boxWidth = width;



                    printf("%i by %i target found at (%i,%i) from camera %i\r\n",target.width,target.height,target.x,target.y, camera_id);
                    t[targetCount] = target;
                    targetCount++;


                    // Add Target to target list
                    targetInfoList.push_back(newTarget);

                    publishTargets = true;

                }
            }
			*/
            // Draw bound rectangles over each target
            for( int i = 0; i< targetCount; i++ )
            {
                Scalar color = Scalar(0, 0, 255);
                int tlx = t[i].x - t[i].width / 2;
                int tly = t[i].y - t[i].height / 2;
                int brx = t[i].x + t[i].width / 2;
                int bry = t[i].y + t[i].width / 2;
                Point tl(tlx,tly);
                Point br(brx,bry);
                rectangle(image, tl, br, color, 2, 8, 0 );
            }
        }

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
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;

    // Parse command line arguments
    if(argc != 7)
    {
        printf("\nCannot start - not enough commnand line arguments. \nUsage: rosrun image_processor image_processor {target_type} {camera_id}. \nTry: rosrun image_processor image_processor 0 0\n");
        printf("Number of command line arguments detected: %i\n",argc);
        exit(EXIT_FAILURE);
    }
    printf("\nYour image processor is running\nClose the Terminal window when done.\n");

    // Set up target type and camera id
    camera_id = atoi(argv[1]);
    find_targets = atoi(argv[2]) == 1 ? true : false;
    display_enabled = atoi(argv[3]) == 1 ? true : false;
    fps = atoi(argv[4]);
    save = atoi(argv[5]) == 1 ? true : false;
    
    // Create directory
    if (save) {
        sprintf(runDirectory, "/home/nvidia/AutoWeed/%s/", argv[6]);
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

    ros::Publisher target_publisher = nh.advertise<custom_messages::TargetInfo>("/targets", 100);

    while(ros::ok()){

        ros::spinOnce();

        // If Targets are found publish them
        if(publishTargets){
            // Publish each Target to the targets topic
            for (std::list<custom_messages::TargetInfo>::iterator target=targetInfoList.begin(); target != targetInfoList.end(); ){
                target_publisher.publish(*target);
                target = targetInfoList.erase(target);
            }
            publishTargets = false;
        }
    }


    if (display_enabled) {
        cv::destroyWindow(windowName);
    }
}
