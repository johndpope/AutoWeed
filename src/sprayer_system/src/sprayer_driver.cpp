#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <string>
#include "custom_messages/SprayerInfo.h"
#include <sstream>

#include "rs232/rs232.h"

bool engageSprayer = false;
uint16_t sprayerIDs;
uint16_t sprayTime;


void sprayerCallback(const custom_messages::SprayerInfoConstPtr& msg){
    sprayerIDs = msg->sprayerIDs;
    sprayTime = msg->sprayerTime;
    engageSprayer = true;
    ROS_INFO("Received Sprayer Message to Spray");
}

// std::string manual_to_string(uint16_t n){
//     std::ostringstream stm ;
//     stm << n ;
//     return stm.str();
// }


template <typename T>
std::string to_string(T value)
{
	std::ostringstream os ;
	os << value ;
	return os.str() ;
}



int main(int argc, char *argv[]) {

	// Set up ROS node //
	/*******************/
	ros::init(argc, argv, "sprayer_driver_node");
	ros::NodeHandle nh;

	if(argc != 3)
	{
		printf("\nCannot start - not enough commnand line arguments. \nUsage: rosrun sprayer_system sprayer_driver_core {port} {baud rate}. \nTry: rosrun sprayer_system sprayer_driver_core /dev/ttyUSB0 115200\n");
		printf("Number of command line arguments detected: %i\n",argc);
		exit(EXIT_FAILURE);
	}

	printf("\nYour ROS Sprayer Driver is currently running\nClose the Terminal window when done.\n");

	// Set up the COM port
	/**********************/
	char* com_port = argv[1];
	int baud_rate = atoi(argv[2]);
    char mode[]={'8','N','1',0};

    // Set up Subscriber
    /********************/
    ros::Subscriber sub2 = nh.subscribe("sprayers", 100, sprayerCallback);

	// Get data from com port //
	/**************************/
	int bytes_received;

	if (OpenComport(com_port, baud_rate))
	{
		printf("Could not open serial port: %s \n",com_port);
		exit(EXIT_FAILURE);
	}

	while(ros::ok())
	{
        if(engageSprayer){
            engageSprayer = false;

            // Byte breakup example
            // uint8_t  bytes[2];
            // uint16_t value;
            // value = 0x1234;
            // bytes[0] = value >> 8;     // high byte (0x12)
            // bytes[1] = value & 0x00FF; // low byte (0x34)
            // uint8_t checksum;
            // checksum ^= sprayerIDs >> 8;
            // checksum ^= sprayerIDs & 0x00FF;
            // checksum ^= sprayTime >> 8;
            // checksum ^= sprayTime & 0x00FF;
            //
            // serialPacket packet;
            // packet.sprayerIDs = sprayerIDs;
            // packet.sprayTime = sprayTime;
            // packet.checksum = checksum;
            // Sends a string to the serial port
            // str[2][512];
            // strcpy(str[0], "The quick brown fox jumped over the lazy grey dog.\n");
            // strcpy(str[1], "Happy serial programming!\n");
            // RS232_cputs(cport_nr, str[i]);
            // RS232_cputs(int comport_number, const char *text)
            // Send a buffer to the serial port
            // RS232_SendBuf(int comport_number, unsigned char *buf, int size)

            unsigned char strBuf[512];
            // std::string sprayerIDs_str = manual_to_string(sprayerIDs);
            // std::string sprayTime_str = manual_to_string(sprayTime);

            std::string sprayerIDs_str = to_string(sprayerIDs);
            std::string sprayTime_str = to_string(sprayTime);

            strBuf[0] = 's';

            int count = 1;
            for ( std::string::iterator it=sprayerIDs_str.begin(); it!=sprayerIDs_str.end(); ++it){
                if(*it != 0){
                    strBuf[count] = *it;
                    ++count;
                }
            }

            strBuf[count] = ',';
            ++count;

            for ( std::string::iterator it=sprayTime_str.begin(); it!=sprayTime_str.end(); ++it){
                if(*it != 0){
                    strBuf[count] = *it;
                    ++count;
                }
            }

            strBuf[count] = 'e';
            ++count;

            int bytesSent = SendBuf(strBuf, count);

            ROS_INFO("Spray %d, Time %d", sprayerIDs, sprayTime);
            ROS_INFO("Bytes sent: %d", bytesSent);
            ROS_INFO("%s", strBuf);
        }

        // Check if new sprayers need to be turned on
        ros::spinOnce();
	}
}
