/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*        ROS Driver, Packet to Published Message Example       */
/*              Copyright 2017, Advanced Navigation             */
/*                                                              */
/****************************************************************/
/*
 * Copyright (C) 2017 Advanced Navigation
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>

#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <nav_msgs/Odometry.h>

#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Constants.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace GeographicLib;

#define RADIANS_TO_DEGREES (180.0/M_PI)

int main(int argc, char *argv[]) {

	// Set up ROS node //
	/*******************/
	ros::init(argc, argv, "an_device_node");
	ros::NodeHandle nh;

	if(argc != 3)
	{
		printf("\nCannot start - not enough commnand line arguments. \nUsage: rosrun an_driver an_driver {port} {baud rate}. \nTry: rosrun an_driver an_driver /dev/ttyUSB0 115200\n");
		printf("Number of command line arguments detected: %i\n",argc);
		exit(EXIT_FAILURE);
	}

	printf("\nYour Advanced Navigation ROS driver is currently running\nClose the Terminal window when done.\n");

	// Set up the COM port
	/**********************/
	char* com_port = argv[1];
	int baud_rate = atoi(argv[2]);

	// Initialise Publishers and Topics //
	/************************************/
	ros::Publisher nav_sat_fix_pub = nh.advertise<sensor_msgs::NavSatFix>("an_device/NavSatFix", 10);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("an_device/Odom", 100);
	// ros::Publisher odom2_pub = nh.advertise<nav_msgs::Odometry>("an_device/Odom2", 10);
	// ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("an_device/Pose", 10);
	// ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("an_device/Twist",10);
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("an_device/Twist", 10);
	// ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("an_device/Twist", 10);
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("an_device/Imu", 10);
	// ros::Publisher imu_ENU_pub = nh.advertise<sensor_msgs::Imu>("an_device/Imu_ENU", 10);
	ros::Publisher system_status_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("an_device/SystemStatus", 10);
	ros::Publisher filter_status_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("an_device/FilterStatus", 10);


	// Initialise messages

	// NAV_SAT VARIABLES
	/********************/
	sensor_msgs::NavSatFix nav_sat_fix_msg;
	nav_sat_fix_msg.header.stamp.sec=0;
	nav_sat_fix_msg.header.stamp.nsec=0;
	nav_sat_fix_msg.header.frame_id="adnav_frame"; // fixed
	nav_sat_fix_msg.status.status=0;
	nav_sat_fix_msg.status.service=1; // fixed to GPS
	nav_sat_fix_msg.latitude=0.0;
	nav_sat_fix_msg.longitude=0.0;
	nav_sat_fix_msg.altitude=0.0;
	nav_sat_fix_msg.position_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	nav_sat_fix_msg.position_covariance_type=2; // fixed to variance on the diagonal

	// ODOMETRY VARIABLES
	/*********************/
	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp.sec = 0;
	odom_msg.header.stamp.nsec = 0;
	odom_msg.header.frame_id = "odom";
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

	// nav_msgs::Odometry odom_msg2;
	// odom_msg2.header.stamp.sec = 0;
	// odom_msg2.header.stamp.nsec = 0;
	// odom_msg2.header.frame_id = "odom";
	// odom_msg2.pose.pose.position.x = 0;
	// odom_msg2.pose.pose.position.y = 0;
	// odom_msg2.pose.pose.position.z = 0;


	// POSE MAPPING VARIABLES
	/*************************/
	bool originReset = true;
	double previousPoseX = 0.0;
	double previousPoseY = 0.0;
	double previousPoseZ = 0.0;
	double currentPoseX = 0.0;
	double currentPoseY = 0.0;
	double currentPoseZ = 0.0;
	double previousLatitude = 0.0;
	double previousLongitude = 0.0;
	double previousAltitude = 0.0;
	double currentLatitude = 0.0;
	double currentLongitude = 0.0;
	double currentAltitude = 0.0;
	double originLatitude = 0.0;
	double originLongitude = 0.0;
	double originAltitude = 0.0;
	double maxSpeed = 28.0; // m/s ~= 100 km/hr

	GeographicLib::Geodesic geod(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());

	tf2_ros::TransformBroadcaster br;

	// POSE VARIABLES
	/***********************/
	// geometry_msgs::PoseStamped pose_msg;
	// pose_msg.header.stamp.sec = 0;
	// pose_msg.header.stamp.nsec = 0;
	// pose_msg.header.frame_id = "adnav_frame"; // fixed
	// pose_msg.pose.position.x = 0;
	// pose_msg.pose.position.y = 0;
	// pose_msg.pose.position.z = 0;
	// pose_msg.pose.orientation.x = 0;
	// pose_msg.pose.orientation.y = 0;
	// pose_msg.pose.orientation.z = 0;
	// pose_msg.pose.orientation.w = 0;
	// TWIST VARIABLES
	/******************/
	// geometry_msgs::Twist twist_msg;
	// twist_msg.linear.x=0.0;
	// twist_msg.linear.y=0.0;
	// twist_msg.linear.z=0.0;
	// twist_msg.angular.x=0.0;
	// twist_msg.angular.y=0.0;
	// twist_msg.angular.z=0.0;

	geometry_msgs::TwistWithCovarianceStamped twist_msg;
	//geometry_msgs::TwistStamped twist_msg;
	twist_msg.header.stamp.sec=0;
	twist_msg.header.stamp.nsec=0;
	twist_msg.header.frame_id="adnav_frame"; // fixed
	// twist_msg.twist.linear.x=0.0;
	// twist_msg.twist.linear.y=0.0;
	// twist_msg.twist.linear.z=0.0;
	// twist_msg.twist.angular.x=0.0;
	// twist_msg.twist.angular.y=0.0;
	// twist_msg.twist.angular.z=0.0;
	twist_msg.twist.twist.linear.x=0.0;
	twist_msg.twist.twist.linear.y=0.0;
	twist_msg.twist.twist.linear.z=0.0;
	twist_msg.twist.twist.angular.x=0.0;
	twist_msg.twist.twist.angular.y=0.0;
	twist_msg.twist.twist.angular.z=0.0;
	twist_msg.twist.covariance = {  0.001,	0.0,	0.0,	0.0,	0.0,	0.0,
								  	0.0,	0.001,	0.0,	0.0,	0.0,	0.0,
									0.0,	0.0,	0.001,	0.0,	0.0,	0.0,
									0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
									0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
									0.0,	0.0,	0.0,	0.0,	0.0,	0.0  };

	// IMU VARIABLES
	/******************/
	sensor_msgs::Imu imu_msg;
	imu_msg.header.stamp.sec=0;
	imu_msg.header.stamp.nsec=0;
	imu_msg.header.frame_id="adnav_frame"; // fixed
	imu_msg.orientation.x=0.0;
	imu_msg.orientation.y=0.0;
	imu_msg.orientation.z=0.0;
	imu_msg.orientation.w=0.0;
	imu_msg.orientation_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	imu_msg.angular_velocity.x=0.0;
	imu_msg.angular_velocity.y=0.0;
	imu_msg.angular_velocity.z=0.0;
	imu_msg.angular_velocity_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	imu_msg.linear_acceleration.x=0.0;
	imu_msg.linear_acceleration.y=0.0;
	imu_msg.linear_acceleration.z=0.0;
	imu_msg.linear_acceleration_covariance={0.001,	0.0,	0.0,
											0.0,	0.001,	0.0,
											0.0,	0.0,	0.001}; // fixed

	// DIAGNOSTIC VARIABLES
	/***********************/
	// System Status
	diagnostic_msgs::DiagnosticStatus system_status_msg;
	system_status_msg.level = 0; // default OK state
	system_status_msg.name = "System Status";
	system_status_msg.message = "";

	// Filter Status
	diagnostic_msgs::DiagnosticStatus filter_status_msg;
	filter_status_msg.level = 0; // default OK state
	filter_status_msg.name = "Filter Status";
	filter_status_msg.message = "";

	// Packet VARIABLES
	/******************/
	an_decoder_t an_decoder;
	an_packet_t *an_packet;
	system_state_packet_t system_state_packet;
	quaternion_orientation_standard_deviation_packet_t quaternion_orientation_standard_deviation_packet;
	velocity_standard_deviation_packet_t velocity_standard_deviation_packet;
	body_velocity_packet_t body_velocity_packet;
	euler_orientation_standard_deviation_packet_t euler_orientation_standard_deviation_packet;


	// Get data from com port //
	/**************************/
	int bytes_received;

	if (OpenComport(com_port, baud_rate))
	{
		printf("Could not open serial port: %s \n",com_port);
		exit(EXIT_FAILURE);
	}

	an_decoder_initialise(&an_decoder);

	// Loop continuously, polling for packets
	// while (1)
	while(ros::ok())
	{
		if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
		{
			// increment the decode buffer length by the number of bytes received //
			an_decoder_increment(&an_decoder, bytes_received);

			// decode all the packets in the buffer //
			while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{
				// system state packet //
				if (an_packet->id == packet_id_system_state)
				{
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{
						// NavSatFix Message
						/********************/

						// If the GPS hasn't connected to a satellite recently the time may be several seconds out.
						// The ROS system time was used instead of the GPS Time
						// This may be an issue if the time must be synced to a GPS for whatever reason.

						// nav_sat_fix_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						// nav_sat_fix_msg.header.stamp.nsec = system_state_packet.microseconds*1000;
						nav_sat_fix_msg.header.stamp = ros::Time::now();

						if ((system_state_packet.filter_status.b.gnss_fix_type == 1) || (system_state_packet.filter_status.b.gnss_fix_type == 2)) {
							nav_sat_fix_msg.status.status=0;
						} else if ((system_state_packet.filter_status.b.gnss_fix_type == 3) || (system_state_packet.filter_status.b.gnss_fix_type == 5)) {
							nav_sat_fix_msg.status.status=1;
						} else if ((system_state_packet.filter_status.b.gnss_fix_type == 4) || (system_state_packet.filter_status.b.gnss_fix_type == 6) || (system_state_packet.filter_status.b.gnss_fix_type == 7)) {
							nav_sat_fix_msg.status.status=2;
						} else {
							nav_sat_fix_msg.status.status=-1;
						}
						nav_sat_fix_msg.latitude=system_state_packet.latitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.longitude=system_state_packet.longitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.altitude=system_state_packet.height;
						nav_sat_fix_msg.position_covariance={	pow(system_state_packet.standard_deviation[1],2), 0.0, 0.0,
																0.0, pow(system_state_packet.standard_deviation[0],2), 0.0,
																0.0, 0.0, pow(system_state_packet.standard_deviation[2],2)	};


						// Odometry Message
						/*******************/

						// Update GPS Coordinates
						/*************************/
						previousLatitude = currentLatitude;
						previousLongitude = currentLongitude;
						previousAltitude = currentAltitude;
						currentLatitude = system_state_packet.latitude * RADIANS_TO_DEGREES;
						currentLongitude = system_state_packet.longitude * RADIANS_TO_DEGREES;
						currentAltitude = system_state_packet.height;

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
							originLatitude = currentLatitude;
							originLongitude = currentLongitude;
							originAltitude = currentAltitude;
							currentPoseX = 0.0;
							currentPoseY = 0.0;
							originReset = false;

						} else {
							// Compute changes in X and Y
							/*****************************/
							/* Calculate the distance between pair of GPS Coordinates */

							// Incremental Positions
							double xDistance; // x distance "adjacent"
							double yDistance; // y distance "opposite"
							double zDistance = currentAltitude - previousAltitude;

							// Difference between Longitudes -> X distance
							geod.Inverse(previousLatitude, previousLongitude, previousLatitude, currentLongitude, xDistance);
							// Difference between Latitide -> Y distance
							geod.Inverse(previousLatitude, currentLongitude,currentLatitude, currentLongitude, yDistance);


							// Distance from origin to current position
							// double xOriginDistance;
							// double yOriginDistance;
							// double zOriginDistance = currentAltitude - originAltitude;
							// geod.Inverse(originLatitude, originLongitude,  originLatitude,  currentLongitude, xOriginDistance);
							// geod.Inverse(originLatitude, currentLongitude, currentLatitude, currentLongitude, yOriginDistance);

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
								// xOriginDistance = -xOriginDistance;
							}

							if((currentLatitude - previousLatitude) >= 0.0){
								// Change in Y is Positive
								// ROS_INFO("Positive Y");
								currentPoseY += yDistance;
							} else {
								// Change in Y is Negative
								// ROS_INFO("Negative Y");
								currentPoseY -= yDistance;
								// yOriginDistance = -yOriginDistance;
							}

							currentPoseZ += zDistance;
						}

						//odom_msg.header.stamp = ros::Time::now();
						odom_msg.header.stamp = nav_sat_fix_msg.header.stamp;
						odom_msg.pose.pose.position.x = currentPoseX;
						odom_msg.pose.pose.position.y = currentPoseY;
						odom_msg.pose.pose.position.z = currentPoseZ;

						// Twist
						twist_msg.header.stamp = nav_sat_fix_msg.header.stamp;
						//NED VELOCITY NOT BODY VELOCITY

						// Linear Velocity in x,y,z
						/****************************/
						// Linear x velocity
						//twist_msg.twist.twist.linear.x = system_state_packet.velocity[0]; // x = x NED -> ENU = NED ?? Maybe not?
						twist_msg.twist.twist.linear.x = system_state_packet.velocity[1]; // x = y FOR NED TO ENU

						// Linear y velocity
						// twist_msg.twist.twist.linear.y = system_state_packet.velocity[1]; // y = y NED -> ENU = -NED ?? Maybe not?
						twist_msg.twist.twist.linear.y = system_state_packet.velocity[0]; // y = x FOR NED TO ENU

						// Linear z velocity -> ENU = -NED
						// CURRENTLY INVERTED
						twist_msg.twist.twist.linear.z = -system_state_packet.velocity[2];


						// Angular Velocity in roll (around x), pitch (around y), yaw (around z)
						/***********************************************************************/
						// Angular x velocity
						//twist_msg.twist.twist.angular.x = system_state_packet.angular_velocity[0]; // x = x NED -> ENU = NED ?? Maybe not?
						twist_msg.twist.twist.angular.x = system_state_packet.angular_velocity[1]; // x = y FOR NED TO ENU

						// Angular y velocity
						// twist_msg.twist.twist.angular.y = system_state_packet.angular_velocity[1]; // y = y NED -> ENU = -NED ?? Maybe not?
						twist_msg.twist.twist.angular.y = system_state_packet.angular_velocity[0]; // y = x FOR NED TO ENU

						// Angular z velocity
						twist_msg.twist.twist.angular.z = -system_state_packet.angular_velocity[2]; // z = -z for NED to ENU

						// twist_msg.twist.angular.z = system_state_packet.angular_velocity[2];
						// twist_msg.twist.covariance = {  0.5,0.0,0.0,0.0,0.0,0.0,
						// 								0.0,0.5,0.0,0.0,0.0,0.0,
						// 								0.0,0.0,0.5,0.0,0.0,0.0,
						// 								0.0,0.0,0.0,0.5,0.0,0.0,
						// 								0.0,0.0,0.0,0.0,0.5,0.0,
						// 								0.0,0.0,0.0,0.0,0.0,0.5  };


						// Add Twist to Odom
						odom_msg.twist = twist_msg.twist;


						// IMU
						// imu_msg.header.stamp.sec=system_state_packet.unix_time_seconds;
						// imu_msg.header.stamp.nsec=system_state_packet.microseconds*1000;
						//imu_msg.header.stamp = ros::Time::now();
						imu_msg.header.stamp = nav_sat_fix_msg.header.stamp;

						// Convert roll, pitch, yaw from radians to quaternion format //
						// ROLL (around x)
						//float phi = system_state_packet.orientation[0] / 2.0f; // x = x NED -> ?? ENU = NED Maybe not
						float phi = system_state_packet.orientation[1] / 2.0f; // Was Theta  x = y FOR NED TO ENU

						// Pitch (around y
						//float theta = system_state_packet.orientation[1] / 2.0f; // y = y NED -> ?? ENU = -NED Maybe not
						float theta = system_state_packet.orientation[0] / 2.0f; // Was phi y = x FOR NED TO ENU

						// Yaw (around z)
						float psi = -system_state_packet.orientation[2] / 2.0f; // z = -z for NED to ENU

						float sin_phi = sinf(phi);
						float cos_phi = cosf(phi);
						float sin_theta = sinf(theta);
						float cos_theta = cosf(theta);
						float sin_psi = sinf(psi);
						float cos_psi = cosf(psi);

						imu_msg.orientation.x=-cos_phi * sin_theta * sin_psi + sin_phi * cos_theta * cos_psi;
						imu_msg.orientation.y=cos_phi * sin_theta * cos_psi + sin_phi * cos_theta * sin_psi;
						imu_msg.orientation.z=cos_phi * cos_theta * sin_psi - sin_phi * sin_theta * cos_psi;
						imu_msg.orientation.w=cos_phi * cos_theta * cos_psi + sin_phi * sin_theta * sin_psi;


						// Add orientation to ODOM message
						odom_msg.pose.pose.orientation = imu_msg.orientation;


						// tf::Quaternion q_orig, q_rot, q_new;
						// double r=3.14159, p=0, y=0;  // Rotate the previous pose by 180* about X
						// q_rot = tf::createQuaternionFromRPY(r, p, y);
						//
						// quaternionMsgToTF(commanded_pose.pose.orientation , q_orig);  // Get the original orientation of 'commanded_pose'
						//
						// q_new = q_rot*q_orig;  // Calculate the new orientation
						// q_new.normalize();
						// quaternionTFToMsg(q_new, commanded_pose.pose.orientation);  // Stuff the new rotation back into the pose. This requires conversion into a msg type

						// Angular Velocity in roll (around x), pitch (around y), yaw (around z)
						// Angular x velocity
						// imu_msg.angular_velocity.x = system_state_packet.angular_velocity[0];
						imu_msg.angular_velocity.x = twist_msg.twist.twist.angular.x; // These the same as the TWIST msg values

						// Angular y velocity
						// imu_msg.angular_velocity.y = -system_state_packet.angular_velocity[1];
						imu_msg.angular_velocity.y = twist_msg.twist.twist.angular.y; // These the same as the TWIST msg values

						// Angular z velocity
						// imu_msg.angular_velocity.z = -system_state_packet.angular_velocity[2];
						imu_msg.angular_velocity.z = twist_msg.twist.twist.angular.z; // These the same as the TWIST msg values

						// Linear Acceleration in x,y,z
						// Linear Acceleration in x
						// imu_msg.linear_acceleration.x = system_state_packet.body_acceleration[0]; // x = x NED
						imu_msg.linear_acceleration.x = system_state_packet.body_acceleration[1]; // x = y for NED TO ENU

						// Linear Acceleration in y
						// imu_msg.linear_acceleration.y = -system_state_packet.body_acceleration[1]; // y = y NED
						imu_msg.linear_acceleration.y = system_state_packet.body_acceleration[0]; // y = x for NED TO ENU

						// Linear Acceleration in z
						imu_msg.linear_acceleration.z = -system_state_packet.body_acceleration[2]; // z = -z for NED TO ENU


						// Sending the Transform for the position of the vehicle in 3D Space
						geometry_msgs::TransformStamped transformStamped;
						transformStamped.header.stamp = nav_sat_fix_msg.header.stamp;
						transformStamped.header.frame_id = "odom";
						transformStamped.child_frame_id = "base_link";
						transformStamped.transform.translation.x = odom_msg.pose.pose.position.x;
						transformStamped.transform.translation.y = odom_msg.pose.pose.position.y;
						transformStamped.transform.translation.z = odom_msg.pose.pose.position.z;
						transformStamped.transform.rotation.x = imu_msg.orientation.x;
						transformStamped.transform.rotation.y = imu_msg.orientation.y;
						transformStamped.transform.rotation.z = imu_msg.orientation.z;
						transformStamped.transform.rotation.w = imu_msg.orientation.w;

						br.sendTransform(transformStamped);






						// System Status
						system_status_msg.message = "";
						system_status_msg.level = 0; // default OK state
						if (system_state_packet.system_status.b.system_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "0. System Failure! ";
						}
						if (system_state_packet.system_status.b.accelerometer_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "1. Accelerometer Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.gyroscope_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "2. Gyroscope Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.magnetometer_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "3. Magnetometer Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.pressure_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "4. Pressure Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.gnss_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "5. GNSS Failure! ";
						}
						if (system_state_packet.system_status.b.accelerometer_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "6. Accelerometer Over Range! ";
						}
						if (system_state_packet.system_status.b.gyroscope_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "7. Gyroscope Over Range! ";
						}
						if (system_state_packet.system_status.b.magnetometer_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "8. Magnetometer Over Range! ";
						}
						if (system_state_packet.system_status.b.pressure_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "9. Pressure Over Range! ";
						}
						if (system_state_packet.system_status.b.minimum_temperature_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "10. Minimum Temperature Alarm! ";
						}
						if (system_state_packet.system_status.b.maximum_temperature_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "11. Maximum Temperature Alarm! ";
						}
						if (system_state_packet.system_status.b.low_voltage_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "12. Low Voltage Alarm! ";
						}
						if (system_state_packet.system_status.b.high_voltage_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "13. High Voltage Alarm! ";
						}
						// if (system_state_packet.system_status.b.gnss_antenna_disconnected) {
						if (system_state_packet.system_status.b.gnss_antenna_fault) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "14. GNSS Antenna Disconnected! ";
						}
						if (system_state_packet.system_status.b.serial_port_overflow_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "15. Data Output Overflow Alarm! ";
						}

						// Filter Status
						filter_status_msg.message = "";
						filter_status_msg.level = 0; // default OK state
						if (system_state_packet.filter_status.b.orientation_filter_initialised) {
							filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter Initialised. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.ins_filter_initialised) {
							filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter Initialised. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.heading_initialised) {
							filter_status_msg.message = filter_status_msg.message + "2. Heading Initialised. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "2. Heading NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.utc_time_initialised) {
							filter_status_msg.message = filter_status_msg.message + "3. UTC Time Initialised. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "3. UTC Time NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.event1_flag) {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "7. Event 1 Occured. ";
						}
						else {
							filter_status_msg.message = filter_status_msg.message + "7. Event 1 NOT Occured. ";
						}
						if (system_state_packet.filter_status.b.event2_flag) {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "8. Event 2 Occured. ";
						}
						else {
							filter_status_msg.message = filter_status_msg.message + "8. Event 2 NOT Occured. ";
						}
						if (system_state_packet.filter_status.b.internal_gnss_enabled) {
							filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS Enabled. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS NOT Enabled. ";
						}
						// if (system_state_packet.filter_status.b.magnetic_heading_enabled) {
						// 	filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading Active. ";
						// }
						// else {
						// 	filter_status_msg.level = 1; // WARN state
						// 	filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading NOT Active. ";
						// }
						if (system_state_packet.filter_status.b.velocity_heading_enabled) {
							filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading Enabled. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading NOT Enabled. ";
						}
						if (system_state_packet.filter_status.b.atmospheric_altitude_enabled) {
							filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude Enabled. ";
						}
						else {
							filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude NOT Enabled. ";
							filter_status_msg.level = 1; // WARN state
						}
						if (system_state_packet.filter_status.b.external_position_active) {
							filter_status_msg.message = filter_status_msg.message + "13. External Position Active. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "13. External Position NOT Active. ";
						}
						if (system_state_packet.filter_status.b.external_velocity_active) {
							filter_status_msg.message = filter_status_msg.message + "14. External Velocity Active. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "14. External Velocity NOT Active. ";
						}
						if (system_state_packet.filter_status.b.external_heading_active) {
							filter_status_msg.message = filter_status_msg.message + "15. External Heading Active. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "15. External Heading NOT Active. ";
						}
					}
				}

				// Old Unused but potentially useful data
				// if (an_packet->id == packet_id_body_velocity)
				// {
				// 	if(decode_body_velocity_packet(&body_velocity_packet, an_packet) == 0)
				// 	{
				// 		// Linear Velocity in x,y,z
				// 		// Linear x velocity -> ENU = NED
				// 		twist_msg.twist.twist.linear.x = body_velocity_packet.velocity[0];
				// 		// Linear y velocity -> ENU = -NED
				// 		// INVERTED
				// 		twist_msg.twist.twist.linear.y = -(body_velocity_packet.velocity[1]);
				// 		// Linearz velocity -> ENU = -NED
				// 		// INVERTED
				// 		twist_msg.twist.twist.linear.z = -(body_velocity_packet.velocity[2]);
				// 	}
				// }
				// quaternion orientation standard deviation packet //
				// if (an_packet->id == packet_id_quaternion_orientation_standard_deviation)
				// {
				// 	// copy all the binary data into the typedef struct for the packet //
				// 	// this allows easy access to all the different values             //
				// 	if(decode_quaternion_orientation_standard_deviation_packet(&quaternion_orientation_standard_deviation_packet, an_packet) == 0)
				// 	{
				// 		// IMU
				// 		imu_msg.orientation_covariance[0] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[0], 2);
				// 		imu_msg.orientation_covariance[4] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[1], 2);
				// 		imu_msg.orientation_covariance[8] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[2], 2);
				// 		// imu_msg.orientation_covariance[0] = quaternion_orientation_standard_deviation_packet.standard_deviation[0];
				// 		// imu_msg.orientation_covariance[4] = quaternion_orientation_standard_deviation_packet.standard_deviation[1];
				// 		// imu_msg.orientation_covariance[8] = quaternion_orientation_standard_deviation_packet.standard_deviation[2];
				// 	}
				// }
				// if (an_packet->id == packet_id_velocity_standard_deviation){
				// 	if(decode_velocity_standard_deviation_packet(&velocity_standard_deviation_packet, an_packet) == 0)
				// 	{
				// 		// Linear velocity Covariance
				// 		twist_msg.twist.covariance[0] = pow(velocity_standard_deviation_packet.standard_deviation[0], 2);
				// 		twist_msg.twist.covariance[7] = pow(velocity_standard_deviation_packet.standard_deviation[1], 2);
				// 		twist_msg.twist.covariance[14] = pow(velocity_standard_deviation_packet.standard_deviation[2], 2);
				// 	}
				// }
				// if (an_packet->id == packet_id_euler_orientation_standard_deviation){
				// 	if(decode_euler_orientation_standard_deviation_packet(&euler_orientation_standard_deviation_packet, an_packet) == 0)
				// 	{
				// 		// Angular velocity Covariance
				// 		twist_msg.twist.covariance[21] = pow(euler_orientation_standard_deviation_packet.standard_deviation[0], 2);
				// 		twist_msg.twist.covariance[28] = pow(euler_orientation_standard_deviation_packet.standard_deviation[1], 2);
				// 		twist_msg.twist.covariance[35] = pow(euler_orientation_standard_deviation_packet.standard_deviation[2], 2);
				//
				// 		imu_msg.angular_velocity_covariance[0] = pow(euler_orientation_standard_deviation_packet.standard_deviation[0], 2);
				// 		imu_msg.angular_velocity_covariance[4] = pow(euler_orientation_standard_deviation_packet.standard_deviation[1], 2);
				// 		imu_msg.angular_velocity_covariance[8] = pow(euler_orientation_standard_deviation_packet.standard_deviation[2], 2);
				// 	}
				// }

				// Ensure that you free the an_packet when your done with it //
				// or you will leak memory                                   //
				an_packet_free(&an_packet);

				// Publish messages //
				nav_sat_fix_pub.publish(nav_sat_fix_msg);
				twist_pub.publish(twist_msg);
				imu_pub.publish(imu_msg);
				odom_pub.publish(odom_msg);
				// odom2_pub.publish(odom_msg2);
				system_status_pub.publish(system_status_msg);
				filter_status_pub.publish(filter_status_msg);
			}
		}
	}
}
