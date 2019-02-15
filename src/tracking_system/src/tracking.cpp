#include "ros/ros.h"
#include "std_msgs/String.h"
#include "custom_messages/TargetInfo.h"
#include "custom_messages/SprayerInfo.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <list>
#include <string>
#include <array>
#include <math.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


bool isPositionWithinCircleLimits(custom_messages::TargetInfo targetInfo, geometry_msgs::TransformStamped sprayerInfo, double radius, double velocity){
    //double delayDistance = velocity * SPRAY_DELAY_TIME;
    //double delayDistance = 0.1 * velocity;
    double diffX = sprayerInfo.transform.translation.x - targetInfo.targetTransform.transform.translation.x;
    double diffY = sprayerInfo.transform.translation.y - targetInfo.targetTransform.transform.translation.y; // - delayDistance;
    //double delayDistance = SPRAY_DELAY_TIME * velocity; // 0.6;// The distance the sprayer will move from the time it is "turned on" to the time it actually activates.
    // Returns true if the sprayer is on or within the delay distance of the target
    // Ideally this will hit the target dead on the centre point
    return ((diffX * diffX + diffY * diffY) <= radius * radius);
    //if ((diffX*diffX + diffY*diffY) <= (radius + delayDistance) * (radius + delayDistance))
    //{
    //    return true;
    //}
    //return false;
}

bool isTargetDuplicate(custom_messages::TargetInfo target1, custom_messages::TargetInfo target2, double radius, double velocity){
    double diffX = target2.targetTransform.transform.translation.x - target1.targetTransform.transform.translation.x;
    double diffY = target2.targetTransform.transform.translation.y - target1.targetTransform.transform.translation.y;
    return ((diffX * diffX + diffY * diffY) <= radius * radius);
}

std::list<custom_messages::TargetInfo> targetInfoList;
std::list<custom_messages::TargetInfo> targetsToTransformList;
nav_msgs::Odometry currentOdomMsg;
//std::list<TargetData> targetList;
double currentVelocityX;
double currentVelocityY;
double sprayRadius = 0.5; // meters (24cm Diameter)
double sprayInbetweenRadius = 0.25; // meters (32cm Diameter)
double boundarySize = 0.5; // meters

double duplicateRejectionSize = 0.01; // meters (2cm Diameter)

bool targetsToTransform = false;

double imageWidth = 0.445;
double imageHeight = 0.275;

double imagePixelWidth = 430;
double imagePixelHeight = 300;


std::string cameras[4] = {"camera_1_frame", "camera_2_frame", "camera_3_frame", "camera_4_frame"};

 /*
 * Updates the local position variables
 */
void positionCallback(const nav_msgs::OdometryConstPtr& msg){
    currentVelocityX = msg->twist.twist.linear.x;
    currentVelocityY = msg->twist.twist.linear.y;

    currentOdomMsg = *msg;
}

void targetFound(const custom_messages::TargetInfoConstPtr& msg){
    // ROS_INFO("Target Received: [%i]", msg->targetId);
    // ROS_INFO("Target Location: Lat=[%f], Long=[%f]", msg->latitude, msg->longitude);
    // ROS_INFO("Current Position: Lat=[%f], Long=[%f]", currentLatitude, currentLongitude);
    // // Create new Target Object
    //TargetData newTarget = TargetData(msg->targetId, msg->positionX, msg->positionY, msg->positionZ, msg->latitude, msg->longitude, msg->altitude, msg->xSize, msg->ySize);
    // Add to the end of the list
    //targetList.push_back(newTarget);

    // Check if the new target is a duplicate in terms of position
    targetsToTransformList.push_back(*msg);
    targetsToTransform = true;
    ROS_INFO("Target %d Added", msg->targetId);

}

 /*
  * Chow Main
  */
int main(int argc, char **argv){
    ros::init(argc, argv, "autoweed_tracking_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("an_device/Odom", 100, positionCallback);
    ros::Subscriber sub2 = nh.subscribe("targets", 100, targetFound);
    ros::Publisher sprayer_publisher = nh.advertise<custom_messages::SprayerInfo>("sprayers", 100);
    ros::Publisher locatedTarget_publisher = nh.advertise<custom_messages::TargetInfo>("locatedTargets", 100);
    // ros::Subscriber sub = n.subscribe("NavSatFix", 100, positionCallback);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Sprayers for 7 Sprayers (Starting from 1-7)
    // std::string sprayerId[7] = {"spray_1_frame", "spray_2_frame", "spray_3_frame", "spray_4_frame", "spray_5_frame", "spray_6_frame", "spray_7_frame"};
    // std::string sprayerMidId[6] = {"spray_mid_1_frame", "spray_mid_2_frame", "spray_mid_3_frame", "spray_mid_4_frame", "spray_mid_5_frame", "spray_mid_6_frame"};
    // std::array<geometry_msgs::TransformStamped, 7> sprayerPosition;
    // std::array<geometry_msgs::TransformStamped, 6> sprayerMidPosition;


    // Sprayers for 5 Sprayers (Starting from sprayer 2-6)
    std::string sprayerId[5] = {"spray_2_frame", "spray_3_frame", "spray_4_frame", "spray_5_frame", "spray_6_frame"};
    std::string sprayerMidId[4] = {"spray_mid_2_frame", "spray_mid_3_frame", "spray_mid_4_frame", "spray_mid_5_frame"};
    std::array<geometry_msgs::TransformStamped, 5> sprayerPosition;
    std::array<geometry_msgs::TransformStamped, 4> sprayerMidPosition;

    bool publishSprayerMsg = false;

    while(ros::ok()){

        ros::spinOnce();

        if(targetsToTransform){

            for (std::list<custom_messages::TargetInfo>::iterator target=targetsToTransformList.begin(); target != targetsToTransformList.end(); ){

                // Determine the transform for the camera from when the image was captured
                geometry_msgs::TransformStamped transformStamped;
                try{
                    transformStamped = tfBuffer.lookupTransform("odom", cameras[(*target).cameraId -1], (*target).header.stamp, ros::Duration(1.0));
                } catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                }

                (*target).cameraTransform = transformStamped;
                // Target transform basically the same as the camera, the actual position is changed for the target below
                (*target).targetTransform = transformStamped;

                // Determine size of bounding box in meters irl
                (*target).width = (((*target).boxWidth / imagePixelWidth) * imageWidth);
                (*target).height = (((*target).boxHeight / imagePixelHeight) * imageHeight);

                // Dumb shit converting geometry_msgs::transforms to tf2::transforms
                geometry_msgs::TransformStamped targetTransformMsg;
                // Determine the position of the target in the image in meters with the origin at the center of the image
                (*target).xPosition = (((*target).xPixelLocation / imagePixelWidth) * imageWidth) - (imageWidth/2);
                (*target).yPosition = (imageHeight/2) - (((*target).yPixelLocation / imagePixelHeight) * imageHeight);

                targetTransformMsg.transform.translation.x = (*target).xPosition;
                targetTransformMsg.transform.translation.y = (*target).yPosition;
                tf2::Transform targetTransform;
                targetTransform.setOrigin(tf2::Vector3(targetTransformMsg.transform.translation.x, targetTransformMsg.transform.translation.y, targetTransformMsg.transform.translation.z));
                //targetTransform.setRotation(tf2::Quaternion(targetTransformMsg.transform.rotation.x, targetTransformMsg.transform.rotation.y, targetTransformMsg.transform.rotation.z, targetTransformMsg.transform.rotation.w));
                tf2::Quaternion q;
                q.setRPY(0.0,0.0,0.0);
                targetTransform.setRotation(q);

                tf2::Transform cameraTransform;
                cameraTransform.setOrigin(tf2::Vector3(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z));
                cameraTransform.setRotation(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));

                // Perform actual translation for target within image to odom frame THROUGH the camera/odom transform
                cameraTransform *= targetTransform;

                // Put the new location of the target transformed to the world coordinates into the target message
                (*target).targetTransform.transform.translation.x = cameraTransform.getOrigin().x();
                (*target).targetTransform.transform.translation.y = cameraTransform.getOrigin().y();
                (*target).targetTransform.transform.translation.z = cameraTransform.getOrigin().z();
                (*target).targetTransform.transform.rotation.x = cameraTransform.getRotation().x();
                (*target).targetTransform.transform.rotation.y = cameraTransform.getRotation().y();
                (*target).targetTransform.transform.rotation.z = cameraTransform.getRotation().z();
                (*target).targetTransform.transform.rotation.w = cameraTransform.getRotation().w();

                bool addTarget = true;

                for (std::list<custom_messages::TargetInfo>::iterator existTarget=targetInfoList.begin(); existTarget != targetInfoList.end(); ++existTarget){

                    double targetBoundaryRadius;

                    // Determine existing target boundary radius from larger of height or width
                    if (existTarget->height >= existTarget->width) {
                        targetBoundaryRadius = (existTarget->height)/2;
                    } else {
                        targetBoundaryRadius = (existTarget->width)/2;
                    }

                    if(isTargetDuplicate(*existTarget, *target, targetBoundaryRadius, 0.0)) {
                        addTarget = false;
                        ROS_INFO("Target %d Rejected", target->targetId);
                        break;
                    }
                }

                if(addTarget){
                    // Add target to list of targets to spray
                    targetInfoList.push_back(*target);
                    locatedTarget_publisher.publish(*target);
                }

                // Remove target from targets to transform
                target = targetsToTransformList.erase(target);
            }

            targetsToTransform = false;
        }





        // Get current transform for sprayer position
        for(int i = 0; i < sprayerPosition.size(); ++i){
            try{
                sprayerPosition[i] = tfBuffer.lookupTransform("odom", sprayerId[i], ros::Time(0), ros::Duration(1.0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                //ros::Duration(0.05).sleep();
                continue;
            }
        }

        // Get current transform for mid sprayer position
        for(int i = 0; i < sprayerMidPosition.size(); ++i){
            try{
                sprayerMidPosition[i] = tfBuffer.lookupTransform("odom", sprayerMidId[i], ros::Time(0), ros::Duration(1.0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                //ros::Duration(0.05).sleep();
                continue;
            }
        }


        double velocityMagnitude = sqrt(currentVelocityX*currentVelocityX + currentVelocityY*currentVelocityY);

        // Iterate through each target
        for (std::list<custom_messages::TargetInfo>::iterator target=targetInfoList.begin(); target != targetInfoList.end(); ){

            custom_messages::SprayerInfo sprayer_msg;
            uint16_t sprayerID = 0;

            bool targetToBeSprayed = false;
            // Check if the targets are going to pass under any indivdual sprayers
            for (int i = 0; i < sprayerPosition.size(); ++i){
                if(isPositionWithinCircleLimits(*target, sprayerPosition[i], sprayRadius, velocityMagnitude)){

                    int sprayerNumber;
                    //Get the correct sprayer number
                    int intsScanned = sscanf(sprayerId[i].c_str(), "spray_%d_frame", &sprayerNumber);

                    ROS_INFO("Spray TARGET %d at SPRAYER %d!!", (*target).targetId, sprayerNumber);

                    // Sprayer ID
                    sprayer_msg.sprayerIDs |= 1UL << (sprayerNumber-1); //-1 for the correct bitshifts

                    //ROS_INFO("SprayerIDs Number: %d", sprayer_msg.sprayerIDs);

                    // Target Info (Wholly embedded)
                    sprayer_msg.targetInfo = *target;
                    sprayer_msg.odom = currentOdomMsg;

                    //ROS_INFO("Target Embedded");
                    sprayer_msg.sprayerTransform = sprayerPosition[i];

                    // Spray Time
                    uint16_t newSprayTime = uint16_t(((*target).height + boundarySize)*1000 / velocityMagnitude); // msec

                    //ROS_INFO("spray time Calculated to %d", newSprayTime);

                    if(newSprayTime > sprayer_msg.sprayerTime){
                        sprayer_msg.sprayerTime = newSprayTime;
                    } else if ( newSprayTime < 64){
                        newSprayTime = 64; // msec
                    }

                    publishSprayerMsg = true;
                    targetToBeSprayed = true;
                    //ROS_INFO("FINISHED single spray check");
                }
            }

            // Check if the targets are going to pass between two sprayers within a smaller area
            for (int i = 0; i < sprayerMidPosition.size(); ++i){
                if(isPositionWithinCircleLimits(*target, sprayerMidPosition[i], sprayInbetweenRadius, velocityMagnitude)){

                    int sprayerNumber;
                    //Get the correct sprayer number
                    int intsScanned = sscanf(sprayerMidId[i].c_str(), "spray_mid_%d_frame", &sprayerNumber);

                    ROS_INFO("Spray TARGET %d at SPRAYER %d and %d!!", (*target).targetId, sprayerNumber, sprayerNumber+1);

                    // Sprayer IDs (to turn on 2 sprayers)
                    sprayer_msg.sprayerIDs |= 1UL << (sprayerNumber-1); // -1 for the correct bitshifts
                    sprayer_msg.sprayerIDs |= 1UL << sprayerNumber; // -1 for the correct bitshifts

                    //ROS_INFO("SprayerIDs Number: %d", sprayer_msg.sprayerIDs);

                    // Target Info (Wholly embedded)
                    sprayer_msg.targetInfo = *target;

                    //ROS_INFO("Target Embedded");

                    sprayer_msg.sprayerMidTransform = sprayerMidPosition[i];


                    // Spray Time
                    uint16_t newSprayTime = uint16_t(((*target).height + boundarySize)*1000 / velocityMagnitude); // msec with twiddle factor

                    //ROS_INFO("spray time Calculated to %d", newSprayTime);

                    if(newSprayTime > sprayer_msg.sprayerTime){
                        sprayer_msg.sprayerTime = newSprayTime;
                    } else if ( newSprayTime < 64){
                        newSprayTime = 64; // msec
                    }

                    publishSprayerMsg = true;
                    targetToBeSprayed = true;

                    //ROS_INFO("FINISHED mid spray check");
                }
            }

            if(targetToBeSprayed){
                // If target to be sprayed, remove from the list
                target = targetInfoList.erase(target);
                //ROS_INFO("Removed target");
            } else {
                ++target;
            }

            if(publishSprayerMsg) {
                sprayer_publisher.publish(sprayer_msg);
                publishSprayerMsg = false;
                //ROS_INFO("Published Spray Message");
            }
        }


        // Check if there are any new targets
        ros::spinOnce();
    }
    return 0;
}
