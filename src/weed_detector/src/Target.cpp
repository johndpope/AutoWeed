/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   Target.cpp
 * Author: jc210083
 *
 * Created on 10 September 2018, 4:39 PM
 */

#include <ros/ros.h>
#include "Target.h"
#include "custom_messages/TargetInfo.h"

// Initialise the target count
unsigned int Target::targetCount = 0;

Target::Target() {
}

Target::Target(unsigned int x, unsigned int y, unsigned int height, unsigned int width, ros::Time timestamp) {
    this->x = x;
    this->y = y;
    this->height = height;
    this->width = width;
    this->timestamp = timestamp;

    // Increment static target count and assign unique target ID
    targetCount++;
    this->targetID = targetCount;
}

Target::Target(unsigned int x, unsigned int y, unsigned int height, unsigned int width, ros::Time timestamp, unsigned int cameraID, unsigned int imageHeight, unsigned int imageWidth) {
    this->x = x;
    this->y = y;
    this->height = height;
    this->width = width;
    this->cameraID = cameraID;
    this->imageHeight = imageHeight;
    this->imageWidth = imageWidth;
    this->timestamp = timestamp;

    // Increment static target count and assign unique target ID
    targetCount++;
    this->targetID = targetCount;
}

Target::Target(const Target& orig) {
}

Target::~Target() {
}
