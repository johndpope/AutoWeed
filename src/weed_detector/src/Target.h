/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   Target.h
 * Author: jc210083
 *
 * Created on 10 September 2018, 4:39 PM
 */

#include <ros/ros.h>
#include <ctime>
#include <opencv2/core/types.hpp>
#include "custom_messages/TargetInfo.h"

#ifndef TARGET_H
#define TARGET_H

class Target {
public:
    Target(unsigned int x, unsigned int y, unsigned int height, unsigned int width, ros::Time timestamp);
    Target(unsigned int x, unsigned int y, unsigned int height, unsigned int width, ros::Time timestamp, unsigned int cameraID, unsigned int imageHeight, unsigned int imageWidth);
    Target();
    Target(const Target& orig);
    virtual ~Target();
    unsigned int targetID;
    unsigned int cameraID;
    unsigned int imageHeight;
    unsigned int imageWidth;
    unsigned int x;
    unsigned int y;
    unsigned int height;
    unsigned int width;
    ros::Time timestamp;
    static unsigned int targetCount;
private:

};

#endif /* TARGET_H */
