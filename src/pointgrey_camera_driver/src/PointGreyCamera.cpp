/*
This code was developed by the National Robotics Engineering Center (NREC), part of the Robotics Institute at Carnegie Mellon University.
Its development was funded by DARPA under the LS3 program and submitted for public release on June 7th, 2012.
Release was granted on August, 21st 2012 with Distribution Statement "A" (Approved for Public Release, Distribution Unlimited).

This software is released under a BSD license:

Copyright (c) 2012, Carnegie Mellon University. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the Carnegie Mellon University nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/



/*-*-C++-*-*/
/**
   @file PointGreyCamera.cpp
   @author Chad Rockey
   @date July 11, 2011
   @brief Interface to Point Grey cameras

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

#include "pointgrey_camera_driver/PointGreyCamera.h"

#include <iostream>
#include <sstream>
#include <flycapture/FlyCapture2Defs.h>

using namespace FlyCapture2;

PointGreyCamera::PointGreyCamera():
  busMgr_(), cam_()
{
  serial_ = 0;
  scene_ = 1;
  fps_ = 10;
  high_res_ = 0;
  captureRunning_ = false;
}

PointGreyCamera::~PointGreyCamera()
{
}

void PointGreyCamera::setTimeout(const double &timeout)
{
  FC2Config pConfig;
  Error error = cam_.GetConfiguration(&pConfig);
  PointGreyCamera::handleError("PointGreyCamera::setTimeout Could not get camera configuration", error);
  pConfig.grabTimeout = (int)(1000.0 * timeout); // Needs to be in ms
  if(pConfig.grabTimeout < 0.00001)
  {
    pConfig.grabTimeout = -1; // Default - no timeout
  }
  error = cam_.SetConfiguration(&pConfig);
  PointGreyCamera::handleError("PointGreyCamera::setTimeout Could not set camera configuration", error);
}

float PointGreyCamera::getCameraTemperature()
{
  Property tProp;
  tProp.type = TEMPERATURE;
  Error error = cam_.GetProperty(&tProp);
  PointGreyCamera::handleError("PointGreyCamera::getCameraTemperature Could not get property.", error);
  return tProp.valueA / 10.0f - 273.15f;  // It returns values of 10 * K
}

float PointGreyCamera::getCameraFrameRate()
{
  Property fProp;
  fProp.type = FRAME_RATE;
  Error error = cam_.GetProperty(&fProp);
  PointGreyCamera::handleError("PointGreyCamera::getCameraFrameRate Could not get property.", error);
  std::cout << "Frame Rate is: " << fProp.absValue << std::endl;
  return fProp.absValue;
}

void PointGreyCamera::setGigEParameters(bool auto_packet_size, unsigned int packet_size, unsigned int packet_delay)
{
  auto_packet_size_ = auto_packet_size;
  packet_size_ = packet_size;
  packet_delay_ = packet_delay;
}

void PointGreyCamera::setupGigEPacketSize(PGRGuid & guid)
{
  Error error;
  unsigned int packet_size;
  error = cam_.DiscoverGigEPacketSize(&packet_size);
  PointGreyCamera::handleError("PointGreyCamera::connect could not discover GigE packet_size", error);
  GigEProperty prop;
  prop.propType = PACKET_SIZE;
  error = cam_.GetGigEProperty(&prop);
  PointGreyCamera::handleError("PointGreyCamera::connect could not get GigE packet_size", error);
  prop.value = packet_size;
  error = cam_.SetGigEProperty(&prop);
  PointGreyCamera::handleError("PointGreyCamera::connect could not set GigE packet_size", error);
}

void PointGreyCamera::setupGigEPacketSize(PGRGuid & guid, unsigned int packet_size)
{
  Error error;
  GigEProperty prop;
  prop.propType = PACKET_SIZE;
  prop.value = packet_size;
  error = cam_.SetGigEProperty(&prop);
  PointGreyCamera::handleError("PointGreyCamera::connect could not set GigE packet_size", error);
}

void PointGreyCamera::setupGigEPacketDelay(PGRGuid & guid, unsigned int packet_delay)
{
  Error error;
  GigEProperty prop;
  prop.propType = PACKET_DELAY;
  prop.value = packet_delay;
  error = cam_.SetGigEProperty(&prop);
  PointGreyCamera::handleError("PointGreyCamera::connect could not set GigE packet_delay", error);
}

void PointGreyCamera::connect()
{
  Error error;
  if(!cam_.IsConnected())
  {
    PGRGuid guid;  // GUIDS are NOT persistent accross executions, do not store them.
    if(serial_ != 0)  // If we have a specific camera to connect to.
    {
      error = busMgr_.GetCameraFromSerialNumber(serial_, &guid);
      std::stringstream serial_string;
      serial_string << serial_;
      std::string msg = "PointGreyCamera::connect Could not find camera with serial number: " + serial_string.str() + ". Is that camera plugged in?";
      PointGreyCamera::handleError(msg, error);
    }
    else     // Connect to any camera (the first)
    {
      error  = busMgr_.GetCameraFromIndex(0, &guid);
      PointGreyCamera::handleError("PointGreyCamera::connect Failed to get first connected camera", error);
    }
    
    // Connect to GigE camera
    error = cam_.Connect(&guid);
    PointGreyCamera::handleError("PointGreyCamera::connect could not connect as GigE camera", error);
    
    // Set packet size
    if (auto_packet_size_)
        setupGigEPacketSize(guid);
    else
        setupGigEPacketSize(guid, packet_size_);

    // Set packet delay
    setupGigEPacketDelay(guid, packet_delay_);

    // Enable packet resend
    GigEConfig gigeconfig;
    error = cam_.GetGigEConfig(&gigeconfig);
    PointGreyCamera::handleError("PointGreyCamera::GetGigEConfig could not get GigE setting", error);
    gigeconfig.enablePacketResend = true;
    error = cam_.SetGigEConfig(&gigeconfig);
    PointGreyCamera::handleError("PointGreyCamera::SetGigEConfig could not set GigE settings (packet resend)", error);
    
    // Set resolution
    if (high_res_ == 1) { // High resolution: 1920 x 1200
        Mode mode = MODE_0;
        error = cam_.SetGigEImagingMode(mode);
        PointGreyCamera::handleError("PointGreyCamera::SetGigEConfig could not set imaging mode", error);
    }
    else { // Low resolution: 480 x 300
        Mode mode = MODE_5;
        error = cam_.SetGigEImagingMode(mode);
        PointGreyCamera::handleError("PointGreyCamera::SetGigEConfig could not set imaging mode", error);
        //error = cam_.SetGigEImageBinningSettings(4, 4);
        //PointGreyCamera::handleError("PointGreyCamera::SetGigEConfig could not set binning settings", error);
    }
    
    
    // Set GigE image settings
    GigEImageSettingsInfo imageSettingsInfo;
    error = cam_.GetGigEImageSettingsInfo(&imageSettingsInfo);
    PointGreyCamera::handleError("PointGreyCamera::connect Could not get image settings", error);
    GigEImageSettings imageSettings;
    imageSettings.offsetX = 0;
    imageSettings.offsetY = 0;
    imageSettings.height = imageSettingsInfo.maxHeight;
    imageSettings.width = imageSettingsInfo.maxWidth;
    imageSettings.pixelFormat = PIXEL_FORMAT_RGB8;
    error = cam_.SetGigEImageSettings( &imageSettings );
    PointGreyCamera::handleError("PointGreyCamera::connect Could not set image settings", error);
    
    // Use a low shutter auto exposure algorithm
    Property autoExposure;
    autoExposure.type = AUTO_EXPOSURE;
    autoExposure.onOff = true;
    autoExposure.autoManualMode = true;
    autoExposure.onePush = false;
    autoExposure.absControl = true;
    error = cam_.SetProperty(&autoExposure);
    PointGreyCamera::handleError("PointGreyCamera::connect Could not set exposure", error);
    
    Property frameRate;
    frameRate.type = FRAME_RATE;
    frameRate.onOff = true;
    frameRate.autoManualMode = true;
    frameRate.absControl = true;
    error = cam_.SetProperty(&frameRate);
    PointGreyCamera::handleError("PointGreyCamera::connect Could not set frame rate", error);
    
    Property gain;
    gain.type = GAIN;
    gain.onOff = true;
    gain.autoManualMode = true;
    gain.onePush = false;
    gain.absControl = true;
    error = cam_.SetProperty(&gain);
    PointGreyCamera::handleError("PointGreyCamera::connect Could not set gain", error);
    
    Property shutter;
    shutter.type = SHUTTER;
    shutter.onOff = true;
    shutter.autoManualMode = true;
    shutter.onePush = false;
    shutter.absControl = true;
    error = cam_.SetProperty(&shutter);
    PointGreyCamera::handleError("PointGreyCamera::connect Could not set shutter", error);
    
    // Ideal exposure range
    cam_.WriteRegister(0x1088, 0x80100300, false); //auto exposure
    
    // Maximum gain range
    cam_.WriteRegister(0x10A0, 0x8000012C, false);
    
    // Set shutter range
    if (scene_ == 1) { //outside
        cam_.WriteRegister(0x1098, 0x80001010, false); // smaller exposure range
    } else {
        cam_.WriteRegister(0x1098, 0x80001FFF, false); // larger exposure range
    }
 
    // Enable metadata
    EmbeddedImageInfo info;
    info.timestamp.onOff = true;
    info.gain.onOff = true;
    info.shutter.onOff = true;
    info.brightness.onOff = true;
    info.exposure.onOff = true;
    info.whiteBalance.onOff = true;
    info.frameCounter.onOff = true;
    info.ROIPosition.onOff = true;
    error = cam_.SetEmbeddedImageInfo(&info);
    PointGreyCamera::handleError("PointGreyCamera::connect Could not enable metadata", error);
  }
}

void PointGreyCamera::disconnect()
{
  boost::mutex::scoped_lock scopedLock(mutex_);
  captureRunning_ = false;
  if(cam_.IsConnected())
  {
    Error error = cam_.Disconnect();
    PointGreyCamera::handleError("PointGreyCamera::disconnect Failed to disconnect camera", error);
  }
}

void PointGreyCamera::start()
{
  if(cam_.IsConnected() && !captureRunning_)
  {
    // Start capturing images
    Error error = cam_.StartCapture();
    PointGreyCamera::handleError("PointGreyCamera::start Failed to start capture", error);
    captureRunning_ = true;
  }
}

bool PointGreyCamera::stop()
{
  if(cam_.IsConnected() && captureRunning_)
  {
    // Stop capturing images
    captureRunning_ = false;
    Error error = cam_.StopCapture();
    PointGreyCamera::handleError("PointGreyCamera::stop Failed to stop capture", error);
    return true;
  }
  return false;
}

void PointGreyCamera::grabImage(sensor_msgs::Image &image, const std::string &frame_id)
{
  boost::mutex::scoped_lock scopedLock(mutex_);
  if(cam_.IsConnected() && captureRunning_)
  {
    // Make a FlyCapture2::Image to hold the buffer returned by the camera.
    Image rawImage;
    // Retrieve an image
    Error error = cam_.RetrieveBuffer(&rawImage);
    PointGreyCamera::handleError("PointGreyCamera::grabImage Failed to retrieve buffer", error);
    metadata_ = rawImage.GetMetadata();

    // Set header timestamp as embedded for now
    TimeStamp embeddedTime = rawImage.GetTimeStamp();
    image.header.stamp.sec = embeddedTime.seconds;
    image.header.stamp.nsec = 1000 * embeddedTime.microSeconds;

    // Check the bits per pixel.
    uint8_t bitsPerPixel = rawImage.GetBitsPerPixel();

    // Set the image encoding
    std::string imageEncoding = sensor_msgs::image_encodings::MONO8;
    BayerTileFormat bayer_format = rawImage.GetBayerTileFormat();
    if(isColor_ && bayer_format != NONE)
    {
      if(bitsPerPixel == 16)
      {
        switch(bayer_format)
        {
          case RGGB:
            imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB16;
            break;
          case GRBG:
            imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG16;
            break;
          case GBRG:
            imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG16;
            break;
          case BGGR:
            imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR16;
            break;
        }
      }
      else
      {
        switch(bayer_format)
        {
        case RGGB:
          imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB8;
          break;
        case GRBG:
          imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG8;
          break;
        case GBRG:
          imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG8;
          break;
        case BGGR:
          imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR8;
          break;
        }
      }
    }
    else     // Mono camera or in pixel binned mode.
    {
      if(bitsPerPixel == 16)
      {
        imageEncoding = sensor_msgs::image_encodings::MONO16;
      }
      else if(bitsPerPixel==24)
      {
        imageEncoding = sensor_msgs::image_encodings::RGB8;
      }
      else
      {
        imageEncoding = sensor_msgs::image_encodings::MONO8;
      }
    }

    fillImage(image, imageEncoding, rawImage.GetRows(), rawImage.GetCols(), rawImage.GetStride(), rawImage.GetData());
    image.header.frame_id = frame_id;
  }
  else if(cam_.IsConnected())
  {
    throw CameraNotRunningException("PointGreyCamera::grabImage: Camera is currently not running.  Please start the capture.");
  }
  else
  {
    throw std::runtime_error("PointGreyCamera::grabImage not connected!");
  }
}

uint PointGreyCamera::getGain()
{
  return metadata_.embeddedGain >> 20;
}

uint PointGreyCamera::getShutter()
{
  return metadata_.embeddedShutter >> 20;
}

uint PointGreyCamera::getBrightness()
{
  return metadata_.embeddedTimeStamp >> 20;
}

uint PointGreyCamera::getExposure()
{
  return metadata_.embeddedBrightness >> 20;
}

uint PointGreyCamera::getWhiteBalance()
{
  return metadata_.embeddedExposure >> 8;
}

uint PointGreyCamera::getROIPosition()
{
  return metadata_.embeddedROIPosition >> 24;
}

void PointGreyCamera::setDesiredCamera(const uint32_t &id)
{
  serial_ = id;
}

void PointGreyCamera::setDesiredScene(const uint32_t &scene)
{
    scene_ = scene;
}

void PointGreyCamera::setDesiredFPS(const uint32_t &fps)
{
    fps_ = fps;
}

void PointGreyCamera::setDesiredHighRes(const uint32_t &high_res)
{
    high_res_ = high_res;
}

std::vector<uint32_t> PointGreyCamera::getAttachedCameras()
{
  std::vector<uint32_t> cameras;
  unsigned int num_cameras;
  Error error = busMgr_.GetNumOfCameras(&num_cameras);
  PointGreyCamera::handleError("PointGreyCamera::getAttachedCameras: Could not get number of cameras", error);
  for(unsigned int i = 0; i < num_cameras; i++)
  {
    unsigned int this_serial;
    error = busMgr_.GetCameraSerialNumberFromIndex(i, &this_serial);
    PointGreyCamera::handleError("PointGreyCamera::getAttachedCameras: Could not get get serial number from index", error);
    cameras.push_back(this_serial);
  }
  return cameras;
}

void PointGreyCamera::handleError(const std::string &prefix, const FlyCapture2::Error &error)
{
  if(error == PGRERROR_TIMEOUT)
  {
    throw CameraTimeoutException("PointGreyCamera: Failed to retrieve buffer within timeout.");
  }
  else if(error == PGRERROR_IMAGE_CONSISTENCY_ERROR)
  {
    throw CameraImageConsistencyError("PointGreyCamera: Image consistency error.");
  }
  else if(error != PGRERROR_OK)     // If there is actually an error (PGRERROR_OK means the function worked as intended...)
  {
    std::string start(" | FlyCapture2::ErrorType ");
    std::stringstream out;
    out << error.GetType();
    std::string desc(error.GetDescription());
    throw std::runtime_error(prefix + start + out.str() + " " + desc);
  }
}
