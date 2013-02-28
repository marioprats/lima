/*
 * Copyright (c) 2007 University of Jaume-I, 2013 Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mario Prats
 */

#ifndef ROSGRABBER_H
#define ROSGRABBER_H

#include <lima/Camera.h>

#include <visp/vpImage.h>
#include <visp/vpFrameGrabber.h>
#include <visp/vpCameraParameters.h>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include <boost/shared_ptr.hpp>

/** Framegrabber for the simulator camera
 */
class ROSGrabber: public Camera
{
  std::string image_topic, info_topic;
  image_transport::ImageTransport *it;
  image_transport::Subscriber image_sub;
  ros::Subscriber image_info_sub;
  int subsample_;
  bool ready_;    //true if images have been acquired

public:
  vpImage<vpRGBa> image;        ///< image data
  std::string frame_id;         ///< tf optical frame name
  ros::Time stamp;              ///< timestamp of the image

  /** Constructor from the server and port where the simulator is listening */
  ROSGrabber(ros::NodeHandle &nh, std::string image_topic, std::string info_topic, int subsample=1);

  virtual void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  virtual void imageInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  virtual void open(vpImage<unsigned char> &I) {} ///< Initialize image (set size)
  virtual void open(vpImage<vpRGBa> &I) {}    ///< Initialize image (set size)

  virtual void acquire(vpImage<unsigned char> &I);    ///< B/W image capture
  virtual void acquire(vpImage<vpRGBa> &I);           ///< Color image capture

  virtual void close();

  bool ready() {return ready_;}

  ~ROSGrabber();
};
typedef boost::shared_ptr<ROSGrabber> ROSGrabberPtr;

#endif
