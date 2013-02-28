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

#include <lima/ROSGrabber.h>
#include <visp/vpImageConvert.h>

ROSGrabber::ROSGrabber(ros::NodeHandle &nh, std::string image_topic, std::string info_topic, int subsample) {
  it=new image_transport::ImageTransport(nh);

  image_sub=it->subscribe(image_topic, 1, &ROSGrabber::imageCallback, this);
  image_info_sub=nh.subscribe<sensor_msgs::CameraInfo>(info_topic, 1, &ROSGrabber::imageInfoCallback, this);
  ready_=false;
  subsample_=subsample;
}

void ROSGrabber::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  //Receive image, convert to ViSP, store in a class attribute
  this->width=msg->width/subsample_;
  this->height=msg->height/subsample_;
  frame_id = msg->header.frame_id;
  stamp = msg->header.stamp;

  if (image.getCols()!=width || image.getRows()!=height)
    image.resize(height, width);

  if (msg->encoding==std::string("bgr8")) {
    for (unsigned int r=0;r<height; r++)
      for (unsigned int c=0; c<width; c++) {
        int index=(width*subsample_)*r*subsample_*3+c*subsample_*3;
        image[r][c].R=msg->data[index+2];
        image[r][c].G=msg->data[index+1];
        image[r][c].B=msg->data[index];
      }
  } else if (msg->encoding==std::string("mono8")) {
    for (unsigned int r=0;r<height; r++)
      for (unsigned int c=0; c<width; c++) {
        int index=(width*subsample_)*r+c*subsample_;
        image[r][c].R=msg->data[index];
        image[r][c].G=msg->data[index];
        image[r][c].B=msg->data[index];
      }
  } else {
    //Assume rgb8
    for (unsigned int r=0;r<height; r++)
      for (unsigned int c=0; c<width; c++) {
        int index=(width*subsample_)*r*subsample_*3+c*subsample_*3;
        image[r][c].R=msg->data[index];
        image[r][c].G=msg->data[index+1];
        image[r][c].B=msg->data[index+2];
      }
  }

  ready_=true;
}


void ROSGrabber::imageInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
  //Receive image info, store in a class attribute
  int binx=(msg->binning_x == 0) ? 1 : msg->binning_x;
  int biny=(msg->binning_y ==0) ? 1 : msg->binning_y;
  cparams.initPersProjWithoutDistortion (msg->P[0]/(binx*subsample_), msg->P[5]/(biny*subsample_), msg->P[2]/(binx*subsample_), msg->P[6]/(biny*subsample_));
}


void ROSGrabber::acquire(vpImage<unsigned char> &I) {
  vpImageConvert::convert(image,I);
}

void ROSGrabber::acquire(vpImage<vpRGBa> &I) {
  I=image;
}

void ROSGrabber::close() {}

ROSGrabber::~ROSGrabber() {
  if (it!=NULL) delete it;
}
