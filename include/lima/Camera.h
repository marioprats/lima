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

#ifndef CAMERA_H
#define CAMERA_H

#include <lima/limaconfig.h>
#include <visp/vpImage.h>
#include <visp/vpFrameGrabber.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpHomogeneousMatrix.h>
#include <lima/Scene.h>

//class CameraObjectPose;

/**
 An abstract class holding the camera interface

 @author Mario Prats <mprats@icc.uji.es>
 */
class Camera : public vpFrameGrabber
{
public:
  Scene *scene; ///< The scene to capture
  unsigned int width, height; ///< Image size
  float cx, cy, cz; ///< point where the camera looks at (x,y,z)
  float tx, ty, tz; ///< camera position (x,y,z)
  vpHomogeneousMatrix wMc; ///< Camera pose in world coordinates

  vpCameraParameters cparams; ///< The camera intrinsic parameters

  Camera() {}

  Camera(vpCameraParameters params)
  {
    this->cparams = params;
  }

  virtual void open(vpImage<unsigned char> &I); ///< Initialize image (set size)
  virtual void open(vpImage<vpRGBa> &I); ///< Initialize image (set size)

  virtual void acquire(vpImage<unsigned char> &I)=0; ///< B/W image capture
  virtual void acquire(vpImage<vpRGBa> &I)=0; ///< Color image capture

  virtual void close()=0;
};

#endif
