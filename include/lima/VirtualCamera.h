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

#ifndef VIRTUALCAMERA_H
#define VIRTUALCAMERA_H

#include <lima/limaconfig.h>
#include <visp/vpImage.h>
#include <visp/vpFrameGrabber.h>
#include <lima/Camera.h>
#include <lima/Scene.h>

/**
 A virtual camera for capturing the virtual scene

 @author Mario Prats <mprats@icc.uji.es>
 */
class VirtualCamera : public Camera
{
public:
  VirtualCamera(Scene *scene, int width, int height);

  /** Makes the camera look to the point (cx,cy,cz) */
  void lookAt(float cx, float cy, float cz);
  /** Sets the camera position to the point (tx,ty,tz) */
  void setCameraPosition(float tx, float ty, float tz);
  /** Translates current camera position by (dtx,dty,dtz) */
  void translate(float dtx, float dty, float dtz);

  virtual void acquire(vpImage<unsigned char> &I)=0; ///< B/W image capture
  virtual void acquire(vpImage<vpRGBa> &I)=0; ///< Color image capture

  virtual void close()=0;

  ~VirtualCamera();

};

#endif
