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

#include <lima/VirtualCamera.h>

VirtualCamera::VirtualCamera(Scene *scene, int width, int height) :
    Camera(vpCameraParameters())
{
  this->width = width;
  this->height = height;
  this->scene = scene;

  //Default camera position is (0,0,10) looking at (0,0,0)
  tx = ty = 0;
  tz = 10;
  cx = cy = cz = 0;
}

/** Makes the camera look to the point (cx,cy,cz) */
void VirtualCamera::lookAt(float cx, float cy, float cz)
{
  this->cx = cx;
  this->cy = cy;
  this->cz = cz;
}

/** Sets the camera position to the point (tx,ty,tz) */
void VirtualCamera::setCameraPosition(float tx, float ty, float tz)
{
  this->tx = tx;
  this->ty = ty;
  this->tz = tz;
}

/** Translates current camera position by (dtx,dty,dtz) */
void VirtualCamera::translate(float dtx, float dty, float dtz)
{
  tx += dtx;
  ty += dty;
  tz += dtz;
}

// void VirtualCamera::close() {
// }

VirtualCamera::~VirtualCamera()
{
}
