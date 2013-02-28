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

#ifndef _C1394Image_H
#define _C1394Image_H

#include <visp/vp1394TwoGrabber.h>
#include <pthread.h>
#include <visp/vpImage.h>

/** Class Image from a 1394 Camera
 * This class represents an image taken a Stereo VidereDesign Camera. At low level, it uses the svs library and ieee1394 interface
 */

enum
{
  COLOR1394, BN1394
};

class C1394Image : public vp1394TwoGrabber
{
  vp1394TwoGrabber *video;
  pthread_t visiont;
  pthread_mutex_t access;
  bool capture_init;
  bool run_thread;
  int mode;

  vpImage<vpRGBa> vIc;
  vpImage<unsigned char> vI;

public:
//		char lens;				///< 'R' for right or 'L' for left image

  C1394Image(int cam_mode = COLOR1394); ///< Constructor from a camera
  void acquire(vpImage<vpRGBa> & I); ///< Get image from capture
  void acquire(vpImage<unsigned char> & I);
  void capture(); ///< Capture an image
// 		void loadParams(char *filename)
  //	{video->loadParams(filename);}        ///< Load camera parameters from a file
  ~C1394Image(); ///< Destructor
};
#endif
