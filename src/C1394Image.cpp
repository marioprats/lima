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

#include <lima/C1394Image.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
using namespace std;
//#include <pthread.h>

void *vision(void *arg)
{
  ((C1394Image*)arg)->capture();
  return 0;
}

C1394Image::C1394Image(int cam_mode)
{
  mode = cam_mode;

  video = new vp1394TwoGrabber();

  try
  {
    if (mode == COLOR1394)
      video->open(vIc);
    else
      video->open(vI);
  }
  catch (...)
  {
    cerr << "The program was stopped..." << endl;
    exit(-1);
  }

  unsigned int cameras;
  video->getNumCameras(cameras);

  cerr << endl;
  cerr << "Number of cameras on the bus: " << cameras << endl;
  cerr << endl;

  if (mode == COLOR1394)
  {
    video->setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_YUV422);
    video->setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);
  }
  else
  {
    video->setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    video->setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);
  }

  /* TODO and FIXME
   for (unsigned int i=0; i < cameras; i++) {
   video->setCamera(i);
   video->getFormat(act_format);
   video->getMode(act_mode);
   video->getFramerate(act_framerate);
   video->getShutter(min_shutter, act_shutter, max_shutter);
   video->getGain(min_gain, act_gain, max_gain);

   cerr << "Actual camera settings for camera: " << i << endl;
   cerr << "  Format: " << act_format	 << " (" << video->convertFormat(act_format) << ")" << endl;
   cerr << "  Mode: " << act_mode	 << " (" << video->convertMode(act_mode) << ")" << endl;
   cerr << "  Framerate: " << act_framerate	 << " (" << video->convertFramerate(act_framerate) << ")" << endl;
   cerr << "  Shutter: " << act_shutter << endl;
   cerr << "  Min shutter: " << min_shutter << endl;
   cerr << "  Max shutter: " << max_shutter << endl;
   cerr << "  Gain: " << act_gain << endl;
   cerr << "  Min gain: " << min_gain << endl;
   cerr << "  Max gain: " << max_gain << endl;
   cerr << endl;
   }
   cerr << endl;
   */

//	pthread_mutex_init(&access,NULL);

  capture_init = false;
  run_thread = true;
  cerr << "thread_create" << endl;
  pthread_create(&visiont, NULL, vision, this);
  cerr << "ok" << endl;
}

void C1394Image::capture()
{
  while (run_thread)
  {
    if (mode == COLOR1394)
      video->acquire(vIc); //image capture is done parallelized
    else
      video->acquire(vI); //image capture is done parallelized
    capture_init = true;
  }
  pthread_exit(NULL);
}

void C1394Image::acquire(vpImage<vpRGBa> & I)
{
  I = vIc;
}
void C1394Image::acquire(vpImage<unsigned char> & I)
{
  I = vI;
}

C1394Image::~C1394Image()
{
  run_thread = false;
  pthread_join(visiont, NULL);
}
