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

#ifndef PERFECTHANDFINGERTIPTOTHUMB_H
#define PERFECTHANDFINGERTIPTOTHUMB_H

#include <lima/limaconfig.h>
#include <lima/PerfectHandFrame.h>

/**
 A hand frame which is located in a fingertip so that Z direction is pointing towards the thumb fingertip. Useful for pinch grasps.

 @author Mario Prats <mprats@icc.uji.es>
 */
class PerfectHandFingertipToThumb : public PerfectHandFrame
{
public:
  unsigned short finger; ///< The finger number where to put the frame (from 0 to 4)

  PerfectHandFingertipToThumb();

  /** Returns the name of the frame. It returns "PerfectHandFingertipToThumb" */
  string getFrameName()
  {
    return string("PerfectHandFingertipToThumb");
  }

  /** Specifies the finger where to put the frame (from 0 to 3) */
  void setFinger(unsigned short finger)
  {
    this->finger = finger;
  }

  ~PerfectHandFingertipToThumb();

};

#endif
