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

#ifndef HAND_H
#define HAND_H

#include <lima/limaconfig.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpColVector.h>
#include <lima/HandPreshape.h>
#include <lima/HandFrame.h>
#include <lima/TOGPrimitive.h>

/**
 A hand, either simulated or real
 At this moment, only position control control of the hand is implemented, which is the most
 common in current hands.

 @author Mario Prats <mprats@icc.uji.es>
 */
class Hand
{
public:
  vpHomogeneousMatrix hMhf; ///< The Hand Frame pose w.r.t the hand origin frame
  HandFrame *handframe; ///< The desired hand frame object
  vpColVector dq; ///< Desired joint values

  Hand();

  /** Gets the name of the hand */
  virtual string getName()=0;
  /** Moves the hand to the given preshape */
  virtual int executePreshape(HandPreshape *p)=0;
  /** Sets the hand frame hMhf to the given one */
  int setHandFrame(HandFrame *h);
  /** Adopts the given TOG primitive with the hand */
  int executeTOGPrimitive(TOGPrimitive *p);
  /** Sets the desired joint positions */
  void setPosition(vpColVector dq)
  {
    this->dq = dq;
  }
  /** Performs the hand control, in order to reach the desired joint positions dq,
   and updates hMhf transformation according to hand motion. */
  void control()
  {
    sendControl();
    if (handframe != NULL)
      computeHandFrameMatrix(handframe, hMhf);
  }

  ~Hand();

protected:
  /** Computes the matrix hMhf from the desired HandFrame object 'handframe' */
  virtual int computeHandFrameMatrix(HandFrame *handframe, vpHomogeneousMatrix &hMhf)=0;
  /** Sends suitable control signals to the hand. All hands must implement this method */
  virtual int sendControl()=0;
};

#endif
