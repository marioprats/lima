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

#ifndef PERFECTHAND_H
#define PERFECTHAND_H

#include <lima/limaconfig.h>
#include <lima/SimulatedHand.h>
#include <lima/HandPreshape.h>
#include <lima/PerfectHandHook.h>
#include <lima/PerfectHandCylindrical.h>
#include <lima/PerfectHandOneFinger.h>
#include <lima/PerfectHandLateral.h>
#include <lima/HandFrame.h>
#include <lima/PerfectHandFrame.h>
#include <lima/PerfectHandFingertip.h>
#include <lima/PerfectHandFingertipToThumb.h>
#include <lima/PerfectHandMPhalanx.h>
#include <lima/PerfectHandOPhalanx.h>
#include <lima/PerfectHandPalm.h>
#include <string>

using namespace std;

/**
 The perfect hand. All the planning algorithms are implemented for this hand. To make them work on another hand, a map between the perfect hand and the new hand must be established. We assume that this hand can do everything ;)

 @author Mario Prats <mprats@icc.uji.es>
 */
class PerfectHand : public SimulatedHand
{
public:
  typedef enum
  {
    LEFT, RIGHT
  } handtype;
  handtype hand_side;
  PerfectHand(handtype side);

  /** Gets the name of the hand */
  string getName()
  {
    return string("PerfectHand");
  }

  /** Sets desired joint values for each finger */
  void setPosition(vpColVector &f1, vpColVector &f2, vpColVector &f3, vpColVector &f4, vpColVector &f5);

  /** Sets desired joint values for each finger */
  void setPosition(float f11, float f12, float f13, float f21, float f22, float f23, float f31, float f32, float f33,
                   float f41, float f42, float f43, float f51, float f52, float f53, float f54);

  /** Moves the hand to the given preshape */
  int executePreshape(HandPreshape *p);

  /** Moves the hand to a hook preshape */
  int executePreshape(PerfectHandHook *p);

  /** Moves the hand to a precission preshape */
  int executePreshape(PerfectHandCylindrical *p);

  /** Moves the hand to a one finger preshape */
  int executePreshape(PerfectHandOneFinger *p);

  /** Moves the hand to a lateral preshape */
  int executePreshape(PerfectHandLateral *p);

  ~PerfectHand();

protected:
  /** Creates or loads the hand object including geometrical (and possibly dynamical) model */
  Object *createObject();

  /** Computes the matrix hMhf from the desired HandFrame object 'handframe'*/
  int computeHandFrameMatrix(HandFrame *handframe, vpHomogeneousMatrix &hMhf);

  /** Sets the hand frame hMhf to a fingertip frame */
  int computeHandFrameMatrix(PerfectHandFingertip *p, vpHomogeneousMatrix &hMhf);

  /** Sets the hand frame hMhf to a fingertip, with Z axis pointing to the thumb */
  int computeHandFrameMatrix(PerfectHandFingertipToThumb *p, vpHomogeneousMatrix &hMhf);

  /** Sets the hand frame hMhf to an outer phalanx frame */
  int computeHandFrameMatrix(PerfectHandOPhalanx *p, vpHomogeneousMatrix &hMhf);

  /** Sets the hand frame hMhf to a middle phalanx frame */
  int computeHandFrameMatrix(PerfectHandMPhalanx *p, vpHomogeneousMatrix &hMhf);

  /** Sets the hand frame hMhf to a palm frame */
  int computeHandFrameMatrix(PerfectHandPalm *p, vpHomogeneousMatrix &hMhf);

  /** Sends suitable control signals to the PerfectHand */
  int sendControl();
};

#endif
