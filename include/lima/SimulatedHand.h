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

#ifndef SIMULATEDHAND_H
#define SIMULATEDHAND_H

#include <lima/limaconfig.h>
#include <lima/Hand.h>
#include <lima/Object.h>
#include <lima/HandPreshape.h>
#include <lima/HandFrame.h>

/**
 A simulated hand

 @author Mario Prats <mprats@icc.uji.es>
 */
class SimulatedHand : public Hand
{
public:
  Object *object; ///< A pointer to the object, containing the hand model

  SimulatedHand();

  /** Gets the name of the hand */
  virtual string getName()=0;
  /** Sets the object attached to the simulated hand (which includes the model) */
  void setObject(Object *o)
  {
    this->object = o;
  }
  /** Moves the hand to the given preshape */
  virtual int executePreshape(HandPreshape *p)=0;

protected:
  /** Creates or loads the hand object including geometrical (and possibly dynamical) model */
  virtual Object *createObject()=0;
  /** Computes the matrix hMhf from the desired HandFrame object 'handframe' */
  virtual int computeHandFrameMatrix(HandFrame *handframe, vpHomogeneousMatrix &hMhf)=0;
  /** Sends suitable control signals to the hand. All hands must implement this method */
  virtual int sendControl()=0;

  ~SimulatedHand();

};

#endif
