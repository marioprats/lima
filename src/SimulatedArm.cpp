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

#include <lima/SimulatedArm.h>
#include <lima/Joint.h>
#include <lima/FJoint.h>
#ifdef _USE_ODE
#include <lima/FJointDynamicModel.h>
#endif

SimulatedArm::SimulatedArm() :
    Arm()
{
  object = NULL;
  force_sensor = NULL;
  hand = NULL;
}

void SimulatedArm::addForceSensor(SimulatedForceSensor *fs, vpHomogeneousMatrix eMf)
{
  force_sensor = fs;
  this->eMf = eMf;
  if (hand == NULL)
  {
    //get the last object in the hierarchy
    Object *lastlink = object;
    while (lastlink->linkedTo.size() > 0)
      lastlink = lastlink->linkedTo[0]->next;

    //Attach the force sensor to it via a fixed joint
    FJoint *qh = new FJoint();
#ifdef _USE_ODE_
    FJointDynamicModel *qh_dmodel=new FJointDynamicModel();
    qh->setDynamicModel(qh_dmodel);
#endif	
    vpHomogeneousMatrix I;
    I.setIdentity();
    lastlink->linkTo(eMf, qh, I, fs->object);
  }
  else
  {
    cerr << "SimulatedArm::addForceSensor ERROR: please, add the force sensor before adding the hand" << endl;
  }
}

void SimulatedArm::addHand(SimulatedHand *h, vpHomogeneousMatrix eMh)
{
  hand = h;
  this->eMh = eMh;

  //get the last object in the hierarchy
  Object *lastlink = object;
  while (lastlink->linkedTo.size() > 0)
    lastlink = lastlink->linkedTo[0]->next;

  //Attach the hand to it via a fixed joint
  FJoint *qh = new FJoint();
#ifdef _USE_ODE_
  FJointDynamicModel *qh_dmodel=new FJointDynamicModel();
  qh->setDynamicModel(qh_dmodel);
#endif	
  vpHomogeneousMatrix I;
  I.setIdentity();
  if (force_sensor != NULL)
  {
    //Attach the hand to the force sensor
    lastlink->linkTo(eMf.inverse() * eMh, qh, I, h->object);
  }
  else
  {
    //Attach the hand to the end-effector
    lastlink->linkTo(eMh, qh, I, h->object);
  }
}

SimulatedArm::~SimulatedArm()
{
}

