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

#include <lima/SimulatedForceSensor.h>
#include <lima/CylinderModel.h>
#include <lima/BBoxDynamicModel.h>
#include <lima/FJointDynamicModel.h>
#include <lima/ObjectModel.h>
#include <lima/Joint.h>

#include <visp/vpRGBa.h>

SimulatedForceSensor::SimulatedForceSensor() :
    ForceSensor()
{
  createForceSensorObject();

  history_i[0].resize(6);
  history_i[1].resize(6);
  history_i[0] = history_i[1] = 0;
  history_o[0].resize(6);
  history_o[1].resize(6);
  history_o[0] = history_o[1] = 0;
  initialized = false;
}

void SimulatedForceSensor::createForceSensorObject()
{
  vpRGBa blue(50, 50, 255, 0);
  CylinderModel *o_gmodel = new CylinderModel(0.015, 0.015, 0.02, blue);
#ifdef _USE_ODE_
  BBoxDynamicModel *o_dmodel=new BBoxDynamicModel(0.03,0.03,0.02,0.01);
  o_dmodel->affectedByGravity(false);
  ObjectModel *o_model=new ObjectModel(o_gmodel,o_dmodel);
#else
  ObjectModel *o_model = new ObjectModel(o_gmodel);
#endif
  object = new Object("Force sensor", o_model);
}

bool SimulatedForceSensor::init()
{
  if (object->linkedTo.size() == 1)
  {
    //A tool is attached to the force sensor. Initialize joint feedback
#ifdef _USE_ODE_
    dJointSetFeedback(((FJointDynamicModel*)object->linkedTo[0]->dynamic_model)->joint, &jfeed);
#endif
    initialized = true;
  }
  else
  {
    cerr << "SimulatedForceSensor::init() ERROR: No object is attached to the force sensor" << endl;
    initialized = false;
  }
  return initialized;
}

vpColVector SimulatedForceSensor::getForce()
{
  vpColVector f(6);
  f = 0;
  if (!initialized)
  {
    init();
  }

#ifdef _USE_ODE_
  dJointFeedback *jf= dJointGetFeedback( ((FJointDynamicModel*)object->linkedTo[0]->dynamic_model)->joint);
  if (jf!=NULL)
  {
    f[0]=jf->f1[0]; //force in world frame
    f[1]=jf->f1[1];
    f[2]=jf->f1[2];
    f[3]=jf->t1[0];
    f[4]=jf->t1[1];
    f[5]=jf->t1[2];

    cerr << "low level force feedback: " << f.t() << endl;

    vpHomogeneousMatrix wMf=object->getPosition();
    vpRotationMatrix fRw;
    wMf.inverse().extract(fRw);
    vpMatrix fRw6(6,6);
    for (int i=0; i< 3; i++)
    for ( int j=0; j<3; j++)
    {
      fRw6[i][j]=fRw[i][j];
      fRw6[i+3][j+3]=fRw[i][j];
    }
    f=fRw6*f; // force in force sensor frame

    //Chebishev filter order 2
    vpColVector ee=f;
    history_i[0]=f;
    for (int i=0; i<6;i++)
    {
      f[i]=1.5223*history_o[0][i]-0.6427*history_o[1][i]+0.0284*f[i]+0.0568*history_i[0][i]+0.0284*history_i[1][i];
    }
    history_o[1]=history_o[0];
    history_o[0]=f;
    history_i[1]=history_i[0];
    history_i[0]=ee;
  }
  else
  {
    f=0;
    cerr << "SimulatedForceSensor::getForce() ERROR: Cannot read force. Force sensor not correctly initialized" << endl;
  }
#endif
  return f;
}

SimulatedForceSensor::~SimulatedForceSensor()
{
}

