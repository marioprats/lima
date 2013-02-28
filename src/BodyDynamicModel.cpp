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

#include <lima/BodyDynamicModel.h>
#include <iostream>

using namespace std;

#ifdef _USE_ODE_

BodyDynamicModel::BodyDynamicModel()
{
  body_initialized=geom_initialized=false;
}

void BodyDynamicModel::setPose(vpHomogeneousMatrix &wMo)
{
  if (geom_initialized)
  {
    dGeomSetPosition(geom,wMo[0][3],wMo[1][3],wMo[2][3]);
    dMatrix3 m;
    m[0]=wMo[0][0]; m[1]=wMo[0][1]; m[2]=wMo[0][2];
    m[4]=wMo[1][0]; m[5]=wMo[1][1]; m[6]=wMo[1][2];
    m[8]=wMo[2][0]; m[9]=wMo[2][1]; m[10]=wMo[2][2];
    m[3]=0; m[7]=0; m[11]=0;
    dGeomSetRotation(geom,m);
  }
  else
  {
    cerr << "BodyDynamicModel::setPose ERROR: body attribute not initialized" << endl;
  }
}

vpHomogeneousMatrix BodyDynamicModel::getPose()
{
  vpHomogeneousMatrix wMo;
  if (geom_initialized)
  {
    const dReal *p = dGeomGetPosition(geom);
    const dReal *r = dGeomGetRotation(geom);
    wMo[0][0]=r[0]; wMo[0][1]=r[1]; wMo[0][2]=r[2]; wMo[0][3]=p[0];
    wMo[1][0]=r[4]; wMo[1][1]=r[5]; wMo[1][2]=r[6]; wMo[1][3]=p[1];
    wMo[2][0]=r[8]; wMo[2][1]=r[9]; wMo[2][2]=r[10]; wMo[2][3]=p[2];
    wMo[3][0]=0; wMo[3][1]=0; wMo[3][2]=0; wMo[3][3]=1;
  }
  else
  {
    cerr << "BodyDynamicModel::setPose ERROR: body attribute not initialized" << endl;
  }
  return wMo;
}

void BodyDynamicModel::fixBody(dWorldID *world)
{
  if (body_initialized)
  {
    dJointID fj=dJointCreateFixed(*world,0);
    dJointAttach(fj,0,body);
    dJointSetFixed(fj);
  }
  else
  {
    cerr << "BodyDynamicModel::fixBody ERROR: body not initialized. Make sure you have added the object to the scene before calling fixBody" << endl;
  }
}

BodyDynamicModel::~BodyDynamicModel()
{
}

#endif
