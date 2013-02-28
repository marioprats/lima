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

#include <lima/BBoxDynamicModel.h>
#ifdef _USE_ODE_
#include <iostream>

BBoxDynamicModel::BBoxDynamicModel():BodyDynamicModel()
{
  dimensions.resize(3);
  dimensions=0;
  mass=0;
  affectedByGravity(true);
}

BBoxDynamicModel::BBoxDynamicModel(vpColVector dimensions, float mass):BodyDynamicModel()
{
  setDimensions(dimensions);
  setMass(mass);
  affectedByGravity(true);
}

BBoxDynamicModel::BBoxDynamicModel(float dx, float dy, float dz, float mass): BodyDynamicModel()
{
  setDimensions(dx,dy,dz);
  setMass(mass);
  affectedByGravity(true);
}

void BBoxDynamicModel::createBody(dWorldID *world)
{
  body = dBodyCreate(*world);
  dMass m;
  dMassSetBox(&m,1.0f,dimensions[0],dimensions[1],dimensions[2]);
  dMassAdjust(&m,mass);
  dBodySetMass(body,&m);
  dBodySetAutoDisableFlag(body, 1);
  dBodySetGravityMode(body,affected_by_gravity);
  body_initialized=true;
}

void BBoxDynamicModel::createGeom(dSpaceID *space)
{
  geom = dCreateBox(*space,dimensions[0],dimensions[1],dimensions[2]);
  if (body_initialized)
  {
    dGeomSetBody(geom,body);
  }
  geom_initialized=true;
}

BBoxDynamicModel::~BBoxDynamicModel()
{
}
#endif
