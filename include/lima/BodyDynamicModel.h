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

#ifndef BODYDYNAMICMODEL_H
#define BODYDYNAMICMODEL_H

#include <lima/limaconfig.h>

#ifdef _USE_ODE_
#include <ode/ode.h>
#include <visp/vpHomogeneousMatrix.h>

/**
 A dynamic model of an object
 @author Mario Prats <mprats@icc.uji.es>
 */
class BodyDynamicModel
{
public:
  dBodyID body; ///< The ODE body
  bool body_initialized;///< Whether the ODE body has been initialized or not
  dGeomID geom;///< The ODE geom
  bool geom_initialized;///< Whether the ODE geom has been initialized or not
  bool affected_by_gravity;///< Whether the body is affected by gravity or not

  BodyDynamicModel();

  /** Sets the pose of the ODE body and geom in world frame */
  void setPose(vpHomogeneousMatrix &wMo);

  /** Gets the pose of the ODE body and geom in world frame */
  vpHomogeneousMatrix getPose();

  /** Fix the ODE body in the environment, so that it is not affected by external forces */
  void fixBody(dWorldID *world);

  /** Sets whether the body is affected by gravity or not */
  void affectedByGravity(bool flag)
  { this->affected_by_gravity=flag;}

  /** Creates an ODE body. All subclasses must implement this method */
  virtual void createBody(dWorldID *world)=0;
  /** Creates an ODE geom. All subclasses must implement this method */
  virtual void createGeom(dSpaceID *space)=0;

  ~BodyDynamicModel();

};
#endif
#endif
