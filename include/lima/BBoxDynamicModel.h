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

#ifndef BBOXDYNAMICMODEL_H
#define BBOXDYNAMICMODEL_H

#include <lima/limaconfig.h>
#ifdef _USE_ODE_
#include <BodyDynamicModel.h>
#include <ode/ode.h>
#include <visp/vpColVector.h>

/**
 The ODE dynamic model for a bounding box

 @author Mario Prats <mprats@icc.uji.es>
 */
class BBoxDynamicModel : public BodyDynamicModel
{
public:
  vpColVector dimensions; ///< Dimensions of the bounding box (X,Y,Z, units in mm)
  float mass;///< Mass of the object.

  /** Empty constructor */
  BBoxDynamicModel();

  /** Constructor from a vector of dimensions (X,Y,Z, units in mm) and mass, units in Kg*/
  BBoxDynamicModel(vpColVector dimensions, float mass);

  /** Constructor from the dimensions (units in mm) and mass (units in Kg) */
  BBoxDynamicModel(float dx, float dy, float dz, float mass);

  /** Sets the dimensions of the bounding box (units in mm) */
  void setDimensions(vpColVector &dimensions)
  { this->dimensions=dimensions;}
  /** Sets the dimensions of the bounding box (units in mm) */
  void setDimensions(float dx, float dy, float dz)
  { vpColVector d(3); d[0]=dx; d[1]=dy; d[2]=dz; setDimensions(d);}
  /** Sets the mass of the object */
  void setMass(float mass)
  { this->mass=mass;}

  /** Creates the ODE body from the parameters */
  void createBody(dWorldID *world);
  /** Creates an ODE geom from the parameters */
  void createGeom(dSpaceID *space);

  ~BBoxDynamicModel();

};
#endif
#endif
