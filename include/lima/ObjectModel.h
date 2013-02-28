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

#ifndef OBJECTMODEL_H
#define OBJECTMODEL_H

#include <lima/limaconfig.h>
#include <lima/GeomModel.h>
#ifdef _USE_ODE_
#include <lima/BodyDynamicModel.h>
#endif

/**
 A 3D model of an object, including geometrical information, and also dynamics (in case of building with ODE support)
 @author Mario Prats <mprats@icc.uji.es>
 */
class ObjectModel
{
public:
  GeomModel *geometry; ///< The geometrical model
#ifdef _USE_ODE_
  BodyDynamicModel *dynamics; ///< The dynamic model 
#endif

  /** Empty constructor */
  ObjectModel();
  /** Constructor from the geometric model */
  ObjectModel(GeomModel *gmodel);

#ifdef _USE_ODE_
  /** Constructor from the geometric and dynamic model */
  ObjectModel(GeomModel *gmodel, BodyDynamicModel *dmodel)
  { setModels(gmodel,dmodel);}
#endif

  /** Sets the geometric model */
  void setGeometricModel(GeomModel *gmodel)
  {
    this->geometry = gmodel;
  }
#ifdef _USE_ODE_
  /** Sets the dynamic model */
  void setDynamicModel(BodyDynamicModel *dmodel)
  { this->dynamics=dmodel;}
  /** Sets the geometric and dynamic models */
  void setModels(GeomModel *gmodel, BodyDynamicModel *dmodel)
  { setGeometricModel(gmodel); setDynamicModel(dmodel);}
#endif
  ~ObjectModel();

};

#endif
