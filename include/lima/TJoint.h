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

#ifndef TJOINT_H
#define TJOINT_H

#include <lima/limaconfig.h>
#ifdef _USE_ODE_
#include <TJointDynamicModel.h>
#endif
#include <visp/vpColVector.h>
#include <lima/Joint.h>

/**
 A prismatic joint (translational)

 @author Mario Prats <mprats@icc.uji.es>
 */
class TJoint : public Joint
{
public:
  vpColVector axis; ///< The translational axis
  float min_value; ///< The minimum value for the joint
  float max_value; ///< The maximum value for the joint
  float current_value; ///< Current value of the joint

  /** Empty constructor */
  TJoint();
  /** Constructor from the translational axis, range and friction
   axis must be a 3-component unitary vector, min_value and max_value
   are the minimum and maximum joint values (in mm)
   */
  TJoint(vpColVector axis, float min_value, float max_value);

  /** Sets the translational axis (axis must be a 3-component unitary vector) */
  void setAxis(vpColVector &axis)
  {
    this->axis = axis;
  }
  /** Sets the range for the joint, i.e, minimum and maximum joint values (in mm) */
  void setRange(float min_value, float max_value)
  {
    this->min_value = min_value;
    this->max_value = max_value;
  }

  /** Sets the current joint value */
  void setValue(float value)
  {
    current_value = value;
  }
  /** Adds a quantity to the current joint value */
  void addValue(float value)
  {
    current_value += value;
  }

  string getType()
  {
    return string("prismatic");
  }

  vpHomogeneousMatrix getJointTransformation();

#ifdef _USE_ODE_
  void setDynamicModel(TJointDynamicModel *dmodel)
  { this->dynamic_model=dmodel;}
#endif

  ~TJoint();
};
#endif
