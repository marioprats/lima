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

#ifndef RJOINT2_H
#define RJOINT2_H

#include <lima/limaconfig.h>
#ifdef _USE_ODE_
#include <RJoint2DynamicModel.h>
#endif
#include <visp/vpColVector.h>
#include <lima/Joint.h>

/**
 A mechanism composed of two revolute joints which rotation axis are perpendicular.

 @author Mario Prats <mprats@icc.uji.es>
 */
class RJoint2 : public Joint
{
public:
public:
  vpColVector axis1; ///< The first rotational axis
  float min_value1; ///< The minimum value for the joint 1
  float max_value1; ///< The maximum value for the joint 1 
  float current_value1; ///< Current value of the joint 1
  vpColVector axis2; ///< The second rotational axis
  float min_value2; ///< The minimum value for the joint 2
  float max_value2; ///< The maximum value for the joint 2
  float current_value2; ///< Current value of the joint 2

  /** Constructor from the rotational axis, range and friction
   axis must be a 3-component unitary vector, min_value and max_value
   are the minimum and maximum joint values (in degrees). All values must
   be specified, first for joint1 and after for joint2
   */
  RJoint2(vpColVector axis1, float min_value1, float max_value1, vpColVector axis2, float min_value2, float max_value2) :
      Joint()
  {
    current_value1 = 0;
    setAxis1(axis1);
    setRange1(min_value1, max_value1);
    current_value2 = 0;
    setAxis2(axis2);
    setRange2(min_value2, max_value2);
  }

  /** Sets the first rotational axis (axis must be a 3-component unitary vector) */
  void setAxis1(vpColVector &axis)
  {
    this->axis1 = axis;
  }
  /** Sets the second rotational axis (axis must be a 3-component unitary vector) */
  void setAxis2(vpColVector &axis)
  {
    this->axis2 = axis;
  }
  /** Sets the range for the first joint, i.e, minimum and maximum joint values (in degrees) */
  void setRange1(float min_value, float max_value)
  {
    this->min_value1 = min_value;
    this->max_value1 = max_value;
  }
  /** Sets the range for the second joint, i.e, minimum and maximum joint values (in degrees) */
  void setRange2(float min_value, float max_value)
  {
    this->min_value2 = min_value;
    this->max_value2 = max_value;
  }

  /** Sets the current first joint value */
  void setValue1(float value)
  {
    current_value1 = value;
  }
  /** Sets the current second joint value */
  void setValue2(float value)
  {
    current_value2 = value;
  }

  /** Adds a quantity to the first current joint value */
  void addValue1(float value)
  {
    current_value1 += value;
  }
  /** Adds a quantity to the second current joint value */
  void addValue2(float value)
  {
    current_value2 += value;
  }

  string getType()
  {
    return string("revolute2");
  }

  vpHomogeneousMatrix getJointTransformation();

#ifdef _USE_ODE_
  void setDynamicModel(RJoint2DynamicModel *dmodel)
  { this->dynamic_model=dmodel;}
#endif

  ~RJoint2()
  {
  }
};

#endif
