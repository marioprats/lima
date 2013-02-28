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

#ifndef RJOINT_H
#define RJOINT_H

#include <lima/limaconfig.h>
#ifdef _USE_ODE_
#include <RJointDynamicModel.h>
#endif
#include <visp/vpColVector.h>
#include <lima/Joint.h>

/**
 A revolute joint (rotational)

 @author Mario Prats <mprats@icc.uji.es>
 */
class RJoint : public Joint
{
public:
  vpColVector axis; ///< The rotational axis
  float min_value; ///< The minimum value for the joint
  float max_value; ///< The maximum value for the joint
  float current_value; ///< Current value of the joint

  /** Empty constructor */
  RJoint() :
      Joint()
  {
    current_value = 0;
    vpColVector ax(3);
    ax = 0;
    ax[1] = 1;
    setAxis(ax);
    setRange(0, M_PI);
  }
  /** Constructor from the rotational axis, range and friction
   axis must be a 3-component unitary vector, min_value and max_value
   are the minimum and maximum joint values (in degrees)
   */
  RJoint(vpColVector axis, float min_value, float max_value) :
      Joint()
  {
    cerr << "Rjoint constructor" << endl;
    current_value = 0;
    cerr << "setAxis" << endl;
    setAxis(axis);
    cerr << "setRange" << endl;
    setRange(min_value, max_value);
    cerr << "done" << endl;
  }

  /** Sets the rotational axis (axis must be a 3-component unitary vector) */
  void setAxis(vpColVector &axis)
  {
    this->axis = axis;
  }
  /** Sets the range for the joint, i.e, minimum and maximum joint values (in degrees) */
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
    return string("revolute");
  }

  vpHomogeneousMatrix getJointTransformation();

#ifdef _USE_ODE_
  void setDynamicModel(RJointDynamicModel *dmodel)
  { this->dynamic_model=dmodel;}
#endif

  ~RJoint()
  {
  }
};
#endif
