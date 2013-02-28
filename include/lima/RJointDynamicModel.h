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

#ifndef RJOINTDYNAMICMODEL_H
#define RJOINTDYNAMICMODEL_H

#include <lima/limaconfig.h>
#ifdef _USE_ODE_
#include <JointDynamicModel.h>
#include <visp/vpColVector.h>
#include <ode/ode.h>

/**
 The dynamic model for a revolute joint (rotational)

 @author Mario Prats <mprats@icc.uji.es>
 */
class RJointDynamicModel : public JointDynamicModel
{
public:
  vpColVector axis; ///< The rotational axis
  float min_value;///< The minimum value for the joint
  float max_value;///< The maximum value for the joint
  float joint_offset;///< The initial value of the joint
  float friction;///< The friction for the joint
  bool joint_configured;///< Whether the joint has been already configured or not
  bool has_motor;///< Whether the joint has a motor attached or not
  float velocity;///< If the joint has a motor, this is the desired velocity
  float max_torque;///< The maximum torque that the motor can apply

  /** Empty constructor */
  RJointDynamicModel();
  /** Constructor from the rotational axis, range and friction
   axis must be a 3-component unitary vector, min_value and max_value
   are the minimum and maximum joint values (in radians)
   */
  RJointDynamicModel(vpColVector axis, float min_value, float max_value, float friction);

  /** Sets the rotational axis (axis must be a 3-component unitary vector) */
  void setAxis(vpColVector &axis)
  { this->axis=axis;}
  /** Sets the range for the joint, i.e, minimum and maximum joint values (in radians) */
  void setRange(float min_value, float max_value)
  { this->min_value=min_value; this->max_value=max_value;}
  /** Sets the friction for the joint */
  void setFriction(float friction);

  /** Configures the ODE parameters of the joint */
  void configureJoint(Joint *j);

  /** Updates the joint object according to the current state of the ODE simulation */
  void updateJointBySimulation(Joint *j);

  /** Creates the ODE joint from the parameters */
  void createJoint(dWorldID *world);

  /** Sets a motor in the joint which can introduce a maximum torque of 'max_torque'*/
  void hasMotor(float max_torque);
  /** Sets the desired angular velocity for the motor */
  void setVelocity(float vel);

  ~RJointDynamicModel();
};
#endif
#endif
