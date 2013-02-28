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

#ifndef RJOINT2DYNAMICMODEL_H
#define RJOINT2DYNAMICMODEL_H

#include <lima/limaconfig.h>
#ifdef _USE_ODE_
#include <lima/JointDynamicModel.h>
#include <visp/vpColVector.h>
#include <ode/ode.h>

/**
 The dynamic model for a mechanism composed of two rotational joints with perpendicular rotational axis (universal joint in ODE)

 @author Mario Prats <mprats@icc.uji.es>
 */
class RJoint2DynamicModel : public JointDynamicModel
{
public:
  vpColVector axis1; ///< The first rotational axis
  float min_value1;///< The minimum value for the joint 1
  float max_value1;///< The maximum value for the joint 1
  float joint_offset1;///< The initial value of the joint 1
  float friction1;///< The friction for the joint 1
  vpColVector axis2;///< The first rotational axis
  float min_value2;///< The minimum value for the joint 1
  float max_value2;///< The maximum value for the joint 1
  float joint_offset2;///< The initial value of the joint 1
  float friction2;///< The friction for the joint 1
  bool joint_configured;///< Whether the joint has been already configured or not
  bool has_motor;///< Whether the joint has a motor attached or not
  float velocity1;///< If the joint has a motor, this is the desired joint 1 velocity
  float velocity2;///< If the joint has a motor, this is the desired joint 2 velocity
  float max_torque1;///< The maximum torque that the joint 1 motor can apply
  float max_torque2;///< The maximum torque that the joint 1 motor can apply

  /** Constructor from the rotational axis, range and friction
   axis must be a 3-component unitary vector, min_value and max_value
   are the minimum and maximum joint values (in radians). All values must
   be specified, first for joint1 and after for joint2
   */
  RJoint2DynamicModel(vpColVector axis1, float min_value1, float max_value1, float friction1,
      vpColVector axis2, float min_value2, float max_value2, float friction2);

  /** Sets the first joint rotational axis (axis must be a 3-component unitary vector) */
  void setAxis1(vpColVector &axis)
  { this->axis1=axis;}
  /** Sets the second joint rotational axis (axis must be a 3-component unitary vector) */
  void setAxis2(vpColVector &axis)
  { this->axis2=axis;}
  /** Sets the range for the first joint, i.e, minimum and maximum joint values (in radians) */
  void setRange1(float min_value, float max_value)
  { this->min_value1=min_value; this->max_value1=max_value;}
  /** Sets the range for the second joint, i.e, minimum and maximum joint values (in radians) */
  void setRange2(float min_value, float max_value)
  { this->min_value2=min_value; this->max_value2=max_value;}
  /** Sets the friction for the first joint  */
  void setFriction1(float friction);
  /** Sets the friction for the second joint  */
  void setFriction2(float friction);

  /** Configures the ODE parameters of the joint */
  void configureJoint(Joint *j);

  /** Updates the joint object according to the current state of the ODE simulation */
  void updateJointBySimulation(Joint *j);

  /** Creates the ODE joint from the parameters */
  void createJoint(dWorldID *world);

  /** Sets a motor in the joint which can introduce a maximum torque of 'max_torque1' 
   in joint 1, and 'max_torque2' in joint 2*/
  void hasMotor(float max_torque1, float max_torque2);
  /** Sets the desired angular velocity for the first motor */
  void setVelocity1(float vel);
  /** Sets the desired angular velocity for the second motor */
  void setVelocity2(float vel);
  /** Sets the desired angular velocity for both motors */
  void setVelocity(float vel1, float vel2);

  ~RJoint2DynamicModel();
};
#endif
#endif
