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

#include <lima/RJointDynamicModel.h>
#ifdef _USE_ODE_
#include <lima/visp/vpHomogeneousMatrix.h>
#include <lima/Joint.h>
#include <lima/RJoint.h>

RJointDynamicModel::RJointDynamicModel()
{
  has_motor=false;
  joint_configured=false;
}

RJointDynamicModel::RJointDynamicModel(vpColVector axis, float min_value, float max_value, float friction)
{
  setAxis(axis);
  setRange(min_value,max_value);
  setFriction(friction);
  has_motor=false;
  joint_configured=false;
}

void RJointDynamicModel::setFriction(float friction)
{
  this->friction=friction;
  //TODO: Implement joint friction
  //It can be easily implementing by setting a motor with desired velocity 0,
  //and non-zero maximum force. The only thing is to relate the friction coefficient
  //with the motor force.
}

void RJointDynamicModel::configureJoint(Joint *j)
{
  //Attach to bodies
  dJointAttach(joint, j->previous->model->dynamics->body,j->next->model->dynamics->body);

  //set position. Go back in the hierarchy in order to compute wMj
  vpHomogeneousMatrix wMj;
  wMj=j->previous->wMo*j->pMj;
  dJointSetHingeAnchor(joint, wMj[0][3],wMj[1][3],wMj[2][3]);

  //set axis in ODE world frame
  vpRotationMatrix wRj;
  wMj.extract(wRj);
  vpTranslationVector taxis(axis[0],axis[1],axis[2]);
  vpTranslationVector waxis;
  waxis=wRj*taxis;
  dJointSetHingeAxis(joint, waxis[0],waxis[1],waxis[2]);

  //Joint limits. ODE has different convention for the rotation sense,
  //that's why we change max and min values.
  //Furthermore, ODE takes the initial joint position as zero, instead of
  //having an absolute zero. So, we have to add initial joint position
  //to the joint limits (hmm... difficult to explain)
  joint_offset=((RJoint*)j)->current_value;
  dJointSetHingeParam (joint, dParamLoStop, -max_value+joint_offset);
  dJointSetHingeParam (joint, dParamHiStop, -min_value+joint_offset);

  if (has_motor)
  {
    dJointSetHingeParam(joint,dParamFudgeFactor,0.1);
    dJointSetHingeParam(joint,dParamFMax,max_torque);
    dJointSetHingeParam(joint,dParamVel,velocity);
  }
  joint_configured=true;
}

void RJointDynamicModel::updateJointBySimulation(Joint *j)
{
  //ODE has different convention for the rotation sense (that's why the sign is changed)
  ((RJoint*)j)->setValue(-dJointGetHingeAngle(joint)+joint_offset);
}

void RJointDynamicModel::createJoint(dWorldID *world)
{
  joint=dJointCreateHinge (*world, 0);
}

void RJointDynamicModel::hasMotor(float max_torque)
{
  has_motor=true;
  this->max_torque=max_torque;
  this->velocity=0;
}

void RJointDynamicModel::setVelocity(float vel)
{
  //ODE has different convention for the rotation sense (that's why the sign is changed)
  this->velocity=-vel;
  if (joint_configured)
  {
    dJointSetHingeParam(joint,dParamVel,velocity);
  }
}

RJointDynamicModel::~RJointDynamicModel()
{
}

#endif

