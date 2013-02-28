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

#include <lima/RJoint2DynamicModel.h>
#ifdef _USE_ODE_
#include <lima/visp/vpHomogeneousMatrix.h>
#include <lima/Joint.h>
#include <lima/RJoint2.h>

RJoint2DynamicModel::RJoint2DynamicModel(vpColVector axis1, float min_value1, float max_value1, float friction1, vpColVector axis2, float min_value2, float max_value2, float friction2)
{
  setAxis1(axis1);
  setRange1(min_value1,max_value1);
  setFriction1(friction1);
  setAxis2(axis2);
  setRange2(min_value2,max_value2);
  setFriction2(friction2);
  has_motor=false;
  joint_configured=false;
}

void RJoint2DynamicModel::setFriction1(float friction)
{
  this->friction1=friction;
}

void RJoint2DynamicModel::setFriction2(float friction)
{
  this->friction2=friction;
}

void RJoint2DynamicModel::configureJoint(Joint *j)
{
  //Attach to bodies
  dJointAttach(joint, j->previous->model->dynamics->body,j->next->model->dynamics->body);

  //set position. Go back in the hierarchy in order to compute wMj
  vpHomogeneousMatrix wMj;
  wMj=j->previous->wMo*j->pMj;
  dJointSetUniversalAnchor(joint, wMj[0][3],wMj[1][3],wMj[2][3]);

  //set axis1 and axis2 in ODE world frame
  vpRotationMatrix wRj;
  wMj.extract(wRj);
  vpTranslationVector waxis1, waxis2;
  vpTranslationVector taxis1(axis1[0],axis1[1],axis1[2]);
  vpTranslationVector taxis2(axis2[0],axis2[1],axis2[2]);
  waxis1=wRj*taxis1;
  waxis2=wRj*taxis2;
  dJointSetUniversalAxis1(joint, waxis1[0],waxis1[1],waxis1[2]);
  dJointSetUniversalAxis2(joint, waxis2[0],waxis2[1],waxis2[2]);

  //Joint limits. ODE has different convention for the rotation sense,
  //that's why we change max and min values.
  //Furthermore, ODE takes the initial joint position as zero, instead of
  //having an absolute zero. So, we have to add initial joint position
  //to the joint limits (hmm... difficult to explain)
  joint_offset1=((RJoint2*)j)->current_value1;
  joint_offset2=((RJoint2*)j)->current_value2;
  dJointSetUniversalParam (joint, dParamLoStop, -max_value1+joint_offset1);
  dJointSetUniversalParam (joint, dParamHiStop, -min_value1+joint_offset1);
  dJointSetUniversalParam (joint, dParamLoStop2, -max_value2+joint_offset2);
  dJointSetUniversalParam (joint, dParamHiStop2, -min_value2+joint_offset2);
  if (has_motor)
  {
    dJointSetUniversalParam(joint,dParamFudgeFactor,0.1);
    dJointSetUniversalParam(joint,dParamFMax,max_torque1);
    dJointSetUniversalParam(joint,dParamVel,velocity1);
    dJointSetUniversalParam(joint,dParamFudgeFactor2,0.1);
    dJointSetUniversalParam(joint,dParamFMax2,max_torque2);
    dJointSetUniversalParam(joint,dParamVel2,velocity2);
  }
  joint_configured=true;
}

void RJoint2DynamicModel::updateJointBySimulation(Joint *j)
{
  //ODE has different convention for the rotation sense (that's why the sign is changed)
  ((RJoint2*)j)->setValue1(-dJointGetUniversalAngle1(joint)+joint_offset1);
  ((RJoint2*)j)->setValue2(-dJointGetUniversalAngle2(joint)+joint_offset2);
}

void RJoint2DynamicModel::createJoint(dWorldID *world)
{
  joint=dJointCreateUniversal (*world, 0);
}

void RJoint2DynamicModel::hasMotor(float max_torque1, float max_torque2)
{
  has_motor=true;
  this->max_torque1=max_torque1;
  this->velocity1=0;
  this->max_torque2=max_torque2;
  this->velocity2=0;
}

void RJoint2DynamicModel::setVelocity1(float vel)
{
  //ODE has different convention for the rotation sense (that's why the sign is changed)
  this->velocity1=-vel;
  if (joint_configured)
  {
    dJointSetUniversalParam(joint,dParamVel,velocity1);
  }
}

void RJoint2DynamicModel::setVelocity2(float vel)
{
  //ODE has different convention for the rotation sense (that's why the sign is changed)
  this->velocity2=-vel;
  if (joint_configured)
  {
    dJointSetUniversalParam(joint,dParamVel2,velocity2);
  }
}

void RJoint2DynamicModel::setVelocity(float vel1, float vel2)
{
  //ODE has different convention for the rotation sense (that's why the sign is changed)
  this->velocity1=-vel1;
  this->velocity2=-vel2;
  if (joint_configured)
  {
    dJointSetUniversalParam(joint,dParamVel,velocity1);
    dJointSetUniversalParam(joint,dParamVel2,velocity2);
  }
}

RJoint2DynamicModel::~RJoint2DynamicModel()
{
}

#endif

