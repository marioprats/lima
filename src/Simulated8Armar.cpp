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

#include <lima/Simulated8Armar.h>

#include <visp/vpRGBa.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpVelocityTwistMatrix.h>

#include <lima/BBoxModel.h>
#include <lima/CylinderModel.h>
#include <lima/ObjectModel.h>
#include <lima/RJoint.h>
#include <lima/RJoint2.h>
#include <lima/FJoint.h>
#include <lima/Object.h>
#ifdef _USE_ODE_
#include <lima/BBoxDynamicModel.h>
#include <lima/RJointDynamicModel.h>
#include <lima/FJointDynamicModel.h>
#include <lima/RJoint2DynamicModel.h>
#endif

Simulated8Armar::Simulated8Armar(arm_t arm) :
    SimulatedArm()
{
  this->arm = arm;
  setObject(createObject());

  qmax.resize(8);
  qmin.resize(8);
  qrange.resize(8);
  qamax.resize(8);
  qamin.resize(8);

  qmax[0] = M_PI;
  qmax[1] = 0.58;
  qmax[2] = 1.51;
  qmax[3] = 1.29;
  qmax[4] = 0.87;
  qmax[5] = M_PI;
  qmax[6] = 0.23;
  qmax[7] = 0.91;

  qmin[0] = -M_PI;
  qmin[1] = -1.52;
  qmin[2] = -0.1;
  qmin[3] = -1.86;
  qmin[4] = -1.48;
  qmin[5] = -0.25;
  qmin[6] = -0.46;
  qmin[7] = -0.41;

  qrange = qmax - qmin;
  qamax = qmax - 0.3 * qrange;
  qamin = qmin + 0.3 * qrange;
// 	qamax[1]=qmax[1]-0.1*qrange[1];
// 	qamin[1]=qmin[1]+0.1*qrange[1];
// 	qamax[6]=qmax[6]-0.5*qrange[6];
// 	qamin[6]=qmin[6]+0.5*qrange[6];
// 	qamax[7]=qmax[7]-0.5*qrange[7];
// 	qamin[7]=qmin[7]+0.5*qrange[7];
}

Object *createBoxLink(string name, float x, float y, float z, float weight, char r, char g, char b)
{
  vpRGBa color(r, g, b, 0);
  BBoxModel *o_gmodel = new BBoxModel(x, y, z, color);
#ifdef _USE_ODE_
  BBoxDynamicModel *o_dmodel=new BBoxDynamicModel(x,y,z/2,weight);
  o_dmodel->affectedByGravity(false);
  ObjectModel *o_model=new ObjectModel(o_gmodel,o_dmodel);
#else
  ObjectModel *o_model = new ObjectModel(o_gmodel);
#endif
  Object *o = new Object(name, o_model);
  return o;
}

Object *createCylinderLink(string name, float x, float y, float z, float weight, char r, char g, char b)
{
  vpRGBa color(r, g, b, 0);
  CylinderModel *o_gmodel = new CylinderModel(x / 2, y / 2, z, color);
#ifdef _USE_ODE_
  BBoxDynamicModel *o_dmodel=new BBoxDynamicModel(x,y,z/2,weight);
  o_dmodel->affectedByGravity(false);
  ObjectModel *o_model=new ObjectModel(o_gmodel,o_dmodel);
#else
  ObjectModel *o_model = new ObjectModel(o_gmodel);
#endif
  Object *o = new Object(name, o_model);
  return o;
}

Object *Simulated8Armar::createObject()
{
  //we create the links
  Object *link_base = createBoxLink("link_base", 0.1, 0.02, 0.1, 1, 50, 50, 50);
  Object *link_q0 = createBoxLink("link_q0", 0.1, 0.5, 0.294, 1, 150, 150, 200);
  Object *link_q1 = createBoxLink("link_q1", 0.08, 0.08, 0.090, 1, 100, 100, 250);
  Object *link_q2 = createCylinderLink("link_q2", 0.05, 0.05, 0.100, 1, 200, 200, 200);
  Object *link_q3 = createCylinderLink("link_q3", 0.07, 0.07, 0.210, 1, 100, 100, 250);
  Object *link_q4 = createCylinderLink("link_q4", 0.05, 0.05, 0.040, 1, 200, 200, 200);
  Object *link_q5 = createCylinderLink("link_q5", 0.07, 0.07, 0.200, 1, 100, 100, 250);
  Object *link_q6 = createCylinderLink("link_q6", 0.01, 0.01, 0.001, 1, 200, 200, 200);
  if (arm == Simulated8Armar::LEFT)
  {
    vpHomogeneousMatrix rotx(0, 0, 0, -M_PI_2, 0, 0);
    vpHomogeneousMatrix rotz(0, 0, 0, 0, 0, M_PI_2);
    vpHomogeneousMatrix trans(0, 0, 0.470, 0, 0, 0);
    setKinematicBase(rotx * rotz * trans);
  }
  else
  {
    vpHomogeneousMatrix rotx(0, 0, 0, -M_PI_2, 0, 0);
    vpHomogeneousMatrix rotz(0, 0, 0, 0, 0, M_PI_2);
    vpHomogeneousMatrix trans(0, 0, 0.470, 0, 0, 0);
    setKinematicBase(rotx * rotz * trans);
  }

  //we create the joints
  vpColVector rzaxis(3), ryaxis(3), rxaxis(3);
  rxaxis = 0;
  rxaxis[0] = 1;
  ryaxis = 0;
  ryaxis[1] = 1;
  rzaxis = 0;
  rzaxis[2] = 1;
  RJoint *q0, *q1, *q2, *q3, *q4, *q5;
  RJoint2 *q67;
  if (arm == Simulated8Armar::LEFT)
  {
    q0 = new RJoint(ryaxis, -M_PI, M_PI);
    q1 = new RJoint(rzaxis, -1.52, 0.58);
    q2 = new RJoint(rxaxis, -1.51, 0.1);
    q3 = new RJoint(rxaxis, -1.86, 1.29);
    q4 = new RJoint(ryaxis, -0.87, 1.48);
    q5 = new RJoint(rzaxis, -3.14, 0.25);
    q67 = new RJoint2(rxaxis, -0.23, 0.46, ryaxis, -0.91, 0.41);
  }
  else
  {
    q0 = new RJoint(ryaxis, -M_PI, M_PI);
    q1 = new RJoint(rzaxis, -0.58, 1.52);
    q2 = new RJoint(rxaxis, -1.51, 0.1);
    q3 = new RJoint(rxaxis, -1.29, 1.86);
    q4 = new RJoint(ryaxis, -1.48, 0.87);
    q5 = new RJoint(rzaxis, -0.25, 3.14);
    q67 = new RJoint2(rxaxis, -0.23, 0.46, ryaxis, -0.41, 0.91);
  }

//  	q2->setValue(-15*M_PI/180);
// 	q4->setValue(M_PI_2);
#ifdef _USE_ODE_
  RJointDynamicModel *q0_dmodel, *q1_dmodel, *q2_dmodel, *q3_dmodel, *q4_dmodel, *q5_dmodel;
  RJoint2DynamicModel *q67_dmodel;
  if (arm==Simulated8Armar::LEFT)
  {
    q0_dmodel=new RJointDynamicModel(ryaxis,-M_PI,M_PI,0);
    q1_dmodel=new RJointDynamicModel(rzaxis,-1.52,0.58,0);
    q2_dmodel=new RJointDynamicModel(rxaxis,-1.51,0.1,0);
    q3_dmodel=new RJointDynamicModel(rzaxis,-1.86,1.29,0);
    q4_dmodel=new RJointDynamicModel(ryaxis,-0.87,1.48,0);
    q5_dmodel=new RJointDynamicModel(rzaxis,-0.25,3.14,0);
    q67_dmodel=new RJoint2DynamicModel(rxaxis,-0.23,0.46,0,
        ryaxis,-0.41,0.91,0);
  }
  else
  {
    q0_dmodel=new RJointDynamicModel(ryaxis,-M_PI,M_PI,0);
    q1_dmodel=new RJointDynamicModel(rzaxis,-0.58,1.52,0);
    q2_dmodel=new RJointDynamicModel(rxaxis,-1.51,0.1,0);
    q3_dmodel=new RJointDynamicModel(rzaxis,-1.29,1.86,0);
    q4_dmodel=new RJointDynamicModel(ryaxis,-1.48,0.87,0);
    q5_dmodel=new RJointDynamicModel(rzaxis,-3.14,0.25,0);
    q67_dmodel=new RJoint2DynamicModel(rxaxis,-0.23,0.46,0,
        ryaxis,-0.91,0.41,0);
  }
  q0_dmodel->hasMotor(20);
  q1_dmodel->hasMotor(20);
  q2_dmodel->hasMotor(20);
  q3_dmodel->hasMotor(20);
  q4_dmodel->hasMotor(20);
  q5_dmodel->hasMotor(20);
  q67_dmodel->hasMotor(20,20);
  q0->setDynamicModel(q0_dmodel);
  q1->setDynamicModel(q1_dmodel);
  q2->setDynamicModel(q2_dmodel);
  q3->setDynamicModel(q3_dmodel);
  q4->setDynamicModel(q4_dmodel);
  q5->setDynamicModel(q5_dmodel);
  q67->setDynamicModel(q67_dmodel);
#endif

  //we attach links and joints
  if (arm == Simulated8Armar::LEFT)
  {
    vpHomogeneousMatrix lbMq0(0, 0.26, 0, 0, 0, 0);
    vpHomogeneousMatrix l0Mq0(0, 0, 0, 0, 0, 0);
    vpHomogeneousMatrix l0Mq1(0, 0.210, 0.147, 0, 0, 0);
    vpHomogeneousMatrix l1Mq1(0, 0, -0.040, 0, 0, 0);
    vpHomogeneousMatrix l1Mq2(0, 0, 0.040, M_PI_2 - 15 * M_PI / 180, 0, 0);
    vpHomogeneousMatrix l2Mq2(0, 0, 0, 0, 0, 0);
    vpHomogeneousMatrix l2Mq3(0, 0, 0.1, 0, 0, 0);
    vpHomogeneousMatrix l3Mq3(0, 0, 0, 0, 0, 0);
    vpHomogeneousMatrix l3Mq4(-0.02, 0, 0.210, 0, -M_PI_2, 0);
    vpHomogeneousMatrix l4Mq4(0, -0.0075, 0, 0, 0, 0);
    vpHomogeneousMatrix l4Mq5(0, 0, 0.04, 0, 0, 0);
    vpHomogeneousMatrix l5Mq5(0, 0, 0, 0, 0, 0);
    vpHomogeneousMatrix l5Mq67(0, 0, 0.2, 0, 0, 0);
    vpHomogeneousMatrix l6Mq67(0, 0, -0.08 + 0.0005, 0, 0, 0);
    link_base->linkTo(lbMq0, q0, l0Mq0, link_q0);
    link_q0->linkTo(l0Mq1, q1, l1Mq1, link_q1);
    link_q1->linkTo(l1Mq2, q2, l2Mq2, link_q2);
    link_q2->linkTo(l2Mq3, q3, l3Mq3, link_q3);
    link_q3->linkTo(l3Mq4, q4, l4Mq4, link_q4);
    link_q4->linkTo(l4Mq5, q5, l5Mq5, link_q5);
    link_q5->linkTo(l5Mq67, q67, l6Mq67, link_q6);
  }
  else
  {
    vpHomogeneousMatrix lbMq0(0, 0.26, 0, 0, 0, 0);
    vpHomogeneousMatrix l0Mq0(0, 0, 0, 0, M_PI, 0);
    vpHomogeneousMatrix l0Mq1(0, 0.210, 0.147, 0, 0, 0);
    vpHomogeneousMatrix l1Mq1(0, 0, -0.040, 0, 0, 0);
    vpHomogeneousMatrix l1Mq2(0, 0, 0.040, M_PI_2 - 15 * M_PI / 180, 0, 0);
    vpHomogeneousMatrix l2Mq2(0, 0, 0, 0, 0, 0);
    vpHomogeneousMatrix l2Mq3(0, 0, 0.1, 0, 0, 0);
    vpHomogeneousMatrix l3Mq3(0, 0, 0, 0, 0, 0);
    vpHomogeneousMatrix l3Mq4(0.02, 0, 0.215, 0, M_PI_2, 0);
    vpHomogeneousMatrix l4Mq4(0, -0.0075, 0, 0, 0, 0);
    vpHomogeneousMatrix l4Mq5(0, 0, 0.04, 0, 0, 0);
    vpHomogeneousMatrix l5Mq5(0, 0, 0, 0, 0, 0);
    vpHomogeneousMatrix l5Mq67(0, 0, 0.2, 0, 0, 0);
    vpHomogeneousMatrix l6Mq67(0, 0, -0.08 + 0.0005, 0, 0, M_PI);
    link_base->linkTo(lbMq0, q0, l0Mq0, link_q0);
    link_q0->linkTo(l0Mq1, q1, l1Mq1, link_q1);
    link_q1->linkTo(l1Mq2, q2, l2Mq2, link_q2);
    link_q2->linkTo(l2Mq3, q3, l3Mq3, link_q3);
    link_q3->linkTo(l3Mq4, q4, l4Mq4, link_q4);
    link_q4->linkTo(l4Mq5, q5, l5Mq5, link_q5);
    link_q5->linkTo(l5Mq67, q67, l6Mq67, link_q6);
  }
  return link_base;
}

vpHomogeneousMatrix Simulated8Armar::getPosition()
{
  vpHomogeneousMatrix rMe = directKinematics(getJointValues());
  rMe[0][3] /= 1000;
  rMe[1][3] /= 1000;
  rMe[2][3] /= 1000;
  return rMe;
}

vpColVector Simulated8Armar::getJointValues()
{
  //get current joint values
  vpColVector q(8);
  if (arm == Simulated8Armar::LEFT)
  {
    q[0] = ((RJoint*)object->linkedTo[0])->current_value;
    q[1] = ((RJoint*)object->linkedTo[0]->next->linkedTo[0])->current_value;
    q[2] = -((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;
    q[3] = ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;
    q[4] =
        -((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;
    q[5] =
        ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;
    q[6] =
        -((RJoint2*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value1;
    q[7] =
        ((RJoint2*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value2;
  }
  else
  {
    q[0] = ((RJoint*)object->linkedTo[0])->current_value;
    q[1] = -((RJoint*)object->linkedTo[0]->next->linkedTo[0])->current_value;
    q[2] = -((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;
    q[3] = -((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;
    q[4] =
        ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;
    q[5] =
        -((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;
    q[6] =
        -((RJoint2*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value1;
    q[7] =
        -((RJoint2*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value2;
  }
  return q;
}

void Simulated8Armar::setJointValues(vpColVector dq)
{
  cerr << "Simulated8Armar::setJointValues ERROR: Not yet implemented" << endl;
}

void Simulated8Armar::setJointVelocity(vpColVector qdot)
{
#ifdef _USE_ODE_
  if (arm==Simulated8Armar::LEFT)
  {
    ((RJointDynamicModel*)object->linkedTo[0]->dynamic_model)->setVelocity(qdot[0]);
    ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qdot[1]);
    ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(-qdot[2]);
    ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qdot[3]);
    ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(-qdot[4]);
    ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qdot[5]);
    ((RJoint2DynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(-qdot[6],qdot[7]);
  }
  else
  {
    ((RJointDynamicModel*)object->linkedTo[0]->dynamic_model)->setVelocity(qdot[0]);
    ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(-qdot[1]);
    ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(-qdot[2]);
    ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(-qdot[3]);
    ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qdot[4]);
    ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(-qdot[5]);
    ((RJoint2DynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(-qdot[6],-qdot[7]);
  }
#endif
}

bool Simulated8Armar::checkJointLimits(vector<int> &inlimit)
{
  //Check joint limits:
  vpColVector q(8);
  q = getJointValues();
  inlimit.clear();
  if (q[0] <= -M_PI || q[0] >= M_PI)
  {
    inlimit.push_back(0);
    cerr << "Simulated8Armar::setJointVelocity ERROR: Joint 0 is in joint limit" << endl;
  }
  else if (q[1] <= -1.52 || q[1] >= 0.58)
  {
    inlimit.push_back(1);
    cerr << "Simulated8Armar::setJointVelocity ERROR: Joint 1 is in joint limit" << endl;
  }
  else if (q[2] <= -0.1 || q[2] >= 1.51)
  {
    inlimit.push_back(2);
    cerr << "Simulated8Armar::setJointVelocity ERROR: Joint 2 is in joint limit" << endl;
  }
  else if (q[3] <= -1.86 || q[3] >= 1.29)
  {
    inlimit.push_back(3);
    cerr << "Simulated8Armar::setJointVelocity ERROR: Joint 3 is in joint limit" << endl;
  }
  else if (q[4] <= -1.48 || q[4] >= 0.87)
  {
    inlimit.push_back(4);
    cerr << "Simulated8Armar::setJointVelocity ERROR: Joint 4 is in joint limit" << endl;
  }
  else if (q[5] <= -0.25 || q[5] >= 3.14)
  {
    inlimit.push_back(5);
    cerr << "Simulated8Armar::setJointVelocity ERROR: Joint 5 is in joint limit" << endl;
  }
  else if (q[6] <= -0.46 || q[6] >= 0.23)
  {
    inlimit.push_back(6);
    cerr << "Simulated8Armar::setJointVelocity ERROR: Joint 6 is in joint limit" << endl;
  }
  else if (q[7] <= -0.41 || q[7] >= 0.91)
  {
    inlimit.push_back(7);
    cerr << "Simulated8Armar::setJointVelocity ERROR: Joint 7 is in joint limit" << endl;
  }
  return (inlimit.size() > 0);
}

void Simulated8Armar::setCartesianVelocity(vpColVector xdot)
{
  vpColVector qv(8), q(8);
  q = getJointValues();
  vpMatrix J = jacobian77(q);
  vpHomogeneousMatrix pMe;
  pMe[2][3] = -100;
  vpVelocityTwistMatrix pMe6(pMe);
  vpMatrix pMe6m = pMe6;
  J = pMe6m * J;
// 	for (int i=4; i<5; i++)
// 		for (int j=0;j<8;j++) {
// 			J[i][j]=0;
// 		}
  vpMatrix Jiv = J.pseudoInverse();

  cerr << "Jiv: " << endl << Jiv << endl;
  cerr << "J: " << endl << J << endl;

  //The jacobian is computed for units in millimeters
  xdot[0] *= 1000;
  xdot[1] *= 1000;
  xdot[2] *= 1000;

  vpColVector costvector(8);
  float hs = 0; //cost function value
  for (int i = 0; i < 8; i++)
  {
    if (q[i] > qamax[i])
    {
      costvector[i] = (q[i] - qamax[i]) / qrange[i];
      hs = hs + pow((q[i] - qamax[i]), 2) / qrange[i];
    }
    else if (q[i] < qamin[i])
    {
      costvector[i] = (q[i] - qamin[i]) / qrange[i];
      hs = hs + pow((q[i] - qamin[i]), 2) / qrange[i];
    }
    else
    {
      costvector[i] = 0;
    }
  }
  hs = hs / 2.0;
  cerr << "cost_function: " << hs << endl;
  cerr << "costvector: " << costvector.t();

  vpMatrix I(8, 8);
  I.setIdentity();
  vpColVector e2(8);
  e2 = -10.0 * (I - Jiv * J) * costvector;
  cerr << "secondary_task: " << e2.t();

  vpColVector e1(8);
  e1 = Jiv * xdot;
  cerr << "primary_task: " << e1.t();

  qv = e1 + e2;

  setJointVelocity(qv);
}

void Simulated8Armar::DHTransformationMatrix(double theta, double alpha, double a, double d,
                                             vpHomogeneousMatrix &result)
{
  double ct, st, ca, sa;
  ct = cos(theta);
  st = sin(theta);
  ca = cos(alpha);
  sa = sin(alpha);

  result[0][0] = ct;
  result[0][1] = -ca * st;
  result[0][2] = sa * st;
  result[0][3] = a * ct;

  result[1][0] = st;
  result[1][1] = ca * ct;
  result[1][2] = -sa * ct;
  result[1][3] = a * st;

  result[2][0] = 0.0;
  result[2][1] = sa;
  result[2][2] = ca;
  result[2][3] = d;

  result[3][0] = 0.0;
  result[3][1] = 0.0;
  result[3][2] = 0.0;
  result[3][3] = 1.0;
}

vpHomogeneousMatrix Simulated8Armar::directKinematics(vpColVector q)
{
  static vpColVector joint_offset(7);
  joint_offset[0] = 0, joint_offset[1] = 15.0 * M_PI / 180.0;
  joint_offset[2] = 0;
  joint_offset[3] = 90.0 * M_PI / 180.0;
  joint_offset[4] = 0;
  joint_offset[5] = 0;
  joint_offset[6] = 0;

  vpHomogeneousMatrix wMe;

  vpHomogeneousMatrix tmpMatrix;

  if (arm == Simulated8Armar::LEFT)
  {
    DHTransformationMatrix(q[0] - M_PI_2, M_PI_2, 0.0, 0.0, tmpMatrix);
    wMe = wMe * tmpMatrix;
// 	DHTransformationMatrix(q[0],  0,  0.0,  232.0,tmpMatrix);
// 	wMe = wMe*tmpMatrix;
    DHTransformationMatrix(q[1] + M_PI_2 + joint_offset[0], M_PI_2, 0.0, 232.0, tmpMatrix);
    wMe = wMe * tmpMatrix;

    DHTransformationMatrix(-q[2] + M_PI_2 - joint_offset[1], M_PI_2, 0.0, 0.0, tmpMatrix);
    wMe = wMe * tmpMatrix;

    DHTransformationMatrix(-q[3] - M_PI_2 - joint_offset[2], M_PI_2, 20, -310, tmpMatrix);
    wMe = wMe * tmpMatrix;

    DHTransformationMatrix(q[4] + joint_offset[3], -M_PI_2, 0.0, -7.5, tmpMatrix);
    wMe = wMe * tmpMatrix;

    DHTransformationMatrix(-q[5] + M_PI_2 - joint_offset[4], -M_PI_2, 0.0, -240.0, tmpMatrix);
    wMe = wMe * tmpMatrix;

    DHTransformationMatrix(-q[6] - M_PI_2 - joint_offset[5], -M_PI_2, 0.0, 0.0, tmpMatrix);
    wMe = wMe * tmpMatrix;

    DHTransformationMatrix(q[7] - M_PI_2 + joint_offset[6], M_PI_2, 0.0, 0.0, tmpMatrix);
    wMe = wMe * tmpMatrix;
    DHTransformationMatrix(0.0, 0.0, 0.0, 80, tmpMatrix);
    wMe = wMe * tmpMatrix;
  }
  else
  {
    DHTransformationMatrix(q[0] - M_PI_2, M_PI_2, 0.0, 0.0, tmpMatrix);
    wMe = wMe * tmpMatrix;
    DHTransformationMatrix(q[1] + M_PI_2 + joint_offset[0], M_PI_2, 0.0, -232.0, tmpMatrix);
    wMe = wMe * tmpMatrix;

    DHTransformationMatrix(q[2] + M_PI_2 + joint_offset[1], M_PI_2, 0.0, 0.0, tmpMatrix);
    wMe = wMe * tmpMatrix;

    DHTransformationMatrix(q[3] - M_PI_2 + joint_offset[2], M_PI_2, 20, -310, tmpMatrix);
    wMe = wMe * tmpMatrix;

    DHTransformationMatrix(q[4] + joint_offset[3], -M_PI_2, 0.0, 7.5, tmpMatrix);
    wMe = wMe * tmpMatrix;

    DHTransformationMatrix(q[5] + M_PI_2 + joint_offset[4], -M_PI_2, 0.0, -240.0, tmpMatrix);
    wMe = wMe * tmpMatrix;

    DHTransformationMatrix(q[6] - M_PI_2 + joint_offset[5], -M_PI_2, 0.0, 0.0, tmpMatrix);
    wMe = wMe * tmpMatrix;

    DHTransformationMatrix(q[7] - M_PI_2 + joint_offset[6], M_PI_2, 0.0, 0.0, tmpMatrix);
    wMe = wMe * tmpMatrix;
    DHTransformationMatrix(0.0, 0.0, 0.0, 80, tmpMatrix);
    wMe = wMe * tmpMatrix;
  }

  return wMe;
}

vpMatrix Simulated8Armar::jacobian07(vpColVector q)
{
  static vpColVector joint_offset(7);
  joint_offset[0] = 0, joint_offset[1] = 15.0 * M_PI / 180.0;
  joint_offset[2] = 0;
  joint_offset[3] = 90.0 * M_PI / 180.0;
  joint_offset[4] = 0;
  joint_offset[5] = 0;
  joint_offset[6] = 0;

  vpMatrix JacResult(6, 7);

  vpHomogeneousMatrix tmpMatrix;

  //array of matrices for use to define Jacobian
  vpMatrix Tij[9][9]; //array of transf. matrices between joints

  //define the size of transf. matrices
  for (int i = 0; i < 9; i++)
  {
    for (int j = 0; j < 9; j++)
    {
      Tij[i][j].resize(4, 4);
    }
    //make all diagonal matrices unit Tii=eye(4)
    Tij[i][i].setIdentity();
  }

  if (arm == Simulated8Armar::LEFT)
  {
    //Hip
    DHTransformationMatrix(q[0] - M_PI_2, M_PI_2, 0.0, 0.0, tmpMatrix);
    Tij[0][1] = Tij[0][0] * tmpMatrix;
    //shoulder1
    DHTransformationMatrix(q[1] + M_PI_2 + joint_offset[0], M_PI_2, 0.0, 232.0, tmpMatrix);
    Tij[1][2] = tmpMatrix;
    //shoulder2
    DHTransformationMatrix(-q[2] + M_PI_2 - joint_offset[1], M_PI_2, 0.0, 0.0, tmpMatrix);
    Tij[2][3] = tmpMatrix;
    //upperarm
    DHTransformationMatrix(-q[3] - M_PI_2 - joint_offset[2], M_PI_2, 20, -310, tmpMatrix);
    Tij[3][4] = tmpMatrix;
    //elbow
    DHTransformationMatrix(q[4] + joint_offset[3], -M_PI_2, 0.0, -7.5, tmpMatrix);
    Tij[4][5] = tmpMatrix;
    //forarm
    DHTransformationMatrix(-q[5] + M_PI_2 - joint_offset[4], -M_PI_2, 0.0, -240, tmpMatrix);
    Tij[5][6] = tmpMatrix;
    //hand1
    DHTransformationMatrix(-q[6] - M_PI_2 - joint_offset[5], -M_PI_2, 0.0, 0.0, tmpMatrix);
    Tij[6][7] = tmpMatrix;
    //hand2
    DHTransformationMatrix(q[7] - M_PI_2 + joint_offset[6], M_PI_2, 0.0, 0.0, tmpMatrix);
    Tij[7][8] = tmpMatrix;
    DHTransformationMatrix(0.0, 0.0, 0.0, 80, tmpMatrix);
    Tij[7][8] = Tij[7][8] * tmpMatrix;
  }
  else
  {
    //Hip
    DHTransformationMatrix(q[0] - M_PI_2, M_PI_2, 0.0, 0.0, tmpMatrix);
    Tij[0][1] = tmpMatrix;
    //shoulder1
    DHTransformationMatrix(q[1] + M_PI_2 + joint_offset[0], M_PI_2, 0.0, -232.0, tmpMatrix);
    Tij[1][2] = tmpMatrix;
    //shoulder2
    DHTransformationMatrix(q[2] + M_PI_2 + joint_offset[1], M_PI_2, 0.0, 0.0, tmpMatrix);
    Tij[2][3] = tmpMatrix;
    //upperarm
    DHTransformationMatrix(q[3] - M_PI_2 + joint_offset[2], M_PI_2, 20, -310, tmpMatrix);
    Tij[3][4] = tmpMatrix;
    //elbow
    DHTransformationMatrix(q[4] + joint_offset[3], -M_PI_2, 0.0, 7.5, tmpMatrix);
    Tij[4][5] = tmpMatrix;
    //forarm
    DHTransformationMatrix(q[5] + M_PI_2 + joint_offset[4], -M_PI_2, 0.0, -240, tmpMatrix);
    Tij[5][6] = tmpMatrix;
    //hand1
    DHTransformationMatrix(q[6] - M_PI_2 + joint_offset[5], -M_PI_2, 0.0, 0.0, tmpMatrix);
    Tij[6][7] = tmpMatrix;
    //hand2
    DHTransformationMatrix(q[7] - M_PI_2 + joint_offset[6], M_PI_2, 0.0, 0.0, tmpMatrix);
    Tij[7][8] = tmpMatrix;
    DHTransformationMatrix(0.0, 0.0, 0.0, 80, tmpMatrix);
    Tij[7][8] = Tij[7][8] * tmpMatrix;
  }

  //define also the elements above diagonal
  for (int i = 0; i < 9; i++)
    for (int j = i + 2; j < 9; j++)
      Tij[i][j] = Tij[i][j - 1] * Tij[j - 1][j];

  vpMatrix JacL(3, 8), JacA(3, 8);

  for (int i = 1; i < 9; i++)
  {
    //see Mathematica code (Robotica for Mathematica function FormTheJacobianJ[])
    vpColVector a(3), b(3), c(3);
    a[0] = Tij[0][i - 1][0][2];
    a[1] = Tij[0][i - 1][1][2];
    a[2] = Tij[0][i - 1][2][2];
    b[0] = Tij[0][8][0][3] - Tij[0][i - 1][0][3];
    b[1] = Tij[0][8][1][3] - Tij[0][i - 1][1][3];
    b[2] = Tij[0][8][2][3] - Tij[0][i - 1][2][3];

    //linear part of the Jacobian	
    c = vpColVector::cross(a, b);
    JacL[0][i - 1] = c[0];
    JacL[1][i - 1] = c[1];
    JacL[2][i - 1] = c[2];

    //angular part of the Jacobian	
    JacA[0][i - 1] = a[0];
    JacA[1][i - 1] = a[1];
    JacA[2][i - 1] = a[2];
  }
  //concatenate matrices
  JacResult = vpMatrix::stackMatrices(JacL, JacA);

  if (arm == Simulated8Armar::LEFT)
  {
    //Joints q2,q3,q5 and q6 have opposite sense
    for (int i = 0; i < 6; i++)
    {
      JacResult[i][2] *= -1;
      JacResult[i][3] *= -1;
      JacResult[i][5] *= -1;
      JacResult[i][6] *= -1;
    }
  }

  return JacResult;
}

vpMatrix Simulated8Armar::jacobian77(vpColVector q)
{
  //Robot jacobian in base frame
  vpMatrix J07(6, 8);
  J07 = jacobian07(q);
  //Direct geometric model
  vpHomogeneousMatrix bMe;
  bMe = directKinematics(q);

  //Robot jacobian J(q) in effector frame: 7J7
  vpMatrix J77(6, 8);
  vpRotationMatrix R07;
  bMe.extract(R07);
  R07 = R07.t();
  vpMatrix W07(6, 6);
  W07 = 0;
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      W07[i][j] = R07[i][j];
      W07[i + 3][j + 3] = R07[i][j];
    }
  }
  J77 = W07 * J07;
  return J77;
}

vpMatrix Simulated8Armar::jacobian77Inverse(vpColVector q)
{
  //Robot jacobian J(q) in EEF frame: 7J7
  vpMatrix J77(6, 8);
  J77 = jacobian77(q);

  return J77.pseudoInverse();
}

Simulated8Armar::~Simulated8Armar()
{
}

