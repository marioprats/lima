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

#include <lima/SimulatedPA10.h>

#include <visp/vpRGBa.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpVelocityTwistMatrix.h>

#include <lima/BBoxModel.h>
#include <lima/GenericFileModel.h>
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

SimulatedPA10::SimulatedPA10(vpColVector qini = 0) :
    SimulatedArm()
{
  qmax.resize(7);
  qmin.resize(7);
  qrange.resize(7);
  qamax.resize(7);
  qamin.resize(7);
  this->qini.resize(7);
  this->qini = qini;

  qmax[0] = 177 * M_PI / 180;
  qmax[1] = 91 * M_PI / 180;
  qmax[2] = 174 * M_PI / 180;
  qmax[3] = 137 * M_PI / 180;
  qmax[4] = 255 * M_PI / 180;
  qmax[5] = 165 * M_PI / 180;
  qmax[6] = 360 * M_PI / 180;

  qmin[0] = -177 * M_PI / 180;
  qmin[1] = -91 * M_PI / 180;
  qmin[2] = -174 * M_PI / 180;
  qmin[3] = -137 * M_PI / 180;
  qmin[4] = -255 * M_PI / 180;
  qmin[5] = -165 * M_PI / 180;
  qmin[6] = -360 * M_PI / 180;

  qrange = qmax - qmin;
  qamax = qmax - 0.1 * qrange;
  qamin = qmin + 0.1 * qrange;
  qamax[3] = qmax[3] - 0.2 * qrange[3];
  qamin[3] = qmin[3] + 0.7 * qrange[3];
  qamax[5] = qmax[5] - 0.45 * qrange[5];
  qamin[5] = qmin[5] + 0.45 * qrange[5];

  setObject(createObject());
  vpHomogeneousMatrix trans(0, 0, -0.202, 0, 0, 0);
  setKinematicBase(trans);
}

Object *SimulatedPA10::createObject()
{
  vpRGBa grey(200, 200, 200, 0);
  //we create the links
  /*	RawModel *link_base=new RawModel("/home/mprats/uji/PA10/3DModel/pa10link0.raw",grey,RawModel::INPUT_METERS);
   RawModel *link1=new RawModel("/home/mprats/uji/PA10/3DModel/pa10link1.raw",grey,RawModel::INPUT_METERS);
   RawModel *link2=new RawModel("/home/mprats/uji/PA10/3DModel/pa10link2.raw",grey,RawModel::INPUT_METERS);
   RawModel *link3=new RawModel("/home/mprats/uji/PA10/3DModel/pa10link3.raw",grey,RawModel::INPUT_METERS);
   RawModel *link4=new RawModel("/home/mprats/uji/PA10/3DModel/pa10link4.raw",grey,RawModel::INPUT_METERS);
   RawModel *link5=new RawModel("/home/mprats/uji/PA10/3DModel/pa10link5.raw",grey,RawModel::INPUT_METERS);
   RawModel *link6=new RawModel("/home/mprats/uji/PA10/3DModel/pa10link6.raw",grey,RawModel::INPUT_METERS);
   RawModel *link7=new RawModel("/home/mprats/uji/PA10/3DModel/pa10link7.raw",grey,RawModel::INPUT_METERS);
   */
  GenericFileModel *link_base = new GenericFileModel("pa10link0.osg");
  GenericFileModel *link1 = new GenericFileModel("pa10link1.osg");
  GenericFileModel *link2 = new GenericFileModel("pa10link2.osg");
  GenericFileModel *link3 = new GenericFileModel("pa10link3.osg");
  GenericFileModel *link4 = new GenericFileModel("pa10link4.osg");
  GenericFileModel *link5 = new GenericFileModel("pa10link5.osg");
  GenericFileModel *link6 = new GenericFileModel("pa10link6.osg");
  GenericFileModel *link7 = new GenericFileModel("pa10link7.osg");

#ifdef _USE_ODE_	
  BBoxDynamicModel *dynBase=new BBoxDynamicModel(0.01,0.01,0.01,0.300);
  dynBase->affectedByGravity(false);
  BBoxDynamicModel *dyn1=new BBoxDynamicModel(0.01,0.01,0.01,0.300);
  dyn1->affectedByGravity(false);
  BBoxDynamicModel *dyn2=new BBoxDynamicModel(0.01,0.01,0.01,0.300);
  dyn2->affectedByGravity(false);
  BBoxDynamicModel *dyn3=new BBoxDynamicModel(0.01,0.01,0.01,0.300);
  dyn3->affectedByGravity(false);
  BBoxDynamicModel *dyn4=new BBoxDynamicModel(0.01,0.01,0.01,0.300);
  dyn4->affectedByGravity(false);
  BBoxDynamicModel *dyn5=new BBoxDynamicModel(0.01,0.01,0.01,0.300);
  dyn5->affectedByGravity(false);
  BBoxDynamicModel *dyn6=new BBoxDynamicModel(0.01,0.01,0.01,0.300);
  dyn6->affectedByGravity(false);
  BBoxDynamicModel *dyn7=new BBoxDynamicModel(0.01,0.01,0.01,0.300);
  dyn7->affectedByGravity(false);

  cerr << "creating model" << endl;
  ObjectModel *objectmBase=new ObjectModel(link_base,dynBase);
  ObjectModel *objectm1=new ObjectModel(link1,dyn1);
  ObjectModel *objectm2=new ObjectModel(link2,dyn2);
  ObjectModel *objectm3=new ObjectModel(link3,dyn3);
  ObjectModel *objectm4=new ObjectModel(link4,dyn4);
  ObjectModel *objectm5=new ObjectModel(link5,dyn5);
  ObjectModel *objectm6=new ObjectModel(link6,dyn6);
  ObjectModel *objectm7=new ObjectModel(link7,dyn7);
#else
  cerr << "creating model" << endl;
  ObjectModel *objectmBase = new ObjectModel(link_base);
  ObjectModel *objectm1 = new ObjectModel(link1);
  ObjectModel *objectm2 = new ObjectModel(link2);
  ObjectModel *objectm3 = new ObjectModel(link3);
  ObjectModel *objectm4 = new ObjectModel(link4);
  ObjectModel *objectm5 = new ObjectModel(link5);
  ObjectModel *objectm6 = new ObjectModel(link6);
  ObjectModel *objectm7 = new ObjectModel(link7);
#endif

  cerr << "creating object" << endl;
  Object *obase = new Object("base", objectmBase);
  Object *o1 = new Object("link1", objectm1);
  Object *o2 = new Object("link2", objectm2);
  Object *o3 = new Object("link3", objectm3);
  Object *o4 = new Object("link4", objectm4);
  Object *o5 = new Object("link5", objectm5);
  Object *o6 = new Object("link6", objectm6);
  Object *o7 = new Object("link7", objectm7);

  //we create the joints
  cerr << "creating joints" << endl;
  vpColVector rzaxis(3), ryaxis(3), rxaxis(3), nryaxis(3);
  rxaxis = 0;
  rxaxis[0] = 1;
  ryaxis = 0;
  ryaxis[1] = 1;
  nryaxis = 0;
  nryaxis[1] = -1;
  rzaxis = 0;
  rzaxis[2] = 1;

  RJoint *q0, *q1, *q2, *q3, *q4, *q5, *q6;
  q0 = new RJoint(rzaxis, qmin[0], qmax[0]);
  q1 = new RJoint(ryaxis, qmin[1], qmax[1]);
  q2 = new RJoint(nryaxis, qmin[2], qmax[2]);
  q3 = new RJoint(ryaxis, qmin[3], qmax[3]);
  q4 = new RJoint(nryaxis, qmin[4], qmax[4]);
  q5 = new RJoint(ryaxis, qmin[5], qmax[5]);
  q6 = new RJoint(nryaxis, qmin[6], qmax[6]);
  cerr << "qini is: " << qini.t() << endl;
  q0->setValue(qini[0]);
  q1->setValue(qini[1]);
  q2->setValue(qini[2]);
  q3->setValue(qini[3]);
  q4->setValue(qini[4]);
  q5->setValue(qini[5]);
  q6->setValue(qini[6]);

#ifdef _USE_ODE
  RJointDynamicModel *q0_dmodel, *q1_dmodel, *q2_dmodel, *q3_dmodel, *q4_dmodel, *q5_dmodel, *q6_dmodel;
  q0_dmodel=new RJointDynamicModel(rzaxis,qmin[0],qmax[0],0);
  q1_dmodel=new RJointDynamicModel(ryaxis,qmin[1],qmax[1],0);
  q2_dmodel=new RJointDynamicModel(nryaxis,qmin[2],qmax[2],0);
  q3_dmodel=new RJointDynamicModel(ryaxis,qmin[3],qmax[3],0);
  q4_dmodel=new RJointDynamicModel(nryaxis,qmin[4],qmax[4],0);
  q5_dmodel=new RJointDynamicModel(ryaxis,qmin[5],qmax[5],0);
  q6_dmodel=new RJointDynamicModel(nryaxis,qmin[6],qmax[6],0);
  q0_dmodel->hasMotor(10);
  q1_dmodel->hasMotor(10);
  q2_dmodel->hasMotor(10);
  q3_dmodel->hasMotor(10);
  q4_dmodel->hasMotor(10);
  q5_dmodel->hasMotor(10);
  q6_dmodel->hasMotor(10);
  q0->setDynamicModel(q0_dmodel);
  q1->setDynamicModel(q1_dmodel);
  q2->setDynamicModel(q2_dmodel);
  q3->setDynamicModel(q3_dmodel);
  q4->setDynamicModel(q4_dmodel);
  q5->setDynamicModel(q5_dmodel);
  q6->setDynamicModel(q6_dmodel);
#endif

  //we attach links and joints	
  vpHomogeneousMatrix lbMq0(0, 0, 0, 0, 0, 0);
  vpHomogeneousMatrix l1Mq0(0, 0, -0.115, 0, 0, 0);

  vpHomogeneousMatrix l1Mq1(0, 0, 0, 0, 0, 0);
  vpHomogeneousMatrix l2Mq1(0, 0, 0, M_PI_2, 0, 0);

  vpHomogeneousMatrix l2Mq2(0, 0, 0, 0, 0, 0);
  vpHomogeneousMatrix l3Mq2(0, 0, -0.45, -M_PI_2, 0, 0);

  vpHomogeneousMatrix l3Mq3(0, 0, 0, 0, 0, 0);
  vpHomogeneousMatrix l4Mq3(0, 0, 0, M_PI_2, 0, 0);

  vpHomogeneousMatrix l4Mq4(0, 0, 0, 0, 0, 0);
  vpHomogeneousMatrix l5Mq4(0, 0, -0.48, -M_PI_2, 0, 0);

  vpHomogeneousMatrix l5Mq5(0, 0, 0, 0, 0, 0);
  vpHomogeneousMatrix l6Mq5(0, 0, 0, M_PI_2, 0, 0);

  vpHomogeneousMatrix l6Mq6(0, 0, 0, 0, 0, 0);
  vpHomogeneousMatrix l7Mq6(0, 0, -0.07, -M_PI_2, 0, 0);

  cerr << "linking joints" << endl;
  obase->linkTo(lbMq0, q0, l1Mq0, o1);
  o1->linkTo(l1Mq1, q1, l2Mq1, o2);
  o2->linkTo(l2Mq2, q2, l3Mq2, o3);
  o3->linkTo(l3Mq3, q3, l4Mq3, o4);
  o4->linkTo(l4Mq4, q4, l5Mq4, o5);
  o5->linkTo(l5Mq5, q5, l6Mq5, o6);
  o6->linkTo(l6Mq6, q6, l7Mq6, o7);

  cerr << "done" << endl;
  return obase;
}

vpHomogeneousMatrix SimulatedPA10::getPosition()
{
  vpHomogeneousMatrix rMe = directKinematics(getJointValues());

  return rMe;
}

vpColVector SimulatedPA10::getJointValues()
{
  //get current joint values
  vpColVector q(7);

  q[0] = ((RJoint*)object->linkedTo[0])->current_value;

  q[1] = ((RJoint*)object->linkedTo[0]->next->linkedTo[0])->current_value;

  q[2] = ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;

  q[3] = ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;

  q[4] =
      ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;

  q[5] =
      ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;

  q[6] =
      ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;

  return q;
}

void SimulatedPA10::setJointValues(vpColVector dq)
{
  cerr << "SimulatedPA10::setJointValues ERROR: Not yet implemented" << endl;
}

void SimulatedPA10::setJointVelocity(vpColVector qdot)
{
#ifdef _USE_ODE_
  ((RJointDynamicModel*)object->linkedTo[0]->dynamic_model)->setVelocity(qdot[0]);
  ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qdot[1]);
  ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qdot[2]);
  ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qdot[3]);
  ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qdot[4]);
  ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qdot[5]);
  ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qdot[6]);
#else
  ((RJoint*)object->linkedTo[0])->addValue(qdot[0]);
  ((RJoint*)object->linkedTo[0]->next->linkedTo[0])->addValue(qdot[1]);
  ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->addValue(qdot[2]);
  ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->addValue(qdot[3]);
  ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->addValue(
      qdot[4]);
  ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->addValue(
      qdot[5]);
  ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->addValue(
      qdot[6]);
#endif
}

bool SimulatedPA10::checkJointLimits(vector<int> &inlimit)
{
  //Check joint limits:
  vpColVector q(7);
  q = getJointValues();
  inlimit.clear();

  if (q[0] <= qmin[0] || q[0] >= qmax[0])
  {
    inlimit.push_back(0);
    cerr << "SimulatedPA10::setJointVelocity ERROR: Joint 0 is in joint limit" << endl;
  }
  else if (q[1] <= qmin[1] || q[1] >= qmax[1])
  {
    inlimit.push_back(1);
    cerr << "SimulatedPA10::setJointVelocity ERROR: Joint 1 is in joint limit" << endl;
  }
  else if (q[2] <= qmin[2] || q[2] >= qmax[2])
  {
    inlimit.push_back(2);
    cerr << "SimulatedPA10::setJointVelocity ERROR: Joint 2 is in joint limit" << endl;
  }
  else if (q[3] <= qmin[3] || q[3] >= qmax[3])
  {
    inlimit.push_back(3);
    cerr << "SimulatedPA10::setJointVelocity ERROR: Joint 3 is in joint limit" << endl;
  }
  else if (q[4] <= qmin[4] || q[4] >= qmax[4])
  {
    inlimit.push_back(4);
    cerr << "SimulatedPA10::setJointVelocity ERROR: Joint 4 is in joint limit" << endl;
  }
  else if (q[5] <= qmin[5] || q[5] >= qmax[5])
  {
    inlimit.push_back(5);
    cerr << "SimulatedPA10::setJointVelocity ERROR: Joint 5 is in joint limit" << endl;
  }
  else if (q[6] <= qmin[6] || q[6] >= qmax[6])
  {
    inlimit.push_back(6);
    cerr << "SimulatedPA10::setJointVelocity ERROR: Joint 6 is in joint limit" << endl;
  }
  return (inlimit.size() > 0);
}

void SimulatedPA10::setCartesianVelocity(vpColVector xdot)
{
  vpColVector qv(7), q(7);
// 	cerr << "getjointvalues" << endl;
  q = getJointValues();
// 	cerr << "jacobian77" << endl;
  vpMatrix J = jacobian77(q);
  vpHomogeneousMatrix pMe;
  pMe[2][3] = 0.15;
  vpVelocityTwistMatrix pMe6(pMe);
  vpMatrix pMe6m = pMe6;
  J = pMe6m * J;

// 	for (int i=4; i<5; i++)
// 		for (int j=0;j<8;j++) {
// 			J[i][j]=0;
// 		}

// 	cerr << "jacobian inverse" << endl;
  vpMatrix Jiv = J.pseudoInverse();

  vpColVector costvector(7);
  float hs = 0; //cost function value
  for (int i = 0; i < 7; i++)
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
  //  cerr << "cost_function: " << hs << endl;
  //  cerr << "costvector is: " << costvector.t();

  vpMatrix I(7, 7);
  I.setIdentity();
  vpColVector e2(7);
  e2 = -10.0 * (I - Jiv * J) * costvector;
//	cerr << "secondary task: " << e2.t();

  vpColVector e1(7);
  e1 = Jiv * xdot;
//	cerr << "primary task: " << e1.t();

  qv = e1 + e2;
//  	qv=Jiv*xdot;

  setJointVelocity(qv);
}

void SimulatedPA10::DHTransformationMatrix(double theta, double alpha, double a, double d, vpHomogeneousMatrix &result)
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

void DHnew(double theta, double alpha, double a, double d, vpHomogeneousMatrix &result)
{
  vpHomogeneousMatrix Ralpha(0.0, 0.0, 0.0, alpha, 0.0, 0.0);
  vpHomogeneousMatrix Rtheta(0.0, 0.0, 0.0, 0.0, 0.0, theta);
  vpHomogeneousMatrix Td(0.0, 0.0, d, 0.0, 0.0, 0.0);

  result = Ralpha * Rtheta * Td;
}

vpHomogeneousMatrix SimulatedPA10::directKinematics(vpColVector q)
{

  vpHomogeneousMatrix wMe;

  vpHomogeneousMatrix tmpMatrix;

  DHnew(q[0], 0, 0.0, 0.317, tmpMatrix);
  wMe = wMe * tmpMatrix;
  ;

  DHnew(q[1], -M_PI_2, 0.0, 0.0, tmpMatrix);
  wMe = wMe * tmpMatrix;

  DHnew(q[2], M_PI_2, 0, 0.45, tmpMatrix);
  wMe = wMe * tmpMatrix;

  DHnew(q[3], -M_PI_2, 0.0, 0, tmpMatrix);
  wMe = wMe * tmpMatrix;

  DHnew(q[4], M_PI_2, 0.0, 0.48, tmpMatrix);
  wMe = wMe * tmpMatrix;

  DHnew(q[5], -M_PI_2, 0.0, 0.0, tmpMatrix);
  wMe = wMe * tmpMatrix;

  DHnew(q[6], M_PI_2, 0.0, 0.07, tmpMatrix);

  wMe = wMe * tmpMatrix;

  return wMe;
}

// vpMatrix SimulatedPA10::newJacobian(vpColVector q) {
// 	vpColVector z0(3), p0(4);
// 	z0=0; z0[2]=1;
// 	p0=0; p0[3]=1;
// 	
// 	vpColVector p(4);
// 	p=p0;
// 
// 	vpHomogeneousMatrix Aij[8][8];
// 	for (int i=0; i<8; i++){
// 		//make all diagonal matrices unit Tii=eye(4)
// 		Aij[i][i].setIdentity();	
// 	}
// 
// 	DHnew(q[0], 0, 0.0, 0.317 ,Aij[0][1]);
// 	DHnew(q[1], -M_PI_2, 0.0, 0.0, Aij[1][2]);
// 	DHnew(q[2], M_PI_2, 0, 0.45, Aij[2][3]);
// 	DHnew(q[3], -M_PI_2,  0.0, 0 , Aij[3][4]);
// 	DHnew(q[4], M_PI_2, 0.0, 0.48 , Aij[4][5]);
// 	DHnew(q[5], -M_PI_2, 0.0, 0.0, Aij[5][6]);
// 	DHnew(q[6],  M_PI_2,  0.0,  0.07, Aij[6][7]);
// 
// 	for (uint i=7; i>0; i--) {
// 		p=Aij[i-1][i]*p;
// 	}
// 
// 	vpColVector rp0(3), rp(3);
// 	for (int i=0; i<3; i++)	{rp0[i]=p0[i]; rp[i]=p[i];}
// 	vpColVector zpp(3);
// 	zpp=vpColVector::cross(z0,(rp-rp0));
// 
// 	vpMatrix J1(6,1);
// 	J1[0][0]=zpp[0]; J1[1][0]=zpp[1]; J1[2][0]=zpp[2];
// 	J1[3][0]=z0[0]; J1[4][0]=z0[1]; J1[5][0]=z0[2];
// 
// 	vpMatrix J;
// 	J=J1;
// 
// 	for (uint i=2; i<8; i++) {
// 		vpColVector zi1, pi1;
// 		zi1=z0;
// 		pi1=p0;
// 		for (uint j=i-1; j>0; j--) {
// 			vpRotationMatrix Rij;
// 			Aij[j-1][j].extract(Rij);
// 			vpMatrix MRij;
// 			MRij=Rij;
// 			zi1=MRij*zi1;
// 			pi1=Aij[j-1][j]*pi1;
// 		}
// 		
// 		vpColVector rpi1(3);
// 		for (int i=0; i<3; i++)	{rpi1[i]=pi1[i];}
// 		vpColVector zi1pp(3);
// 		zi1pp=vpColVector::cross(zi1,(rp-rpi1));
// 		vpMatrix Ji(6,1);
// 		Ji[0][0]=zi1pp[0]; Ji[1][0]=zi1pp[1]; Ji[2][0]=zi1pp[2];
// 		Ji[3][0]=zi1[0]; Ji[4][0]=zi1[1]; Ji[5][0]=zi1[2];
// 
// 		J=vpMatrix::juxtaposeMatrices(J,Ji);
// 	}
// 	return J;
// }

vpMatrix SimulatedPA10::jacobian07(vpColVector q)
{
// 	cerr << "q inside jacobian is: " << q.t() << endl;
  vpMatrix JacResult(6, 7);

  vpHomogeneousMatrix tmpMatrix;

  //array of matrices for use to define Jacobian
  vpMatrix Tij[8][8]; //array of transf. matrices between joints

  //define the size of transf. matrices
  for (int i = 0; i < 8; i++)
  {
    for (int j = 0; j < 8; j++)
    {
      Tij[i][j].resize(4, 4);
    }
    //make all diagonal matrices unit Tii=eye(4)
    Tij[i][i].setIdentity();
  }

  DHnew(q[0], 0, 0.0, 0.317, tmpMatrix);
  Tij[0][1] = Tij[0][0] * tmpMatrix;
// 	  cerr << "T01: " << endl << Tij[0][1] << endl;

  DHnew(q[1], -M_PI_2, 0.0, 0.0, tmpMatrix);
  Tij[1][2] = tmpMatrix;
// 	  cerr << "T12: " << endl << Tij[1][2] << endl;

  DHnew(q[2], M_PI_2, 0, 0.45, tmpMatrix);
  Tij[2][3] = tmpMatrix;

  DHnew(q[3], -M_PI_2, 0.0, 0, tmpMatrix);
  Tij[3][4] = tmpMatrix;

  DHnew(q[4], M_PI_2, 0.0, 0.48, tmpMatrix);
  Tij[4][5] = tmpMatrix;

  DHnew(q[5], -M_PI_2, 0.0, 0.0, tmpMatrix);
  Tij[5][6] = tmpMatrix;

  DHnew(q[6], M_PI_2, 0.0, 0.07, tmpMatrix);
  Tij[6][7] = tmpMatrix;

  //define also the elements above diagonal
  for (int i = 0; i < 8; i++)
    for (int j = i + 2; j < 8; j++)
      Tij[i][j] = Tij[i][j - 1] * Tij[j - 1][j];

  vpMatrix JacL(3, 7), JacA(3, 7);

  for (int i = 1; i <= 7; i++)
  {
    //see Mathematica code (Robotica for Mathematica function FormTheJacobianJ[])
    vpColVector a(3), b(3), c(3);
    a[0] = Tij[0][i][0][2];
    a[1] = Tij[0][i][1][2];
    a[2] = Tij[0][i][2][2];
    b[0] = Tij[0][7][0][3] - Tij[0][i][0][3];
    b[1] = Tij[0][7][1][3] - Tij[0][i][1][3];
    b[2] = Tij[0][7][2][3] - Tij[0][i][2][3];

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

  return JacResult;
}

vpMatrix SimulatedPA10::jacobian77(vpColVector q)
{
  //Robot jacobian in base frame
  vpMatrix J07(6, 7);
  J07 = jacobian07(q);
  //Direct geometric model
  vpHomogeneousMatrix bMe;
  bMe = directKinematics(q);

  //Robot jacobian J(q) in effector frame: 7J7
  vpMatrix J77(6, 7);
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

vpMatrix SimulatedPA10::jacobian77Inverse(vpColVector q)
{
  //Robot jacobian J(q) in EEF frame: 7J7
  vpMatrix J77(6, 7);
  J77 = jacobian77(q);

  return J77.pseudoInverse();
}

SimulatedPA10::~SimulatedPA10()
{
}

