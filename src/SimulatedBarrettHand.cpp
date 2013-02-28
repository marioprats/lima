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

#include <visp/vpRGBa.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpColVector.h>

#include <lima/SimulatedBarrettHand.h>
#include <lima/BBoxModel.h>
#include <lima/ObjectModel.h>
#include <lima/CylinderModel.h>
#include <lima/RawModel.h>
#include <lima/RJoint.h>
#include <lima/Object.h>
#ifdef _USE_ODE_
#include <lima/BBoxDynamicModel.h>
#include <lima/RJointDynamicModel.h>
#endif

SimulatedBarrettHand::SimulatedBarrettHand() :
    SimulatedHand()
{
  //initialize desired finger position
  this->dq.resize(4);
  dq = 0;
  setObject(createObject());
}

Object *createBase()
{
  vpRGBa yellow(200, 200, 0, 0);
  RawModel *o_gmodel = new RawModel("/tmp/palm.raw", yellow, RawModel::INPUT_MILLIMETERS);
//  	CylinderModel *o_gmodel=new CylinderModel(0.089/2,0.089/2,0.0829,yellow);
#ifdef _USE_ODE_
  BBoxDynamicModel *o_dmodel=new BBoxDynamicModel(0.089,0.089,0.0504,0.3);
  o_dmodel->affectedByGravity(false);
  ObjectModel *o_model=new ObjectModel(o_gmodel,o_dmodel);
#else
  ObjectModel *o_model = new ObjectModel(o_gmodel);
#endif
  Object *o = new Object("BarrettBase", o_model);
  return o;
}

Object *createOPhalanx()
{
  vpRGBa grey(200, 200, 200, 0);
// 	RawModel *o_gmodel=new RawModel("/tmp/link2.raw",grey,RawModel::INPUT_MILLIMETERS);
  BBoxModel *o_gmodel = new BBoxModel(0.015, 0.056, 0.02, grey);
#ifdef _USE_ODE_
  BBoxDynamicModel *o_dmodel=new BBoxDynamicModel(0.015,0.056,0.02,0.05);
  o_dmodel->affectedByGravity(false);
  ObjectModel *o_model=new ObjectModel(o_gmodel,o_dmodel);
#else
  ObjectModel *o_model = new ObjectModel(o_gmodel);
#endif
  Object *o = new Object("BarrettOPhalanx", o_model);
  return o;
}

Object *createPPhalanx()
{
  vpRGBa grey(200, 200, 200, 0);
  BBoxModel *o_gmodel = new BBoxModel(0.015, 0.07, 0.02, grey);
// 	RawModel *o_gmodel=new RawModel("/tmp/link3.raw",grey,RawModel::INPUT_MILLIMETERS);
#ifdef _USE_ODE_
  BBoxDynamicModel *o_dmodel=new BBoxDynamicModel(0.015,0.07,0.02,0.05);
  o_dmodel->affectedByGravity(false);
  ObjectModel *o_model=new ObjectModel(o_gmodel,o_dmodel);
#else
  ObjectModel *o_model = new ObjectModel(o_gmodel);
#endif
  Object *o = new Object("BarrettPPhalanx", o_model);

  Object *op = createOPhalanx();

  vpColVector rjointaxis(3);
  rjointaxis = 0;
  rjointaxis[0] = 1;
  RJoint *rjoint1 = new RJoint(rjointaxis, 0, M_PI_2);
#ifdef _USE_ODE_
  RJointDynamicModel *rjoint1_dmodel=new RJointDynamicModel(rjointaxis,-0.1,M_PI_2+0.1,0);
  rjoint1_dmodel->hasMotor(2);
  rjoint1->setDynamicModel(rjoint1_dmodel);
#endif

  vpHomogeneousMatrix pMj1(0, -0.035, 0, 0, 0, 0);

  vpHomogeneousMatrix opMj(0, 0.028, 0, M_PI_2, 0, 0);

  o->linkTo(pMj1, rjoint1, opMj, op);

  return o;
}

// Object *createFinger() {
// 	vpRGBa grey(200,200,200,0);
// // 	BBoxModel *o_gmodel=new BBoxModel(0.015,0.07,0.02,grey);
// 	RawModel *o_gmodel=new RawModel("/tmp/link1_lowres.raw",grey,RawModel::INPUT_MILLIMETERS);
// #ifdef _USE_ODE_
// 	BBoxDynamicModel *o_dmodel=new BBoxDynamicModel(0.015,0.07,0.02,0.05);
// 	o_dmodel->affectedByGravity(false);
// 	ObjectModel *o_model=new ObjectModel(o_gmodel,o_dmodel);
// #else
// 	ObjectModel *o_model=new ObjectModel(o_gmodel);
// #endif
// 	Object *o=new Object("BarrettPPhalanx",o_model);
// 
// 	Object *op=createPPhalanx();
// 	
// 	vpColVector rjointaxis(3);
// 	rjointaxis=0; rjointaxis[0]=1;
// 	RJoint *rjoint1=new RJoint(rjointaxis,0,M_PI_2);
// #ifdef _USE_ODE_
// 	RJointDynamicModel *rjoint1_dmodel=new RJointDynamicModel(rjointaxis,-0.1,M_PI_2+0.1,0);
// 	rjoint1_dmodel->hasMotor(2);
// 	rjoint1->setDynamicModel(rjoint1_dmodel);
// #endif
// 
// 	vpHomogeneousMatrix pMj1(0,-0.035,0,0,0,0);
// 	
// 	vpHomogeneousMatrix opMj(0,0.028,0,0.985,0,0);
// 
// 	o->linkTo(pMj1,rjoint1,opMj,op);
// 	
// 	return o;
// }

Object *SimulatedBarrettHand::createObject()
{
  Object *base = createBase();
  Object *pp1 = createPPhalanx();
  Object *pp2 = createPPhalanx();
  Object *pp3 = createPPhalanx();

  vpColVector rjointaxis(3);
  rjointaxis = 0;
  rjointaxis[0] = -1;
  RJoint *rjoint1 = new RJoint(rjointaxis, 0, M_PI);
  RJoint *rjoint2 = new RJoint(rjointaxis, 0, M_PI);
  RJoint *rjoint3 = new RJoint(rjointaxis, 0, M_PI);
  rjoint1->setValue(M_PI_2 + 0.5);
  rjoint2->setValue(M_PI_2 + 0.5);
  rjoint3->setValue(M_PI_2 + 0.5);

#ifdef _USE_ODE_
  RJointDynamicModel *rjoint1_dmodel=new RJointDynamicModel(rjointaxis,-0.1,M_PI+0.1,0);
  RJointDynamicModel *rjoint2_dmodel=new RJointDynamicModel(rjointaxis,-0.1,M_PI+0.1,0);
  RJointDynamicModel *rjoint3_dmodel=new RJointDynamicModel(rjointaxis,-0.1,M_PI+0.1,0);
  rjoint1_dmodel->hasMotor(2);
  rjoint2_dmodel->hasMotor(2);
  rjoint3_dmodel->hasMotor(2);
  rjoint1->setDynamicModel(rjoint1_dmodel);
  rjoint2->setDynamicModel(rjoint2_dmodel);
  rjoint3->setDynamicModel(rjoint3_dmodel);
#endif

  vpHomogeneousMatrix pMj1(0, -0.05, 0, 0, 0, 0);
  vpHomogeneousMatrix pMj2(0.025, -0.05, 0, 0, 0, 0);
  vpHomogeneousMatrix pMj3(-0.025, -0.05, 0, 0, 0, 0);

  vpHomogeneousMatrix ppMj(0, 0.035, 0, 0, 0, 0);

  base->linkTo(pMj1, rjoint1, ppMj, pp1);
  base->linkTo(pMj2, rjoint2, ppMj, pp2);
  base->linkTo(pMj3, rjoint3, ppMj, pp3);

  return base;
}

void SimulatedBarrettHand::setPosition(float f1, float f2, float f3, float spread)
{
  vpColVector dq(4);
  dq[0] = f1;
  dq[1] = f2;
  dq[2] = f3;
  dq[3] = spread;
  Hand::setPosition(dq);
}

int SimulatedBarrettHand::sendControl()
{
  cerr << "SimulatedBarrettHand::sendControl() spread still NOT IMPLEMENTED" << endl;

  //read current joint values
  vpColVector q(4);
  q[0] = ((RJoint*)object->linkedTo[0])->current_value;
  q[1] = ((RJoint*)object->linkedTo[1])->current_value;
  q[2] = ((RJoint*)object->linkedTo[2])->current_value;
  q[3] = 0;

  //control
  vpColVector qc(16);
#ifdef _USE_ODE_
  qc=2*(dq-q);
  cerr << "qc is : " << qc.t() << endl;

  //send joint velocities
  ((RJointDynamicModel*)object->linkedTo[0]->dynamic_model)->setVelocity(qc[0]);
  ((RJointDynamicModel*)object->linkedTo[1]->dynamic_model)->setVelocity(qc[1]);
  ((RJointDynamicModel*)object->linkedTo[2]->dynamic_model)->setVelocity(qc[2]);
// 	((RJointDynamicModel*)object->linkedTo[3]->dynamic_model)->setVelocity(qc[3]);
#else
  qc = 0.05 * (dq - q);

  ((RJoint*)object->linkedTo[0])->addValue(qc[0]);
  ((RJoint*)object->linkedTo[1])->addValue(qc[1]);
  ((RJoint*)object->linkedTo[2])->addValue(qc[2]);
// 	((RJoint*)object->linkedTo[3])->addValue(qc[3]);
#endif
  return 1;
}

int SimulatedBarrettHand::executePreshape(HandPreshape *p)
{
  cerr << "SimulatedBarrettHand::executePreshape() STILL NOT IMPLEMENTED" << endl;
  return 0;
}

int SimulatedBarrettHand::computeHandFrameMatrix(HandFrame *handframe, vpHomogeneousMatrix &hMhf)
{
  cerr << "SimulatedBarrettHand::computeHandFrameMatrix() STILL NOT IMPLEMENTED" << endl;
  return 0;
}

SimulatedBarrettHand::~SimulatedBarrettHand()
{
  //TODO: destroy the object
}

