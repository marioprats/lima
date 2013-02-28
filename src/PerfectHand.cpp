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

#include <lima/PerfectHand.h>
#include <lima/PerfectHandPreshape.h>
#include <lima/PerfectHandFrame.h>
#include <lima/PerfectHandFingertip.h>
#include <lima/PerfectHandMPhalanx.h>
#include <lima/PerfectHandOPhalanx.h>
#include <lima/PerfectHandPalm.h>
#include <lima/BBoxModel.h>
#include <lima/ObjectModel.h>
#include <lima/RJoint.h>
#include <lima/Object.h>
#ifdef _USE_ODE_
#include <lima/BBoxDynamicModel.h>
#include <lima/RJointDynamicModel.h>
#endif

PerfectHand::PerfectHand(handtype side) :
    SimulatedHand()
{
  //initialize desired finger position
  this->dq.resize(16);
  dq = 0;
  hand_side = side;
  setObject(createObject());
}

Object *createPhalanx(float x, float y, float z, float weight)
{
  vpRGBa blue(100, 100, 250, 0);
  //vpRGBa blue(255,100,100,0);
  BBoxModel *o_gmodel = new BBoxModel(x, y, z, blue);
#ifdef _USE_ODE_
  BBoxDynamicModel *o_dmodel=new BBoxDynamicModel(x,y,z,weight);
  o_dmodel->affectedByGravity(false);
  ObjectModel *o_model=new ObjectModel(o_gmodel,o_dmodel);
#else
  ObjectModel *o_model = new ObjectModel(o_gmodel);
#endif
  Object *o = new Object("Phalanx", o_model);
  return o;
}

Object *createFinger(float size)
{
  Object *p1 = createPhalanx(size * 0.01, size * 0.03, size * 0.01, size * 0.01);
  Object *p2 = createPhalanx(size * 0.01, size * 0.02, size * 0.008, size * 0.005);
  Object *p3 = createPhalanx(size * 0.008, size * 0.016, size * 0.005, size * 0.002);

  vpColVector rjointaxis(3);
  rjointaxis = 0;
  rjointaxis[0] = 1;
  RJoint *rjoint1 = new RJoint(rjointaxis, 0, M_PI_2);
  RJoint *rjoint2 = new RJoint(rjointaxis, 0, M_PI_2);
#ifdef _USE_ODE_
  RJointDynamicModel *rjoint1_dmodel=new RJointDynamicModel(rjointaxis,-0.1,M_PI_2+0.1,0);
  RJointDynamicModel *rjoint2_dmodel=new RJointDynamicModel(rjointaxis,-0.1,M_PI_2+0.1,0);
  rjoint1_dmodel->hasMotor(size*2);
  rjoint2_dmodel->hasMotor(size*2);
  rjoint1->setDynamicModel(rjoint1_dmodel);
  rjoint2->setDynamicModel(rjoint2_dmodel);
#endif

  vpHomogeneousMatrix p1Mj1;
  p1Mj1.setIdentity();
  p1Mj1[1][3] = size * 0.019;
  vpHomogeneousMatrix p2Mj1;
  p2Mj1.setIdentity();
  p2Mj1[1][3] = -size * 0.014;
  vpHomogeneousMatrix p2Mj2;
  p2Mj2.setIdentity();
  p2Mj2[1][3] = size * 0.014;
  vpHomogeneousMatrix p3Mj2;
  p3Mj2.setIdentity();
  p3Mj2[1][3] = -size * 0.010;
  p1->linkTo(p1Mj1, rjoint1, p2Mj1, p2);
  p2->linkTo(p2Mj2, rjoint2, p3Mj2, p3);

  return p1;
}

Object *PerfectHand::createObject()
{
  //create the object and attach it to the hand
  vpRGBa red(150, 150, 150, 0);
  //vpRGBa red(255,50,50,0);
  BBoxModel *palm_gmodel = new BBoxModel(0.055, 0.08, 0.02, red);
  Object *f1 = createFinger(1);
  Object *f2 = createFinger(1);
  Object *f3 = createFinger(1);
  Object *f4 = createFinger(1);
  Object *f5 = createFinger(0.7);
  BBoxModel *thumbase_gmodel = new BBoxModel(0.01, 0.01, 0.01, red);

#ifdef _USE_ODE_
  BBoxDynamicModel *palm_dmodel=new BBoxDynamicModel(0.05,0.08,0.02,0.4);
  palm_dmodel->affectedByGravity(false);
  ObjectModel *palm_model=new ObjectModel(palm_gmodel,palm_dmodel);
  BBoxDynamicModel *thumbase_dmodel=new BBoxDynamicModel(0.01,0.01,0.01,0.02);
  thumbase_dmodel->affectedByGravity(false);
  ObjectModel *thumbase_model=new ObjectModel(thumbase_gmodel,thumbase_dmodel);
#else
  ObjectModel *palm_model = new ObjectModel(palm_gmodel);
  ObjectModel *thumbase_model = new ObjectModel(thumbase_gmodel);
#endif

  vpColVector rjointaxis(3);
  rjointaxis = 0;
  rjointaxis[0] = 1;
  vpColVector rjointyaxis(3);
  if (hand_side == LEFT)
  {
    rjointyaxis = 0;
    rjointyaxis[1] = -1;
  }
  else
  {
    rjointyaxis = 0;
    rjointyaxis[1] = 1;
  }
  RJoint *rjoint1 = new RJoint(rjointaxis, 0, M_PI_2);
  RJoint *rjoint2 = new RJoint(rjointaxis, 0, M_PI_2);
  RJoint *rjoint3 = new RJoint(rjointaxis, 0, M_PI_2);
  RJoint *rjoint4 = new RJoint(rjointaxis, 0, M_PI_2);
  RJoint *rjoint5 = new RJoint(rjointyaxis, -M_PI_2, 0);
  RJoint *rjoint6 = new RJoint(rjointaxis, -M_PI_2, M_PI_4);
//  	rjoint5->setValue(-0.4);
#ifdef _USE_ODE_
  RJointDynamicModel *rjoint1_dmodel=new RJointDynamicModel(rjointaxis,-0.1,M_PI_2+0.1,0);
  RJointDynamicModel *rjoint2_dmodel=new RJointDynamicModel(rjointaxis,-0.1,M_PI_2+0.1,0);
  RJointDynamicModel *rjoint3_dmodel=new RJointDynamicModel(rjointaxis,-0.1,M_PI_2+0.1,0);
  RJointDynamicModel *rjoint4_dmodel=new RJointDynamicModel(rjointaxis,-0.1,M_PI_2+0.1,0);
  RJointDynamicModel *rjoint5_dmodel=new RJointDynamicModel(rjointyaxis,-M_PI_2-0.1,0.1,0);
  RJointDynamicModel *rjoint6_dmodel=new RJointDynamicModel(rjointaxis,-0.1-M_PI_2,M_PI_4+0.1,0);
  rjoint1_dmodel->hasMotor(2);
  rjoint2_dmodel->hasMotor(2);
  rjoint3_dmodel->hasMotor(2);
  rjoint4_dmodel->hasMotor(2);
  rjoint5_dmodel->hasMotor(2);
  rjoint6_dmodel->hasMotor(2);
  rjoint1->setDynamicModel(rjoint1_dmodel);
  rjoint2->setDynamicModel(rjoint2_dmodel);
  rjoint3->setDynamicModel(rjoint3_dmodel);
  rjoint4->setDynamicModel(rjoint4_dmodel);
  rjoint5->setDynamicModel(rjoint5_dmodel);
  rjoint6->setDynamicModel(rjoint6_dmodel);
#endif
  Object *palm = new Object("Palm", palm_model);
  Object *thumbase = new Object("Thumb base", thumbase_model);
  if (hand_side == RIGHT)
  {
    vpHomogeneousMatrix pMj1;
    pMj1.setIdentity();
    pMj1[0][3] = -0.0225;
    pMj1[1][3] = 0.045;
    vpHomogeneousMatrix pMj2;
    pMj2.setIdentity();
    pMj2[0][3] = -0.0075;
    pMj2[1][3] = 0.045;
    vpHomogeneousMatrix pMj3;
    pMj3.setIdentity();
    pMj3[0][3] = 0.0075;
    pMj3[1][3] = 0.045;
    vpHomogeneousMatrix pMj4;
    pMj4.setIdentity();
    pMj4[0][3] = 0.0225;
    pMj4[1][3] = 0.045;
    vpHomogeneousMatrix fMj;
    fMj.setIdentity();
    fMj[1][3] = -0.015;
    vpRxyzVector rv(0, 0, 0);
    vpTranslationVector tv(0.0125, 0.01, 0.02);
    vpRotationMatrix rm(rv);
    vpHomogeneousMatrix pMj5(tv, rm);
    vpRxyzVector rv2(M_PI_4 / 2, M_PI_2, 0);
    vpTranslationVector tv2(0, 0, 0.029);
    vpRotationMatrix rm2(rv2);
    vpHomogeneousMatrix f5bMj(tv2, rm2);
    vpHomogeneousMatrix f5bMj6;
    f5bMj6.setIdentity();
    f5bMj6[1][3] = 0.010;
    vpHomogeneousMatrix f5Mj6;
    f5Mj6.setIdentity();
    f5Mj6[1][3] = -0.012;
    palm->linkTo(pMj1, rjoint1, fMj, f1);
    palm->linkTo(pMj2, rjoint2, fMj, f2);
    palm->linkTo(pMj3, rjoint3, fMj, f3);
    palm->linkTo(pMj4, rjoint4, fMj, f4);
    palm->linkTo(pMj5, rjoint5, f5bMj, thumbase);
    thumbase->linkTo(f5bMj6, rjoint6, f5Mj6, f5);
  }
  else
  {
    vpHomogeneousMatrix pMj1;
    pMj1.setIdentity();
    pMj1[0][3] = 0.0225;
    pMj1[1][3] = 0.045;
    vpHomogeneousMatrix pMj2;
    pMj2.setIdentity();
    pMj2[0][3] = 0.0075;
    pMj2[1][3] = 0.045;
    vpHomogeneousMatrix pMj3;
    pMj3.setIdentity();
    pMj3[0][3] = -0.0075;
    pMj3[1][3] = 0.045;
    vpHomogeneousMatrix pMj4;
    pMj4.setIdentity();
    pMj4[0][3] = -0.0225;
    pMj4[1][3] = 0.045;
    vpHomogeneousMatrix fMj;
    fMj.setIdentity();
    fMj[1][3] = -0.015;
    vpRxyzVector rv(0, 0, 0);
    vpTranslationVector tv(-0.0125, 0.01, 0.02);
    vpRotationMatrix rm(rv);
    vpHomogeneousMatrix pMj5(tv, rm);
    vpRxyzVector rv2(M_PI_4 / 2, -M_PI_2, 0);
    vpTranslationVector tv2(0, 0, 0.029);
    vpRotationMatrix rm2(rv2);
    vpHomogeneousMatrix f5bMj(tv2, rm2);
    vpHomogeneousMatrix f5bMj6;
    f5bMj6.setIdentity();
    f5bMj6[1][3] = 0.010;
    vpHomogeneousMatrix f5Mj6;
    f5Mj6.setIdentity();
    f5Mj6[1][3] = -0.012;
    palm->linkTo(pMj1, rjoint1, fMj, f1);
    palm->linkTo(pMj2, rjoint2, fMj, f2);
    palm->linkTo(pMj3, rjoint3, fMj, f3);
    palm->linkTo(pMj4, rjoint4, fMj, f4);
    palm->linkTo(pMj5, rjoint5, f5bMj, thumbase);
    thumbase->linkTo(f5bMj6, rjoint6, f5Mj6, f5);
  }

  return palm;
}

void PerfectHand::setPosition(vpColVector &f1, vpColVector &f2, vpColVector &f3, vpColVector &f4, vpColVector &f5)
{
  vpColVector dq(16);
  dq[0] = f1[0];
  dq[1] = f1[1];
  dq[2] = f1[2];
  dq[3] = f2[0];
  dq[4] = f2[1];
  dq[5] = f2[2];
  dq[6] = f3[0];
  dq[7] = f3[1];
  dq[8] = f3[2];
  dq[9] = f4[0];
  dq[10] = f4[1];
  dq[11] = f4[2];
  dq[12] = f5[0];
  dq[13] = f5[1];
  dq[14] = f5[2];
  dq[15] = f5[3];
  Hand::setPosition(dq);
}

void PerfectHand::setPosition(float f11, float f12, float f13, float f21, float f22, float f23, float f31, float f32,
                              float f33, float f41, float f42, float f43, float f51, float f52, float f53, float f54)
{
  vpColVector f1(3), f2(3), f3(3), f4(3), f5(4);
  f1[0] = f11;
  f1[1] = f12;
  f1[2] = f13;
  f2[0] = f21;
  f2[1] = f22;
  f2[2] = f23;
  f3[0] = f31;
  f3[1] = f32;
  f3[2] = f33;
  f4[0] = f41;
  f4[1] = f42;
  f4[2] = f43;
  f5[0] = f51;
  f5[1] = f52;
  f5[2] = f53;
  f5[3] = f54;
  setPosition(f1, f2, f3, f4, f5);
}

int PerfectHand::sendControl()
{
  //read current joint values
  vpColVector q(16);
  q[0] = ((RJoint*)object->linkedTo[0])->current_value;
  q[1] = ((RJoint*)object->linkedTo[0]->next->linkedTo[0])->current_value;
  q[2] = ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;
  q[3] = ((RJoint*)object->linkedTo[1])->current_value;
  q[4] = ((RJoint*)object->linkedTo[1]->next->linkedTo[0])->current_value;
  q[5] = ((RJoint*)object->linkedTo[1]->next->linkedTo[0]->next->linkedTo[0])->current_value;
  q[6] = ((RJoint*)object->linkedTo[2])->current_value;
  q[7] = ((RJoint*)object->linkedTo[2]->next->linkedTo[0])->current_value;
  q[8] = ((RJoint*)object->linkedTo[2]->next->linkedTo[0]->next->linkedTo[0])->current_value;
  q[9] = ((RJoint*)object->linkedTo[3])->current_value;
  q[10] = ((RJoint*)object->linkedTo[3]->next->linkedTo[0])->current_value;
  q[11] = ((RJoint*)object->linkedTo[3]->next->linkedTo[0]->next->linkedTo[0])->current_value;
  q[12] = ((RJoint*)object->linkedTo[4])->current_value;
  q[13] = ((RJoint*)object->linkedTo[4]->next->linkedTo[0])->current_value;
  q[14] = ((RJoint*)object->linkedTo[4]->next->linkedTo[0]->next->linkedTo[0])->current_value;
  q[15] = ((RJoint*)object->linkedTo[4]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->current_value;

  //control
  vpColVector qc(16);
#ifdef _USE_ODE_
  qc=2*(dq-q);

  //send joint velocities
  ((RJointDynamicModel*)object->linkedTo[0]->dynamic_model)->setVelocity(qc[0]);
  ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qc[1]);
  ((RJointDynamicModel*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qc[2]);
  ((RJointDynamicModel*)object->linkedTo[1]->dynamic_model)->setVelocity(qc[3]);
  ((RJointDynamicModel*)object->linkedTo[1]->next->linkedTo[0]->dynamic_model)->setVelocity(qc[4]);
  ((RJointDynamicModel*)object->linkedTo[1]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qc[5]);
  ((RJointDynamicModel*)object->linkedTo[2]->dynamic_model)->setVelocity(qc[6]);
  ((RJointDynamicModel*)object->linkedTo[2]->next->linkedTo[0]->dynamic_model)->setVelocity(qc[7]);
  ((RJointDynamicModel*)object->linkedTo[2]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qc[8]);
  ((RJointDynamicModel*)object->linkedTo[3]->dynamic_model)->setVelocity(qc[9]);
  ((RJointDynamicModel*)object->linkedTo[3]->next->linkedTo[0]->dynamic_model)->setVelocity(qc[10]);
  ((RJointDynamicModel*)object->linkedTo[3]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qc[11]);
  ((RJointDynamicModel*)object->linkedTo[4]->dynamic_model)->setVelocity(qc[12]);
  ((RJointDynamicModel*)object->linkedTo[4]->next->linkedTo[0]->dynamic_model)->setVelocity(qc[13]);
  ((RJointDynamicModel*)object->linkedTo[4]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qc[14]);
  ((RJointDynamicModel*)object->linkedTo[4]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->dynamic_model)->setVelocity(qc[15]);
#else
  qc = 0.05 * (dq - q);

  ((RJoint*)object->linkedTo[0])->addValue(qc[0]);
  ((RJoint*)object->linkedTo[0]->next->linkedTo[0])->addValue(qc[1]);
  ((RJoint*)object->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->addValue(qc[2]);
  ((RJoint*)object->linkedTo[1])->addValue(qc[3]);
  ((RJoint*)object->linkedTo[1]->next->linkedTo[0])->addValue(qc[4]);
  ((RJoint*)object->linkedTo[1]->next->linkedTo[0]->next->linkedTo[0])->addValue(qc[5]);
  ((RJoint*)object->linkedTo[2])->addValue(qc[6]);
  ((RJoint*)object->linkedTo[2]->next->linkedTo[0])->addValue(qc[7]);
  ((RJoint*)object->linkedTo[2]->next->linkedTo[0]->next->linkedTo[0])->addValue(qc[8]);
  ((RJoint*)object->linkedTo[3])->addValue(qc[9]);
  ((RJoint*)object->linkedTo[3]->next->linkedTo[0])->addValue(qc[10]);
  ((RJoint*)object->linkedTo[3]->next->linkedTo[0]->next->linkedTo[0])->addValue(qc[11]);
  ((RJoint*)object->linkedTo[4])->addValue(qc[12]);
  ((RJoint*)object->linkedTo[4]->next->linkedTo[0])->addValue(qc[13]);
  ((RJoint*)object->linkedTo[4]->next->linkedTo[0]->next->linkedTo[0])->addValue(qc[14]);
  ((RJoint*)object->linkedTo[4]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0])->addValue(qc[15]);
#endif
  return 1;
}

int PerfectHand::executePreshape(HandPreshape *p)
{
  if (p->getGroupName() == string("PerfectHandPreshape"))
  {
    if (((PerfectHandPreshape*)p)->getPreshapeName() == string("PerfectHandHook"))
    {
      return executePreshape((PerfectHandHook*)p);
    }
    else if (((PerfectHandPreshape*)p)->getPreshapeName() == string("PerfectHandCylindrical"))
    {
      return executePreshape((PerfectHandCylindrical*)p);
    }
    else if (((PerfectHandPreshape*)p)->getPreshapeName() == string("PerfectHandOneFinger"))
    {
      return executePreshape((PerfectHandOneFinger*)p);
    }
    else if (((PerfectHandPreshape*)p)->getPreshapeName() == string("PerfectHandLateral"))
    {
      return executePreshape((PerfectHandLateral*)p);
    }
    else
    {
      cerr << "PerfectHand::executePreshape doesn't know how to execute preshape "
          << ((PerfectHandPreshape*)p)->getPreshapeName() << endl;
      return 0;
    }
  }
  else
  {
    cerr << "PerfectHand::executePreshape cannot execute preshapes of group " << p->getGroupName() << endl;
    return 0;
  }
  return 0;
}

int PerfectHand::executePreshape(PerfectHandHook *p)
{
  //Compute the motor angles given the desired closing percentage
  float angle = M_PI_2 * p->closing / 100;

  //Move all fingers (not the thumb) to this configuration
  //The thumb angles are put to zero.
  setPosition(angle, angle, angle, angle, angle, angle, angle, angle, angle, angle, angle, angle, 0, 0.3, 0.2, 0.2);
  return 1;
}

int PerfectHand::executePreshape(PerfectHandCylindrical *p)
{
  //Compute the motor angles given the desired closing percentage
  float angle = M_PI_2 * p->closing / 100;

  //Move all fingers to this configuration
  setPosition(angle, angle, angle, angle, angle, angle, angle, angle, angle, angle, angle, angle, -M_PI_2,
              -M_PI_2 + angle, 0.4 * p->closing / 50, 0.4 * p->closing / 50);

  /*	setPosition(angle-0.6,angle-0.6,angle-0.6,
   angle-0.6,angle-0.6,angle-0.6,
   angle-0.6,angle-0.6,angle-0.6,
   angle,angle,angle,
   -M_PI_2,-M_PI_2+angle,0.4*p->closing/50,0.4*p->closing/50);*/
  return 1;
}

int PerfectHand::executePreshape(PerfectHandOneFinger *p)
{
  float angle = M_PI_4 + 0.5;

  setPosition(angle + 0.2, angle + 0.2, angle + 0.2, angle + 0.1, angle + 0.1, angle + 0.1, angle, angle, angle, 0.2,
              0.2, 0.2, 0, 0, 0, 0);
  return 1;
}

int PerfectHand::executePreshape(PerfectHandLateral *p)
{
  //Compute the motor angles given the desired closing percentage
  float angle = M_PI_2 * p->closing / 100;

  //Move all fingers (not the thumb) to this configuration
  //The thumb angles are put to zero.
  setPosition(angle + 0.3, angle + 0.3, angle + 0.3, angle + 0.2, angle + 0.2, angle + 0.2, angle + 0.1, angle + 0.1,
              angle + 0.1, angle, angle, angle, -0.2, 0.2, 0.1, 0.1);
  return 1;
}

int PerfectHand::computeHandFrameMatrix(HandFrame *handframe, vpHomogeneousMatrix &hMhf)
{
//   cerr << "computeHAndFrameMAtrix on " << ((PerfectHandFrame*)handframe)->getFrameName() << endl; 
  if (handframe->getGroupName() == string("PerfectHandFrame"))
  {
    if (((PerfectHandFrame*)handframe)->getFrameName() == string("PerfectHandFingertip"))
    {
      return computeHandFrameMatrix((PerfectHandFingertip*)handframe, hMhf);
    }
    else if (((PerfectHandFrame*)handframe)->getFrameName() == string("PerfectHandMPhalanx"))
    {
      return computeHandFrameMatrix((PerfectHandMPhalanx*)handframe, hMhf);
    }
    else if (((PerfectHandFrame*)handframe)->getFrameName() == string("PerfectHandOPhalanx"))
    {
      return computeHandFrameMatrix((PerfectHandOPhalanx*)handframe, hMhf);
    }
    else if (((PerfectHandFrame*)handframe)->getFrameName() == string("PerfectHandFingertipToThumb"))
    {
      return computeHandFrameMatrix((PerfectHandFingertipToThumb*)handframe, hMhf);
    }
    else if (((PerfectHandFrame*)handframe)->getFrameName() == string("PerfectHandPalm"))
    {
      return computeHandFrameMatrix((PerfectHandPalm*)handframe, hMhf);
    }
    else
    {
      cerr << "PerfectHand::setHandFrame doesn't know how to set frame "
          << ((PerfectHandFrame*)handframe)->getFrameName() << endl;
      return 0;
    }
  }
  else
  {
    cerr << "PerfectHand::setHandFrame cannot set frames of group " << handframe->getGroupName() << endl;
    return 0;
  }
  return 0;
}

int PerfectHand::computeHandFrameMatrix(PerfectHandFingertip *p, vpHomogeneousMatrix &hMhf)
{
  //We start from the Outer phalanx frame
  PerfectHandOPhalanx pho;
  pho.setFinger(p->finger);
  computeHandFrameMatrix(&pho, hMhf);

  //and then, rotate hMhf so that Z is pointing where the finger is pointing to
  //and translate the half of the fingertip lenght
  float flenght;
  if (p->finger != 4)
  {
    flenght =
        dynamic_cast<BBoxModel*>(object->linkedTo[p->finger]->next->linkedTo[0]->next->linkedTo[0]->next->model->geometry)->dimensions[1];
  }
  else
  {
    flenght =
        dynamic_cast<BBoxModel*>(object->linkedTo[p->finger]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->model->geometry)->dimensions[1];
  }
  vpHomogeneousMatrix rotx(0, flenght / 2, 0, -M_PI_2, 0, 0);
  hMhf = hMhf * rotx;

  return 1;
}

int PerfectHand::computeHandFrameMatrix(PerfectHandFingertipToThumb *p, vpHomogeneousMatrix &hMhf)
{
// 	cerr << "computing hand frame fingertip to thumb" << endl;
  //get fingertip position f w.r.t hand h
  static vpHomogeneousMatrix hMf;
  if (p->finger != 4)
    hMf = object->linkedTo[p->finger]->getNextObjectPose()
        * object->linkedTo[p->finger]->next->linkedTo[0]->getNextObjectPose()
        * object->linkedTo[p->finger]->next->linkedTo[0]->next->linkedTo[0]->getNextObjectPose();
  else
    hMf = object->linkedTo[p->finger]->getNextObjectPose()
        * object->linkedTo[p->finger]->next->linkedTo[0]->getNextObjectPose()
        * object->linkedTo[p->finger]->next->linkedTo[0]->next->linkedTo[0]->getNextObjectPose()
        * object->linkedTo[p->finger]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->getNextObjectPose();

  /*	
   wMf=object->linkedTo[p->finger]->next->linkedTo[0]->next->linkedTo[0]->next->getPosition();
   hMf=object->getPosition().inverse()*wMf;*/

// 	cerr << "wMf: " << endl << wMf << endl;
// 	cerr << "hmf: " << endl << hMf << endl;
  //get thumb position t w.r.t hand h
  static vpHomogeneousMatrix hMt;
  hMt = object->linkedTo[4]->getNextObjectPose() * object->linkedTo[4]->next->linkedTo[0]->getNextObjectPose()
      * object->linkedTo[4]->next->linkedTo[0]->next->linkedTo[0]->getNextObjectPose()
      * object->linkedTo[4]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->getNextObjectPose();
// 	hMt=object->getPosition().inverse()*wMt;
// 	cerr << "hmt: " << endl << hMt << endl;

  //get unitary vector t->f. This will be X axis of hf
  static vpColVector n(3);
  n[0] = -(hMt[0][3] - hMf[0][3]);
  n[1] = -(hMt[1][3] - hMf[1][3]);
  n[2] = -(hMt[2][3] - hMf[2][3]);
  n.normalize();

  //get perpendicular vector in YZ plane. This will be Z axis of hf
  static vpColVector a(3);
  a[0] = n[0];
  a[1] = -n[2];
  a[2] = n[1];

  //get perpendicular vector to the last two. This will be Y axis of hf
  static vpColVector o(3);
  o = vpColVector::cross(a, n);

  hMhf.setIdentity();
  for (int i = 0; i < 3; i++)
  {
    hMhf[i][0] = n[i];
    hMhf[i][1] = o[i];
    hMhf[i][2] = a[i];
  }
  hMhf[0][3] = (hMf[0][3] + hMt[0][3]) / 2;
  hMhf[1][3] = (hMf[1][3] + hMt[1][3]) / 2;
  hMhf[2][3] = (hMf[2][3] + hMt[2][3]) / 2;
// 	cerr << "hMhf updated" << endl;

  return 1;
}

int PerfectHand::computeHandFrameMatrix(PerfectHandPalm *p, vpHomogeneousMatrix &hMhf)
{
  PerfectHandFingertipToThumb hf;
  computeHandFrameMatrix(&hf, hMhf);

  hMhf = hMhf * vpHomogeneousMatrix(0, 0, -0.02, 0, 0, 0);

  return 1;
}

int PerfectHand::computeHandFrameMatrix(PerfectHandOPhalanx *p, vpHomogeneousMatrix &hMhf)
{
#ifdef _USE_ODE_
  //We must read fingertip pose from the corresponding ODE geom
  //This geom gives fingertip pose in world coordinates
  //However, we want it in hand origin coordinates
  //Thus, we get hand pose, fingertip pose, and compute the relative pose.
  vpHomogeneousMatrix wMf;
  if (p->finger!=4)
  {
    wMf=object->linkedTo[p->finger]->next->linkedTo[0]->next->linkedTo[0]->next->model->dynamics->getPose();
  }
  else
  {
    wMf=object->linkedTo[p->finger]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->next->model->dynamics->getPose();
  }
  vpHomogeneousMatrix wMh=object->model->dynamics->getPose();
  hMhf=wMh.inverse()*wMf;
#else
  //We must compute fingertip pose from joint angles.
  if (p->finger != 4)
    hMhf = object->linkedTo[p->finger]->getNextObjectPose()
        * object->linkedTo[p->finger]->next->linkedTo[0]->getNextObjectPose()
        * object->linkedTo[p->finger]->next->linkedTo[0]->next->linkedTo[0]->getNextObjectPose();
  else
    hMhf = object->linkedTo[p->finger]->getNextObjectPose()
        * object->linkedTo[p->finger]->next->linkedTo[0]->getNextObjectPose()
        * object->linkedTo[p->finger]->next->linkedTo[0]->next->linkedTo[0]->getNextObjectPose()
        * object->linkedTo[p->finger]->next->linkedTo[0]->next->linkedTo[0]->next->linkedTo[0]->getNextObjectPose();
#endif
  return 1;
}

int PerfectHand::computeHandFrameMatrix(PerfectHandMPhalanx *p, vpHomogeneousMatrix &hMhf)
{
#ifdef _USE_ODE_
  //We must read outer phalanx pose from the corresponding ODE geom
  //This geom gives fingertip pose in world coordinates
  //However, we want it in hand origin coordinates
  //Thus, we get hand pose, phalanx pose, and compute the relative pose.
  vpHomogeneousMatrix wMp;
  if (p->finger!=4)
  {
    wMp=object->linkedTo[p->finger]->next->linkedTo[0]->next->model->dynamics->getPose();
  }
  else
  {
    wMp=object->linkedTo[p->finger]->next->linkedTo[0]->next->linkedTo[0]->next->model->dynamics->getPose();
  }
  vpHomogeneousMatrix wMh=object->model->dynamics->getPose();
  hMhf=wMh.inverse()*wMp;
#else
  //We must compute fingertip pose from joint angles.
  if (p->finger != 4)
    hMhf = object->linkedTo[p->finger]->getNextObjectPose();
  else
    hMhf = object->linkedTo[p->finger]->getNextObjectPose()
        * object->linkedTo[p->finger]->next->linkedTo[0]->getNextObjectPose();
#endif
  return 1;
}

PerfectHand::~PerfectHand()
{
  //TODO: destroy the object
}

