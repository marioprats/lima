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

#include <lima/PerfectHandTOG.h>
#include <lima/ObjectClass.h>
#include <lima/ObjectAction.h>
#include <lima/Joint.h>
#include <lima/RJoint.h>
#include <lima/TJoint.h>
#include <lima/BBoxModel.h>
#include <lima/PerfectHandOneFinger.h>
#include <lima/PerfectHandCylindrical.h>
#include <lima/PerfectHandHook.h>
#include <lima/PerfectHandFingertip.h>
#include <lima/PerfectHandOPhalanx.h>
#include <lima/PerfectHandMPhalanx.h>
#include <lima/PerfectHandFingertipToThumb.h>
#include <lima/PerfectHandPalm.h>
#include <lima/PerfectHandLateral.h>
#include <lima/TOGPrimitive.h>

#include <lima/FixedHandlePush.h>

#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>

PerfectHandTOG::PerfectHandTOG() :
    TOG()
{
}

/** Checks whether object 'o' is supported for planning
 */
bool checkIfSupported(Object *o)
{
  return !(o->model == NULL || o->model->geometry == NULL || o->model->geometry->getType() != "bbox");
}

void PerfectHandTOG::computeTOG(TOGPrimitive **togp, ObjectFrame **of, TaskFrame **tf)
{
  *togp = NULL;
  *of = NULL;
  //TODO: Go back in the object hierarchy and chech that all object models are present and have bbox type

  //Depending on the object class, call to the specific planner
  if (task->actions.size() > 0)
  {
    //TODO: Plan the grasp for several consecutive actions, not for only one.
    ObjectAction *oa = task->actions[0];
    if (oa->getClassName() == "button")
    {
      if (oa->getActionName() == "push")
      {
        computeButtonPushTOG(oa, togp, of, tf);
      }
    }
    else if (oa->getClassName() == "roller")
    {
      if (oa->getActionName() == "turn")
      {
        computeRollerTurnTOG(oa, togp, of, tf);
      }
    }
    else if (oa->getClassName() == "fixedhandle")
    {
      if (oa->getActionName() == "push")
      {
        computeFixedHandlePushTOG(oa, togp, of, tf);
      }
    }
  }

  if (*togp != NULL && *of != NULL)
  {
    cerr << "Preshape: " << (*togp)->preshape->getPreshapeName() << endl;
    cerr << "Hand Frame: " << (*togp)->frame->getFrameName() << endl;
    cerr << "Object frame: " << endl << (*of)->oMof[0];
    cerr << "Task frame: " << endl << (*tf)->oMt;
  }
}

/** If object 'o' is the child of another object (the container), this function
 *    finds the unitary vector, in 'o' reference frame, which matches with the normal 
 *    of the container's face where 'o' is placed.
 */
vpColVector getFacingVector(Object *o)
{
  //get the container's object 'c'
  Object *c = o->linkedBy->previous;

  //get the position of 'o' w.r.t 'c'
  vpHomogeneousMatrix cMo = o->linkedBy->getNextObjectPose();

  //find the face 'f' of 'c' where 'o' is placed
  //get the normal of face 'f' in 'c' reference frame
  vpColVector f(3);
  for (int i = 0; i < 3; i++)
  {
    if (cMo[i][3] > dynamic_cast<BBoxModel*>(c->model->geometry)->dimensions[i] / 2)
    {
      cerr << "Position " << i << " is " << cMo[i][3] << ", bigger than dimension "
          << dynamic_cast<BBoxModel*>(c->model->geometry)->dimensions[i] / 2 << endl;
      f[i] = 1;
    }
    else if (cMo[i][3] < -dynamic_cast<BBoxModel*>(c->model->geometry)->dimensions[i] / 2)
    {
      f[i] = -1;
      cerr << "Position " << i << " is " << cMo[i][3] << ", smaller than dimension "
          << dynamic_cast<BBoxModel*>(c->model->geometry)->dimensions[i] / 2 << endl;
    }
    else
      f[i] = 0;
  }
  cerr << "fp is: " << f.t() << endl;

  //rotate the normal vector 'f' to the frame of 'o'
  vpHomogeneousMatrix cRo;
  cRo = cMo;
  cRo[0][3] = cRo[1][3] = cRo[2][3] = 0;

  vpColVector fv4 = cRo.inverse() * f;
  vpColVector fv3(3);
  fv3[0] = fv4[0];
  fv3[1] = fv4[1];
  fv3[2] = fv4[2];

  cerr << "fn is: " << fv3.t() << endl;
  return fv3;
}

void PerfectHandTOG::computeButtonPushTOG(ObjectAction *oa, TOGPrimitive **togp, ObjectFrame **of, TaskFrame **tf)
{
  cerr << "Computing Button Push plan on object " << oa->object->getName() << " for PerfectHand" << endl;

  //Compute the facing vector
  vpColVector f = getFacingVector(oa->object);

  //Set preshape to one finger precission
  *togp = new TOGPrimitive(new PerfectHandOneFinger(), new PerfectHandFingertip());

  //Set object frame to face which normal matches the container's normal.
  //Z direction of the object frame must go in opposite direction to the 
  //facing vector. Object frame Y axis is aligned with container's Y axis
  //when possible. However, for this object frame, rotation in Z is not defined.
  vpHomogeneousMatrix oMof;
  vpColVector n(3), o(3), a(3);
  a = -f;
  if (fabs(f[2]) == 1 || fabs(f[0]) == 1)
  {
    o = 0;
    o[1] = 1;
  }
  else
  {
    o = 0;
    o[2] = -f[1];
  }
  n = vpColVector::cross(o, a);

  oMof.setIdentity();
  for (int i = 0; i < 3; i++)
  {
    oMof[i][0] = n[i];
    oMof[i][1] = o[i];
    oMof[i][2] = a[i];
  }
  vpColVector t(3);
  t[0] = (f[0] * dynamic_cast<BBoxModel*>(oa->object->model->geometry)->dimensions[0]) / 2;
  t[1] = (f[1] * dynamic_cast<BBoxModel*>(oa->object->model->geometry)->dimensions[1]) / 2;
  t[2] = (f[2] * dynamic_cast<BBoxModel*>(oa->object->model->geometry)->dimensions[2]) / 2;
  oMof[0][3] = t[0];
  oMof[1][3] = t[1];
  oMof[2][3] = t[2];
  *of = new ObjectFrame(oa->object, oMof);

  vpColVector uv(6);
  uv = 0;
  uv[2] = 1;
  *tf = new TaskFrame(oa->object, oMof, uv);
}

void PerfectHandTOG::computeRollerTurnTOG(ObjectAction *oa, TOGPrimitive **togp, ObjectFrame **of, TaskFrame **tf)
{
  cerr << "Computing Roller Turn plan on object " << oa->object->getName() << " for PerfectHand" << endl;

  //Compute the facing vector
  vpColVector f = getFacingVector(oa->object);

  //Set preshape to cylindrical precission
  *togp = new TOGPrimitive(new PerfectHandLateral(), new PerfectHandOPhalanx());
  ((PerfectHandLateral*)(*togp)->preshape)->setClosing(60);
  ((PerfectHandOPhalanx*)(*togp)->frame)->setFinger(4);
  //Set object frame to face which normal matches the container's normal.
  //Y direction of the object frame must go in opposite direction to the 
  //facing vector. Object frame X axis is aligned with container's Y axis
  //when possible. However, for this object frame, rotation in Z is not defined.
  vpHomogeneousMatrix oMof;
  vpColVector n(3), o(3), a(3);
  o = -f;
  if (fabs(f[2]) == 1 || fabs(f[0]) == 1)
  {
    n = 0;
    n[1] = -1;
  }
  else
  {
    n = 0;
    n[2] = -f[1];
  }
  a = vpColVector::cross(n, o);

  oMof.setIdentity();
  for (int i = 0; i < 3; i++)
  {
    oMof[i][0] = n[i];
    oMof[i][1] = o[i];
    oMof[i][2] = a[i];
  }
  *of = new ObjectFrame(oa->object, oMof);

  vpColVector uv(6);
  uv = 0;
  uv[3] = f[0];
  uv[4] = f[1];
  uv[5] = f[2];
  //TODO: chech turning direction and multiply uv accordingly
  *tf = new TaskFrame(oa->object, vpHomogeneousMatrix(0, 0, 0, 0, 0, 0), uv);
}

void PerfectHandTOG::computeFixedHandlePushTOG(ObjectAction *oa, TOGPrimitive **togp, ObjectFrame **of, TaskFrame **tf)
{
  cerr << "Computing FixedHandle Push plan on object " << oa->object->getName() << " for PerfectHand" << endl;

  //Compute the facing vector
  vpColVector f = getFacingVector(oa->object);
  cerr << "Facing vector: " << f.t() << endl;

  //Compute the gap between the handle and the container object
  vpHomogeneousMatrix nMp = oa->object->linkedBy->getNextObjectPose().inverse();
  vpTranslationVector nTp;
  nMp.extract(nTp);
  vpRotationMatrix R;
  nMp.extract(R);
  vpMatrix nRp(3, 3);
  nRp = R;
  vpColVector dp = nRp * dynamic_cast<BBoxModel*>(oa->object->linkedBy->previous->model->geometry)->dimensions;
  dp[0] = fabs(dp[0]);
  dp[1] = fabs(dp[1]);
  dp[2] = fabs(dp[2]);
  vpColVector dn = dynamic_cast<BBoxModel*>(oa->object->model->geometry)->dimensions;
  vpColVector dir(3);
  dir[0] = fabs(f[0]);
  dir[1] = fabs(f[1]);
  dir[2] = fabs(f[2]);
  float gap = ((vpColVector)nTp * dir) + (dp * f) / 2 + (dn * f) / 2;
  for (int i = 0; i < 3; i++)
    if (fabs(f[i]) > 0.1)
      gap = -gap * f[i];
  cerr << "Gap is: " << gap << endl;

  //Dimensions of the object in directions perpendicular to facing vector
  //Compute the longest perpendicular dimension
  //The pinch will be along that dimension.
  vpColVector fperpdim(3);
  float max = 0;
  int maxindex = 0;
  for (int i = 0; i < 3; i++)
  {
    fperpdim[i] = (1 - f[i]) * dynamic_cast<BBoxModel*>(oa->object->model->geometry)->dimensions[i];
    if (fperpdim[i] > max)
    {
      max = fperpdim[i];
      maxindex = i;
    }
  }
  cerr << "Perpendicular dimensions are: " << fperpdim.t() << endl;

  vpHomogeneousMatrix oMof_1, oMof_2;
  vector<vpHomogeneousMatrix> v_oMof;
  //TODO: Check if space for grasp and pushing direction to decide if hook power, hook precission or cyclindrical
  if (gap > 0.025)
  {
    //there is gap: make hook grasp
    *togp = new TOGPrimitive(new PerfectHandHook(), new PerfectHandMPhalanx());
    ((PerfectHandHook*)(*togp)->preshape)->setClosing(40);

    //Y direction of the object frame must go in opposite direction to the 
    //facing vector. Object frame X axis is aligned with minima inertia axis
    //Two possibilites: X aligned with positive and negative minima intertia axis
    vpColVector n(3), o(3), a(3);
    n = 0;
    n[maxindex] = -1;
    o = -f;
    a = vpColVector::cross(n, o);
    oMof_1.setIdentity();
    for (int i = 0; i < 3; i++)
    {
      oMof_1[i][0] = n[i];
      oMof_1[i][1] = o[i];
      oMof_1[i][2] = a[i];
    }
    n = 0;
    n[maxindex] = 1;
    o = -f;
    a = vpColVector::cross(n, o);
    oMof_2.setIdentity();
    for (int i = 0; i < 3; i++)
    {
      oMof_2[i][0] = n[i];
      oMof_2[i][1] = o[i];
      oMof_2[i][2] = a[i];
    }
  }
  else
  {
    //if no space: cylindrical precission
    *togp = new TOGPrimitive(new PerfectHandCylindrical(), new PerfectHandFingertipToThumb());
    ((PerfectHandCylindrical*)(*togp)->preshape)->setClosing(40);

    //Z direction of the object frame must go in opposite direction to the 
    //facing vector. Object frame X axis is aligned with minima inertia axis
    //Two possibilites: X aligned with positive and negative minima intertia axis
    vpColVector n(3), o(3), a(3);
    o = 0;
    o[maxindex] = -1;
    a = -f;
    n = vpColVector::cross(o, a);
    oMof_1.setIdentity();
    for (int i = 0; i < 3; i++)
    {
      oMof_1[i][0] = n[i];
      oMof_1[i][1] = o[i];
      oMof_1[i][2] = a[i];
    }
    o = 0;
    o[maxindex] = 1;
    a = -f;
    n = vpColVector::cross(o, a);
    oMof_2.setIdentity();
    for (int i = 0; i < 3; i++)
    {
      oMof_2[i][0] = n[i];
      oMof_2[i][1] = o[i];
      oMof_2[i][2] = a[i];
    }
  }
  v_oMof.push_back(oMof_1);
  v_oMof.push_back(oMof_2);
  *of = new ObjectFrame(oa->object, v_oMof);

  if (((FixedHandlePush*)oa)->direction == FixedHandlePush::BACKWARDS)
  {
    //backwards motion is negative Z axis
    vpColVector uvz(6);
    uvz = 0;
    uvz[2] = -1;
    *tf = new TaskFrame(oa->object, oMof_1, uvz);
  }
  else
  {
    //lateral motion is negative X axis
    vpColVector uvx(6);
    uvx = 0;
    uvx[0] = -1;
    *tf = new TaskFrame(oa->object, oMof_1, uvx);
  }
}

PerfectHandTOG::~PerfectHandTOG()
{
}

