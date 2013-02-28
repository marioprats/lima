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

#ifndef OBJECTFRAME_H
#define OBJECTFRAME_H

#include <lima/limaconfig.h>
#include <visp/vpHomogeneousMatrix.h>
#include <lima/Object.h>
#include <vector>
using namespace std;

/**
 The object frame is a frame, expressed in the coordinate system of an object, which is related to the grasping position and strategy. To perform the grasp, the hand frame must be aligned with the object frame. This class allows to define a set of object frames in an object, for a given task.
 The object frame which will be finally used must be chosen at execution, and depends on current
 hand position. Normally, it is desired to minimize hand rotation. In this case, the one which 
 is closer in rotation to hand frame would be chosen.

 @author Mario Prats <mprats@icc.uji.es>
 */
class ObjectFrame
{
public:
  Object *object; ///< The object which contains this frame
  vector<vpHomogeneousMatrix> oMof; ///< The possible object frames w.r.t 'object' origin frame

  ObjectFrame(Object *object, vpHomogeneousMatrix oMof);
  ObjectFrame(Object *object, vector<vpHomogeneousMatrix> oMof);

  /** Adds an object frame */
  void addObjectFrame(vpHomogeneousMatrix oMof);

  ~ObjectFrame();

};

#endif
