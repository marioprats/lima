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

#ifndef SCENE_H
#define SCENE_H

#include <lima/limaconfig.h>
#include <lima/Object.h>
#include <visp/vpHomogeneousMatrix.h>
#include <vector>

using namespace std;

/**
 A virtual world where objects can be created and dynamics can be simulated (if using the child class ODEGLScene). The robot can use this virtual world as an internal representation of the real world. It can even try actions on the virtual world and observing the results, before performing the tasks in the real world.

 @author Mario Prats <mprats@icc.uji.es>
 */
class Scene
{
public:
  vector<Object*> objects; ///< A vector of pointers to the objects in the scene

  Scene();

  /** Adds an object to the scene, at a position given by the homogeneous
   matrix wMo. TODO: check that 'o' is a root object */
  virtual void addObject(vpHomogeneousMatrix wMo, Object *o);

  /** Abstract virtual method for configuring the scene (lights, scenario, etc) */
  virtual void configure()=0;
  /** Abstract virtual method for updating the scene from one iteration to another */
  virtual void simulationStep()=0;

  ~Scene();

};

#endif
