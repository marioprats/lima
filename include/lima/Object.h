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

#ifndef OBJECT_H
#define OBJECT_H

#include <lima/limaconfig.h>
#include <vector>
#include <visp/vpHomogeneousMatrix.h>
#include <iostream>
using namespace std;

#include <lima/ObjectModel.h>
#include <lima/Task.h>
#include <string>

using namespace std;

//forward declaration of class Joint and ObjectClass
class Joint;
class ObjectClass;

/**
 An object, described by:
 - Its name
 - The object model (may include dynamics)
 - The object class
 - The object position in the world frame
 - The joints which link the object to the subobjects (next in the object tree)
 - The joint which links the object to the parent (previous in the object tree)
 - The actions that can be performed on the object
 @author Mario Prats <mprats@icc.uji.es>
 */
class Object
{
public:
  string name; ///< The name of the object
  ObjectClass *oclass; ///< The object class (button, roller, etc.)
  vector<Joint*> linkedTo; ///< The joints that link this object to the subobjects
  Joint *linkedBy; ///< The joint that links this object with the previous one
  ObjectModel *model; ///< The model of the object (geometry and dynamics)
  vpHomogeneousMatrix wMo; ///< The pose of the object in the world frame
  vector<Task*> tasks; ///< The tasks that can be performed on the object

  /** Empty constructor */
  Object()
  {
    name = string("");
    oclass = NULL;
    linkedBy = NULL;
    model = NULL;
    wMo.setIdentity();
  }
  /** Constructor from name */
  Object(string name)
  {
    linkedBy = NULL;
    oclass = NULL;
    model = NULL;
    wMo.setIdentity();
    setName(name);
  }
  /** Constructor from name and model */
  Object(string name, ObjectModel *model)
  {
    linkedBy = NULL;
    oclass = NULL;
    wMo.setIdentity();
    setName(name);
    setModel(model);
  }

  /** Sets the name of the object */
  void setName(string name)
  {
    this->name = name;
  }
  /** Gets the name of the object */
  string getName()
  {
    return name;
  }
  /** Sets the class of the object */
  void setClass(ObjectClass *oclass)
  {
    this->oclass = oclass;
  }
  /** Gets the class of the object */
  ObjectClass *getClass()
  {
    return oclass;
  }
  /** Sets the model of the object */
  void setModel(ObjectModel *model)
  {
    this->model = model;
  }
  /** Sets the object position w.r.t world frame */
  void setPosition(vpHomogeneousMatrix wMo)
  {
    this->wMo = wMo;
  }
  /** Gets the object position w.r.t world frame */
  vpHomogeneousMatrix getPosition()
  {
    return wMo;
  }
  /** Links this object to the object 'next' via the joint 'j',
   located at pose 'cMj' in current object reference frame,
   and 'nNj' in the next object reference frame.
   */
  void linkTo(vpHomogeneousMatrix cMj, Joint *j, vpHomogeneousMatrix nMj, Object *next);

  /** Adds a task to the object */
  void addTask(Task *task);

  /** Returns all the tasks that can be performed on this object */
  vector<Task*> getTasks();

  /** Returns all the tasks that can be performed on this object */
  vector<string> getTasksInStrings();

  /** Returns the task with the given name */
  Task *getTask(string name);

  ~Object()
  {
  }

};

#endif
