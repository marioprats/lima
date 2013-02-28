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

#include <lima/FixedHandlePush.h>
#include <lima/ObjectClass.h>

FixedHandlePush::FixedHandlePush(Object *object) :
    FixedHandleAction(object)
{
  if (object->getClass() == NULL || object->getClass()->getClass() != "fixedhandle")
  {
    cerr << "FixedHandlePush::FixedHandlePush ERROR: Cannot perform FixedHandlePush on a non-fixedhandle object"
        << endl;
    this->object = NULL;
  }

  //This action has six modifiers: push left, right, forwards, backwards, up and down
  action_params.push_back("left");
  action_params.push_back("right");
  action_params.push_back("forwards");
  action_params.push_back("backwards");
  action_params.push_back("up");
  action_params.push_back("down");
  direction = NOTSET;
}

void FixedHandlePush::setActionParam(string param)
{
  if (param == "left")
  {
    direction = LEFT;
  }
  else if (param == "right")
  {
    direction = RIGHT;
  }
  else if (param == "forwards")
  {
    direction = FORWARDS;
  }
  else if (param == "backwards")
  {
    direction = BACKWARDS;
  }
  else if (param == "up")
  {
    direction = UP;
  }
  else if (param == "down")
  {
    direction = DOWN;
  }
  else
  {
    direction = NOTSET;
  }
}

FixedHandlePush::~FixedHandlePush()
{
}

