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

#ifndef SIMULATEDFORCESENSOR_H
#define SIMULATEDFORCESENSOR_H

#include <lima/limaconfig.h>
#include <lima/Object.h>
#include <lima/ForceSensor.h>

/**
 An ODE simulated force sensor

 @author Mario Prats <mprats@icc.uji.es>
 */
class SimulatedForceSensor : public ForceSensor
{
public:
  bool initialized; ///< Whether the sensor is initialized (can be used) or not. 
  Object *object; ///< The force sensor object, including model

  SimulatedForceSensor();

  /** Initializes the force sensor. @returns true if success. */
  bool init();

  /** Gets the force/torque vector */
  vpColVector getForce();

  ~SimulatedForceSensor();

protected:
  /** Creates the simulated force sensor object */
  void createForceSensorObject();

private:
  vpColVector history_i[2], history_o[2]; ///< For signal filtering 
#ifdef _USE_ODE_
              dJointFeedback jfeed; ///< For ODE joint feedback
#endif
};

#endif
