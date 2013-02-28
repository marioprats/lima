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

#ifndef SIMULATEDARM_H
#define SIMULATEDARM_H

#include <lima/limaconfig.h>
#include <lima/Arm.h>
#include <lima/Object.h>
#include <lima/SimulatedForceSensor.h>
#include <lima/SimulatedHand.h>

/**
 A simulated arm

 @author Mario Prats <mprats@icc.uji.es>
 */
class SimulatedArm : public Arm
{
public:
  Object *object; ///< A pointer to the object, containing the arm model
  SimulatedForceSensor *force_sensor; ///< The force sensor attached to the wrist (NULL if any)
  SimulatedHand *hand; ///< The hand attached to the arm (NULL if any)
  vpHomogeneousMatrix mMk; ///< The robot kinematic base w.r.t the robot model base

  SimulatedArm();

  /** Gets the name of the arm */
  virtual string getName()=0;

  /** Gets the current end-effector pose w.r.t robot kinematic base (units in meters) */
  virtual vpHomogeneousMatrix getPosition()=0;
  /** Gets the current joint values */
  virtual vpColVector getJointValues()=0;
  /** Check joint limits. Returns true if a limit has been reached.
   In that case, the vector inlimit is filled with the indexes of the joints which are in limit
   */
  virtual bool checkJointLimits(vector<int> &inlimit)=0;

  /** Sets the desired joint positions */
  virtual void setJointValues(vpColVector dq)=0;
  /** Sets the desired joint velocities */
  virtual void setJointVelocity(vpColVector qdot)=0;
  /** Sets the desired cartesian velocities. Units in meters/s and radians/s */
  virtual void setCartesianVelocity(vpColVector xdot)=0;

  /** Attaches a force sensor 'fs' to the arm. The force sensor frame w.r.t the end-effector must be indicated as the 'eMf' homogeneous matrix */
  void addForceSensor(SimulatedForceSensor *fs, vpHomogeneousMatrix eMf);
  /** Attaches a hand 'h' to the arm. The hand frame w.r.t the end-effector must be indicated as the 'eMh' homogeneous matrix */
  void addHand(SimulatedHand *h, vpHomogeneousMatrix eMh);
  /** Whether the arm has a force sensor attached to the wrist */
  bool hasForceSensor()
  {
    return (force_sensor != NULL);
  }
  /** Whether the arm has a hand */
  bool hasHand()
  {
    return (hand != NULL);
  }

  /** Sets the relation between the robot model base and the robot kinematic base
   The robot model base is the frame of the first object in the arm model.
   The robot kinematic base is the frame used for the kinematic models
   */
  void setKinematicBase(vpHomogeneousMatrix mMk)
  {
    this->mMk = mMk;
  }
  /** Gets the kinematic base frame w.r.t the arm model base frame */
  vpHomogeneousMatrix getKinematicBase()
  {
    return mMk;
  }

  /** Sets the object attached to the simulated arm (which includes the model) */
  void setObject(Object *o)
  {
    this->object = o;
  }

  ~SimulatedArm();

protected:
  /** Creates or loads the arm object including geometrical (and possibly dynamical) model */
  virtual Object *createObject()=0;
};

#endif
