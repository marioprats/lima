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

#ifndef SIMULATEDPA10_H
#define SIMULATEDPA10_H

#include <lima/limaconfig.h>
#include <lima/SimulatedArm.h>
#include <lima/SimulatedHand.h>
#include <string>
using namespace std;

/**
 A simulated PA10 arm

 @author Mario Prats <mprats@icc.uji.es>
 */
class SimulatedPA10 : public SimulatedArm
{
public:
  /** The constructor */
  SimulatedPA10(vpColVector qini);

  /** Gets the name of the arm */
  string getName()
  {
    return string("SimulatedPA10");
  }

  /** Gets the current end-effector pose w.r.t robot kinematic base (units in meters) */
  vpHomogeneousMatrix getPosition();
  /** Gets the current joint values */
  vpColVector getJointValues();
  /** Check joint limits. Returns true if a limit has been reached.
   In that case, the vector inlimit is filled with the indexes of the joints which are in limit
   */
  bool checkJointLimits(vector<int> &inlimit);

  /** Sets the desired joint positions */
  void setJointValues(vpColVector dq);
  /** Sets the desired joint velocities */
  void setJointVelocity(vpColVector qdot);
  /** Sets the desired cartesian velocities. Units in meters/s and radians/s */
  void setCartesianVelocity(vpColVector qdot);

  ~SimulatedPA10();

protected:
  /** Creates or loads the arm object including geometrical (and possibly dynamical) model */
  Object *createObject();
public:
  /** Computes the forward kinematics of the arm */
  vpHomogeneousMatrix directKinematics(vpColVector q);
  /** Computes the Jacobian matrix of the end-effector w.r.t the robot base frame */
  vpMatrix jacobian07(vpColVector q);
//     vpMatrix newJacobian(vpColVector q);

  /** Computes the Jacobian matrix of the end-effector w.r.t the end-effector frame */
  vpMatrix jacobian77(vpColVector q);
  /** Computes the inverse of the jacobian matrix in the end-effector frame */
  vpMatrix jacobian77Inverse(vpColVector q);
  /** Computes the DH transformation matrix corresponding to the given parameters */
  void DHTransformationMatrix(double theta, double alpha, double a, double d, vpHomogeneousMatrix &result);
private:
  vpColVector qmax, qmin, qrange, qamax, qamin; ///< Allowed joint range
  vpColVector qini; ///< Initial joint configuration
};

#endif
