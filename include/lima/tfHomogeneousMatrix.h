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

#ifndef TFHOMOGENEOUSMATRIX_H
#define TFHOMOGENEOUSMATRIX_H

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpQuaternionVector.h>
#include <visp/vpTranslationVector.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

/** Class that encapsulates a ROS tf transform into a visp homogeneous matrix
 */
class tfHomogeneousMatrix: public vpHomogeneousMatrix
{
public:

  tfHomogeneousMatrix(boost::shared_ptr<tf::TransformListener> listener) : vpHomogeneousMatrix(), listener_(listener) {}

  /** Constructor. Builds tMo being t the target frame and o the origin frame
   * @param origin the origin frame name
   * @param target the target frame name
   */
  tfHomogeneousMatrix(boost::shared_ptr<tf::TransformListener> listener, const std::string &target, const std::string &origin) :
    vpHomogeneousMatrix(),
    listener_(listener)
  {
    buildFrom(target, origin);
  }

  bool update(ros::Time time = ros::Time(0));

  void buildFrom(const std::string &target, const std::string &origin)
  {
    target_ = target;
    origin_ = origin;
    update();
  }

  const ros::Time &getStamp()
  {
    return stamp_;
  }

  ~tfHomogeneousMatrix() {}

private:
  boost::shared_ptr<tf::TransformListener> listener_;
  std::string target_, origin_;

  ros::Time stamp_;
};

#endif
