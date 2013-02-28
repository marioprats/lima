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

#ifndef PERFECTHANDLATERAL_H
#define PERFECTHANDLATERAL_H

#include <lima/limaconfig.h>
#include <lima/PerfectHandPreshape.h>
#include <string>
using namespace std;

/**
 The lateral configuration for the PerfectHand

 @author Mario Prats <mprats@icc.uji.es>
 */
class PerfectHandLateral : public PerfectHandPreshape
{
public:
  float closing; ///< The closing percentage of the pinch

  PerfectHandLateral();

  /** Returns the name of the preshape. It returns "PerfectHandLateral" */
  string getPreshapeName()
  {
    return string("PerfectHandLateral");
  }

  /** Gets the closing parameter */
  float getClosing()
  {
    return closing;
  }

  /** Sets the closing percentage parameter */
  void setClosing(float closing)
  {
    if (closing > 100)
      closing = 100;
    else if (closing < 0)
      closing = 0;
    this->closing = closing;
  }

  ~PerfectHandLateral();

};

#endif
