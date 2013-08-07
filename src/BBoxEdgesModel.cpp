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

#include <lima/BBoxEdgesModel.h>
#include <visp/vpRGBa.h>

void BBoxEdgesModel::computeEdges()
{
  vpColVector pfront(4), pback(4), pright(4), pleft(4), pup(4), pdown(4);
  vpPoint a;

  pfront = pback = pright = pleft = pup = pdown = 0;
  pfront[2] = 1;
  pfront[3] = -dimensions[2] / 2;
  pback[2] = -1;
  pback[3] = -dimensions[2] / 2;
  pright[0] = 1;
  pright[3] = -dimensions[0] / 2;
  pleft[0] = -1;
  pleft[3] = -dimensions[0] / 2;
  pup[1] = 1;
  pup[3] = -dimensions[1] / 2;
  pdown[1] = -1;
  pdown[3] = -dimensions[1] / 2;

  vector<vpPoint> vertex;
  a.setWorldCoordinates(-dimensions[0] / 2, -dimensions[1] / 2, dimensions[2] / 2);
  vertex.push_back(a); //v0

  a.setWorldCoordinates(dimensions[0] / 2, -dimensions[1] / 2, dimensions[2] / 2);
  vertex.push_back(a); //v1

  a.setWorldCoordinates(dimensions[0] / 2, dimensions[1] / 2, dimensions[2] / 2);
  vertex.push_back(a); //v2

  a.setWorldCoordinates(-dimensions[0] / 2, dimensions[1] / 2, dimensions[2] / 2);
  vertex.push_back(a); //v3

  if (dimensions[2] > 0.02)
  {
    a.setWorldCoordinates(-dimensions[0] / 2, -dimensions[1] / 2, -dimensions[2] / 2);
    vertex.push_back(a); //v4

    a.setWorldCoordinates(dimensions[0] / 2, -dimensions[1] / 2, -dimensions[2] / 2);
    vertex.push_back(a); //v5

    a.setWorldCoordinates(dimensions[0] / 2, dimensions[1] / 2, -dimensions[2] / 2);
    vertex.push_back(a); //v6

    a.setWorldCoordinates(-dimensions[0] / 2, dimensions[1] / 2, -dimensions[2] / 2);
    vertex.push_back(a); //v7	
  }

  vector<vpPoint> V(4);
  V[0] = vertex[0];
  V[1] = vertex[1];
  V[2] = vertex[2];
  V[3] = vertex[3];
  addFace(V, color);

  //FIXME: This condition shouldn't be here
  if (dimensions[2] > 0.02)
  {
    V[0] = vertex[7];
    V[1] = vertex[6];
    V[2] = vertex[5];
    V[3] = vertex[4];
    addFace(V, color);

    V[0] = vertex[7];
    V[1] = vertex[4];
    V[2] = vertex[0];
    V[3] = vertex[3];
    addFace(V, color);

    V[0] = vertex[5];
    V[1] = vertex[6];
    V[2] = vertex[2];
    V[3] = vertex[1];
    addFace(V, color);

    V[0] = vertex[6];
    V[1] = vertex[7];
    V[2] = vertex[3];
    V[3] = vertex[2];
    addFace(V, color);

    V[0] = vertex[4];
    V[1] = vertex[5];
    V[2] = vertex[1];
    V[3] = vertex[0];
    addFace(V, color);
  }
}
