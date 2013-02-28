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

#ifndef BBOXMODEL_H
#define BBOXMODEL_H

#include <lima/limaconfig.h>
#include <lima/GeomModel.h>
#include <visp/vpColVector.h>
#include <visp/vpRGBa.h>

/**
 A bounding box model. The reference frame is in the center of mass.
 @author Mario Prats <mprats@icc.uji.es>
 */
class BBoxModel : virtual public GeomModel
{
public:
  vpColVector dimensions; ///< Dimensions of the bounding box (X,Y,Z, units in mm)
  vpRGBa color; ///< Color of the box

  /** Empty constructor */
  BBoxModel() :
      GeomModel()
  {
    dimensions.resize(3);
  }
  /** Constructor from a column vector with dimensions (units in mm), and color*/
  BBoxModel(vpColVector &dimensions, vpRGBa &color) :
      GeomModel()
  {
    dimensions.resize(3);
    setDimensions(dimensions);
    setColor(color);
  }
  /** Constructor from the dimensions in X, Y and Z (units in mm), and color R,G,B,A (range from 0 to 255) */
  BBoxModel(float dx, float dy, float dz, unsigned char r, unsigned char g, unsigned char b, unsigned char a) :
      GeomModel()
  {
    dimensions.resize(3);
    setDimensions(dx, dy, dz);
    setColor(r, g, b, a);
  }
  /** Constructor from the dimensions in X, Y and Z (units in mm), and color */
  BBoxModel(float dx, float dy, float dz, vpRGBa &color) :
      GeomModel()
  {
    dimensions.resize(3);
    setDimensions(dx, dy, dz);
    setColor(color);
  }

  /** Gets the type of geometric model (returns "bbox")*/
  virtual string getType()
  {
    return "bbox";
  }
  /** Sets the dimensions of the bounding box (units in mm) */
  void setDimensions(vpColVector &dimensions)
  {
    this->dimensions = dimensions;
  }
  /** Sets the dimensions of the bounding box (units in mm) */
  void setDimensions(float dx, float dy, float dz)
  {
    vpColVector d(3);
    d[0] = dx;
    d[1] = dy;
    d[2] = dz;
    setDimensions(d);
  }
  /** Sets the color of the bounding box */
  void setColor(vpRGBa &color)
  {
    this->color = color;
  }
  /** Sets the color of the bounding box with R,G,B,A coefficients (range from 0 to 255) */
  void setColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
  {
    vpRGBa c(r, g, b, a);
    setColor(c);
  }

#ifdef _USE_GL_
  virtual void GLRender();
#endif

  ~BBoxModel()
  {
  }

};

#endif
