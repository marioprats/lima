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

#include <lima/RawModel.h>
#ifdef _USE_GL_
#include <GL/glut.h>
#endif

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string.h>

using namespace std;

RawModel::RawModel(string file, vpRGBa color, inputType input) :
    GeomModel()
{
  this->input = input;
  float factor = 1.0;
  if (input == INPUT_MILLIMETERS)
    factor = 1000.0;
  else if (input == INPUT_METERS)
    factor = 1.0;

  std::ifstream s(file.c_str(), ifstream::in);
  if (s.good())
  {
    char name[256];
    while (!s.eof())
    {
// 			cerr << "next line" << endl;
      s.getline(name, 256);
      //process line
      char *token = strtok(name, " ");
      while (token != NULL)
      {
        vertex.push_back(atof(token) / factor);
// 				cerr << "token: " << token << endl;	
        token = strtok(NULL, " ");
      }
// 			cerr << "vertex: " << vertex.size() << endl;
    }
  }
  else
  {
    cerr << "RawModel ERROR: Cannot open file " << file << endl;
    exit(0);
  }
  setColor(color);
}

#ifdef _USE_GL_

/** Renders the object defined in the RAW file using OpenGL.
 */
void RawModel::GLRender()
{
  glPushMatrix();
  glColor4f(color.R/255.0,color.G/255.0,color.B/255.0,color.A/255.0);
// 	glScalef(dimensions[0],dimensions[1],dimensions[2]);
  glBegin(GL_TRIANGLES);
  for (int i=0; i<vertex.size(); i+=9)
  {
    vpColVector v1(3);
    v1[0]=vertex[i+3]-vertex[i];
    v1[1]=vertex[i+4]-vertex[i+1];
    v1[2]=vertex[i+5]-vertex[i+2];
    vpColVector v2(3);
    v2[0]=vertex[i+6]-vertex[i];
    v2[1]=vertex[i+7]-vertex[i+1];
    v2[2]=vertex[i+8]-vertex[i+2];
    vpColVector normal(3);
    normal=vpColVector::cross(v1,v2);
    glNormal3f(normal[0],normal[1],normal[2]);

    glVertex3f(vertex[i], vertex[i+1], vertex[i+2]);
    glVertex3f(vertex[i+3], vertex[i+4], vertex[i+5]);
    glVertex3f(vertex[i+6], vertex[i+7], vertex[i+8]);
  }
  glEnd();
  glPopMatrix();
}
#endif
