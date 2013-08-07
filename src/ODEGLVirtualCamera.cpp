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

#include <lima/ODEGLVirtualCamera.h>
#include <lima/Joint.h>
#include <stdlib.h>

#include <iostream>
using namespace std;

#ifdef _USE_GL_
#include <GL/glut.h>

ODEGLVirtualCamera::ODEGLVirtualCamera(ODEGLScene *scene, int width, int height)
: VirtualCamera(scene,width,height)
{
  /* Allocate the image buffer */
  image = (unsigned char *)malloc( width * height * 4 );

  /* Bind the buffer to the context and make it current */
  OSMesaMakeCurrent( ((ODEGLScene*)scene)->GLWorld, image, GL_UNSIGNED_BYTE, width, height );

  setMaxVisibleDistance(5);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(tx,ty,-tz,cx,cy,-cz,0,-0.1,0);
}

/** Sets the maximum visible distance for the OpenGL camera (clipping plane)
 Objects farer that 'max' will not be rendered
 */
void ODEGLVirtualCamera::setMaxVisibleDistance(float max)
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(30.0, 1.0, 0.4, max);
  glMatrixMode(GL_MODELVIEW);
}

void ODEGLVirtualCamera::setCalibrationMatrix(float fx, float fy, int w, int h, float cx, float cy)
{
  float proj[16];
  //get current projection matrix
  glGetFloatv(GL_PROJECTION_MATRIX , proj);

  // How to obtain opengl projection matrix from camera calibration parameters:
  // 2.0*fx/w    2.0*k/w    1-2*x0/w       0
  // 0          2.0*fy/h   1-2*y0/h       0
  // 0           0          (f+n)/(n-f)    2*fn/(n-f)
  // 0           0         -1              0

  proj[0]=2.0*fx/w;
  proj[5]=2.0*fy/h;
  proj[2]=1-2*cx/w;
  proj[6]=1-2*cy/h;
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMultMatrixf(proj);
  glMatrixMode(GL_MODELVIEW);
}

void ODEGLVirtualCamera::setModelViewMatrix(vpHomogeneousMatrix M)
{
  //In OpenGL, XxY=-Z. In our convention XxY=Z. We have to inverse Z axis:
  vpHomogeneousMatrix M_gl=M;
  M_gl[2][0]=-M[2][0];
  M_gl[2][1]=-M[2][1];
  M_gl[2][2]=-M[2][2];
  M_gl[2][3]=-M[2][3];

  GLfloat wMo[16];
  for (int c=0; c<4; c++)
  for (int r=0; r<4; r++)
  {
    wMo[4*c+r]=M_gl[r][c];
  }
  glMultMatrixf(wMo);
}

void renderObject(Object *o)
{
  if (o!=NULL)
  {
    glPushMatrix();
    ODEGLVirtualCamera::setModelViewMatrix(o->wMo);

    /*	cerr<<"Object:"<<o<<endl;
     cerr<<"O.geom:"<< o->model->geometry<<endl;*/
    o->model->geometry->GLRender(); //!original

    glPopMatrix();

    for (unsigned int i=0; i<o->linkedTo.size(); i++)
    {
      renderObject(o->linkedTo[i]->next);
    }
  }
}

void ODEGLVirtualCamera::renderScene()
{

  OSMesaMakeCurrent( ((ODEGLScene*)scene)->GLWorld, image, GL_UNSIGNED_BYTE, width, height );
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(tx,ty,-tz,cx,cy,-cz,0,-0.1,0);
  scene->configure();

  for (int i=0; i<scene->objects.size(); i++)
  {
    glPushMatrix();
    renderObject(scene->objects[i]);
    glPopMatrix();
  }

}

void ODEGLVirtualCamera::acquire(vpImage<unsigned char> &I)
{
  int index=0;
  for (int i=0; i<height; i++)
  {
    for (int j=0; j<width; j++)
    {
      I[i][j]=(image[index]+image[index+1]+image[index+2])/3;
      index+=4;
    }
  }
}

void ODEGLVirtualCamera::acquire(vpImage<vpRGBa> &I)
{
  int index=0;
  for (int i=0; i<height; i++)
  {
    for (int j=0; j<width; j++)
    {
      I[i][j]=vpRGBa(image[index],image[index+1],image[index+2], image[index+3]);
      index+=4;
    }
  }
}

ODEGLVirtualCamera::~ODEGLVirtualCamera()
{
  free(image);
}
#endif

