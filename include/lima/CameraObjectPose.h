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

#ifndef CAMERAOBJECTPOSE_H
#define CAMERAOBJECTPOSE_H

#include <lima/limaconfig.h>
#include <lima/Camera.h>
#include <lima/Object.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPose.h>
#include <visp/vpDisplay.h>
#include <visp/vpPoint.h>
#include <visp/vpColor.h>
#include <visp/vpPixelMeterConversion.h>

#include <sstream>

/** Class that represents a relative camera-object configuration, including:
 - The relative pose between camera and object
 - An image taken from the camera from this configuration.
 The template type can be 'vpRGBa' for color capture or 'unsigned char' for B&W capture
 */
template<typename T>
  class CameraObjectPose
  {
  public:
    Camera *camera; ///< Pointer to the camera that observes the object
    Object *object; ///< Pointer to the object that is being observed

    vpImage<T> view; ///< The camera view (camera image)
    vpHomogeneousMatrix cMo; ///< Relative pose between camera and object

    CameraObjectPose()
    {
    }
    CameraObjectPose(Camera *cam, Object *ob, vpHomogeneousMatrix cMo);
    CameraObjectPose(Camera *cam, Object *ob);

    void init(Camera *cam, Object *ob)
    {
      camera = cam;
      object = ob;
      cMo.setIdentity();
    }

    const vpHomogeneousMatrix &getObjectPose()
    {
      return cMo;
    }
    void setObjectPose(vpHomogeneousMatrix &cMo)
    {
      this->cMo = cMo;
    }

    /** Init the pose by clicking on the image the pixels corresponding to the coordinates in file
     */
    void initClick(const vpImage<T> &I, const std::string &file);

    void captureView()
    {
      camera->acquire(view);
    }
    vpImage<T> &getView()
    {
      return view;
    }

    ~CameraObjectPose()
    {
    }
  };

template<typename T>
  CameraObjectPose<T>::CameraObjectPose(Camera *cam, Object *ob, vpHomogeneousMatrix cMo)
  {
    init(cam, ob);
    setObjectPose(cMo);
  }

template<typename T>
  CameraObjectPose<T>::CameraObjectPose(Camera *cam, Object *ob)
  {
    init(cam, ob);
  }

// Adapted from ViSP
// http://www.irisa.fr/lagadic/visp/documentation/visp-2.7.0/a00299.html#a8ca5d48a823641b372792ad7913cb059
template<typename T>
void CameraObjectPose<T>::initClick(const vpImage<T> &I, const std::string &file)
{
  vpDisplay::display(I) ;
  vpDisplay::flush(I) ;

  vpPose pose ;

  pose.clearPoint() ;

  // file parser
  // number of points
  // X Y Z
  // X Y Z

  double X,Y,Z ;

  vpImagePoint ip;
  std::string ext = ".init";
  std::string str_pose = "";
  unsigned int pos = file.rfind(ext);

  std::stringstream ss;
  if( pos == file.size()-ext.size() && pos != 0)
    ss << file;
  else
    ss << file << ".init";

  std::cout << "filename " << ss.str() << std::endl;
  std::fstream finit ;
  finit.open(ss.str().c_str(), std::ios::in) ;
  if (finit.fail()){
    std::cout << "cannot read " << ss.str() << std::endl;
    throw vpException(vpException::ioError, "cannot read init file");
  }

  unsigned int n ;
  finit >> n ;
  std::cout << "number of points  " << n << std::endl ;
  vpPoint *P = new vpPoint [n]  ;
  for (unsigned int i=0 ; i < n ; i++){
    finit >> X ;
    finit >> Y ;
    finit >> Z ;
    P[i].setWorldCoordinates(X,Y,Z) ; // (X,Y,Z)
  }
  finit.close();

  for(unsigned int i=0 ; i< n ; i++)
  {
    std::cout << "Click on point " << i+1 << std::endl ;
    double x=0,y=0;
    vpDisplay::getClick(I, ip);
    vpDisplay::displayCross(I, ip, 5,vpColor::green);
    vpDisplay::flush(I);
    vpPixelMeterConversion::convertPoint(camera->cparams, ip, x, y);
    P[i].set_x(x);
    P[i].set_y(y);

    vpDisplay::displayPoint (I, ip, vpColor::green); //display target point
    pose.addPoint(P[i]) ; // and added to the pose computation point list
  }
  vpDisplay::flush(I) ;

  vpHomogeneousMatrix cMo1, cMo2;
  pose.computePose(vpPose::LAGRANGE, cMo1) ;
  double d1 = pose.computeResidual(cMo1);
  pose.computePose(vpPose::DEMENTHON, cMo2) ;
  double d2 = pose.computeResidual(cMo2);

  if(d1 < d2){
    cMo = cMo1;
  }
  else{
    cMo = cMo2;
  }
  pose.computePose(vpPose::VIRTUAL_VS, cMo);

  delete [] P;
}


#endif
