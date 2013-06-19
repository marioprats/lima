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

#ifndef VVSPOSEESTIMATOR_H
#define VVSPOSEESTIMATOR_H

#include <lima/limaconfig.h>
#include <lima/PoseEstimator.h>
#include <lima/CameraObjectPose.h>
#include <lima/EdgesModel.h>
#include <lima/Joint.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpDisplay.h>

#include <ros/ros.h>

template<typename T>
  class VVSPoseEstimator : public PoseEstimator
  {

  protected:
    CameraObjectPose<T> *camobject_;

    double lambda_;
    double error_; //The euclidean norm of the vvs velocity vector
    int ival_; //number of pixels to explore around a point
    int distance_; //distance (pixels) between sampled points
    int mingrad_; //Minimum gradient for considering the edge

    bool detect_outliers_;

    vpMatrix S_, kerS_, D_;

    double sigma_, med_deltai_;

    ConvolutionMatrix * CM_;

  public:
    VVSPoseEstimator(CameraObjectPose<T> *camobject);

    virtual vpHomogeneousMatrix computePose();
    virtual vpHomogeneousMatrix computePoseFromDescriptors(const std::vector<vpColVector> &descriptors);

    virtual double getError()
    {
      return error_;
    }

    /** Sets the gain of the VVS minimization */
    void setLambda(double l)
    {
      lambda_ = l;
    }

    /** Sets an interval (in pixels) where to perform a linear search around 
     the sampled point */
    void setIval(int ival)
    {
      this->ival_ = ival;
    }

    /** Sets the distance (in pixels) between two consecutive sampled points */
    void setDistance(int distance)
    {
      this->distance_ = distance;
    }

    /** Sets a gradient threshold. Features with gradient below the threshold will
     be discarded */
    void setMinGrad(int mingrad)
    {
      this->mingrad_ = mingrad;
    }

    //TODO: The articulation matrix should be computed automatically from the object	  
    void setS(vpMatrix S)
    {
      this->S_ = S;
      vpMatrix I(6, 6);
      I.setIdentity();
      kerS_ = I - S_;
    }

    void setDetectOutliers(bool flag)
    {
      detect_outliers_ = flag;
    }

    void project_model(Object *o, const vpHomogeneousMatrix &cMo);
    void drawModel(Object *o, const vpImage<T> &I);
    void drawSearchPixels(Object *o, const vpImage<T> &I);
    void drawMatchedPixels(Object *o, const vpImage<T> &I);

    virtual ~VVSPoseEstimator()
    {
    }


  protected:
    void project_edges(Object *o, const vpHomogeneousMatrix &cMo);
    void select_visible_edges(Object *o, const vpHomogeneousMatrix &cMo);
    void find_edges(Object *o);
    void find_edges_from_descriptors(Object *o, const std::vector<vpColVector> &descriptors);
    void discard_bad_confidence_points(Object *o, vpMatrix &D, int index = 0);

    vpMatrix &compute_s(Object *o);
    void stack_s_matrix(Object *o, vpMatrix &s);
    vpMatrix &compute_Ls(Object *o);
    void stack_Ls_matrix(Object *o, vpMatrix &Ls);
  };

template<typename T>
  VVSPoseEstimator<T>::VVSPoseEstimator(CameraObjectPose<T> *camobject)
  {
    this->camobject_ = camobject;

    S_.resize(6, 6);
    S_.setIdentity();
    setS(S_);
    setLambda(0.8);

    setDetectOutliers(true);
    setIval(5);
    setDistance(8);
    setMinGrad(4);

    //CM_= new ConvolutionMatrix(3,180);

    sigma_ = 0.001;
    med_deltai_ = 0.0;
  }

template<typename T>
vpHomogeneousMatrix VVSPoseEstimator<T>::computePose()
{
  //TODO: put inside a loop that repeats until error<tol or timeout
  vpMatrix Ls, s;

  //project the model in view v, find distance to real edges
  project_model(camobject_->object, camobject_->getObjectPose());

  //compute distance feature vector and interaction matrix
  Ls = compute_Ls(camobject_->object);
  s = compute_s(camobject_->object);

  if (detect_outliers_)
  {
    D_.resize(s.getRows(), s.getRows());

    //M-estimator
    double emed = 0; //median of the error
    for (std::size_t i = 0; i < s.getRows(); i++)
    {
      emed = emed + fabs(s[i][0]);
    }

    emed = emed / s.getRows();
    double deltai[s.getRows()];
    double aux_med_deltai = 0, medsigma = 0;
    for (std::size_t i = 0; i < s.getRows(); i++)
    {
      double ui, tukeyi = 0;

      //deltai
      deltai[i] = s[i][0] - emed;
      medsigma += fabs(deltai[i] - med_deltai_);
      aux_med_deltai += deltai[i];

      //ui
      ui = deltai[i] / sigma_;
      static const double C = 4.6851; //constant for the tukey's function
      if (fabs(ui) <= C)
        tukeyi = ui * pow(1 - pow(ui / C, 2), 2); //tukey's function
      if (ui != 0)
        D_[i][i] = tukeyi / ui;
      else
        D_[i][i] = 1;
    }

    med_deltai_ = aux_med_deltai / s.getRows();
    medsigma /= s.getRows();
    sigma_ = 1.48 * medsigma; //Median Absolute Deviation robust statistic (MAD)

    discard_bad_confidence_points(camobject_->object, D_);

    Ls =  D_ * Ls;
    s =  D_ * s;
  }

  // control law
  vpVelocityTwistMatrix V(camobject_->cMo);
  vpMatrix Vm = V;
  vpMatrix A = Vm * S_;
  vpMatrix vm = -lambda_ * A * (Ls * A).pseudoInverse() * s;

  //update cMo estimation
  vpHomogeneousMatrix cnMc(-vm[0][0], -vm[1][0], -vm[2][0], -vm[3][0], -vm[4][0], -vm[5][0]);
  camobject_->cMo = cnMc * camobject_->cMo;

  error_ = vm.euclideanNorm();

  return camobject_->cMo;
}

template<typename T>
vpHomogeneousMatrix VVSPoseEstimator<T>::computePoseFromDescriptors(const std::vector<vpColVector> &descriptors)
{
  //TODO: put inside a loop that repeats until error<tol or timeout
  vpMatrix Ls, s;

  //project the model in view v, find distance to real edges
  project_edges(camobject_->object, camobject_->getObjectPose());
  select_visible_edges(camobject_->object, camobject_->getObjectPose());
  find_edges_from_descriptors(camobject_->object, descriptors);

  //compute distance feature vector and interaction matrix
  Ls = compute_Ls(camobject_->object);
  s = compute_s(camobject_->object);

  if (detect_outliers_)
  {
    D_.resize(s.getRows(), s.getRows());

    //M-estimator
    double emed = 0; //median of the error
    for (std::size_t i = 0; i < s.getRows(); i++)
    {
      emed = emed + fabs(s[i][0]);
    }

    emed = emed / s.getRows();
    double deltai[s.getRows()];
    double aux_med_deltai = 0, medsigma = 0;
    for (std::size_t i = 0; i < s.getRows(); i++)
    {
      double ui, tukeyi = 0;

      //deltai
      deltai[i] = s[i][0] - emed;
      medsigma += fabs(deltai[i] - med_deltai_);
      aux_med_deltai += deltai[i];

      //ui
      ui = deltai[i] / sigma_;
      static const double C = 4.6851; //constant for the tukey's function
      if (fabs(ui) <= C)
        tukeyi = ui * pow(1 - pow(ui / C, 2), 2); //tukey's function
      if (ui != 0)
        D_[i][i] = tukeyi / ui;
      else
        D_[i][i] = 1;
    }

    med_deltai_ = aux_med_deltai / s.getRows();
    medsigma /= s.getRows();
    sigma_ = 1.48 * medsigma; //Median Absolute Deviation robust statistic (MAD)

    discard_bad_confidence_points(camobject_->object, D_);

    Ls =  D_ * Ls;
    s =  D_ * s;
  }

  // control law
  vpVelocityTwistMatrix V(camobject_->cMo);
  vpMatrix Vm = V;
  vpMatrix A = Vm * S_;
  vpMatrix vm = -lambda_ * A * (Ls * A).pseudoInverse() * s;

  //update cMo estimation
  vpHomogeneousMatrix cnMc(-vm[0][0], -vm[1][0], -vm[2][0], -vm[3][0], -vm[4][0], -vm[5][0]);
  camobject_->cMo = cnMc * camobject_->cMo;

  error_ = vm.euclideanNorm();

  return camobject_->cMo;
}

template<typename T>
void VVSPoseEstimator<T>::project_model(Object *o, const vpHomogeneousMatrix &cMo)
{
  project_edges(o, cMo);
  select_visible_edges(o, cMo);
  find_edges(o);
}

template<typename T>
  void VVSPoseEstimator<T>::project_edges(Object *o, const vpHomogeneousMatrix &cMo)
  {
    EdgesModel *m = dynamic_cast<EdgesModel*>(o->model->geometry);

    //Project edges. By default all edges not visible
    for (std::size_t i = 0; i < m->edges_list.size(); ++i)
    {
      m->edges_list[i]->project(cMo);
      m->edges_list[i]->active = false;
    }
    for (std::size_t i = 0; i < o->linkedTo.size(); ++i)
    {
      project_edges(o->linkedTo[i]->getNext(), cMo * o->linkedTo[i]->getNextObjectPose());
    }
  }

template<typename T>
  void VVSPoseEstimator<T>::select_visible_edges(Object *o, const vpHomogeneousMatrix &cMo)
  {
    EdgesModel *m = dynamic_cast<EdgesModel*>(o->model->geometry);

    vpHomogeneousMatrix oMc(cMo.inverse());
    for (std::size_t i = 0; i < m->faces_list.size(); ++i)
    {
      double vside = oMc[0][3] * m->faces_list[i]->normal[0] + oMc[1][3] * m->faces_list[i]->normal[1]
          + oMc[2][3] * m->faces_list[i]->normal[2] + m->faces_list[i]->normal[3];

      if (vside > 0.1)
      {
        //face is visible. Activate its edges
        for (std::size_t e = 0; e < m->faces_list[i]->edges.size(); ++e)
        {
          m->faces_list[i]->edges[e]->active = true;
        }
      }
    }
    for (std::size_t i = 0; i < o->linkedTo.size(); ++i)
    {
      select_visible_edges(o->linkedTo[i]->getNext(), cMo * o->linkedTo[i]->getNextObjectPose());
    }
  }

template<typename T>
  void VVSPoseEstimator<T>::find_edges(Object *o)
  {
    ROS_WARN("find_edges called");
    EdgesModel *m = dynamic_cast<EdgesModel*>(o->model->geometry);

    //project the selected model edges, find distance to real edges
    for (std::size_t i = 0; i < m->edges_list.size(); ++i)
    {
      if (m->edges_list[i]->active)
      {
        m->edges_list[i]->split(camobject_->view.getCols(), camobject_->view.getRows(), camobject_->camera->cparams,
                                distance_);
//      dynamic_cast<EdgesModel*>(o->model->geometry)->edges_list[i]->find(camobject_->view, camobject_->camera->cparams, ival_, CM_);
        m->edges_list[i]->find(camobject_->view, camobject_->camera->cparams, ival_);
      }
    }

    for (std::size_t i = 0; i < o->linkedTo.size(); ++i)
    {
      find_edges(o->linkedTo[i]->getNext());
    }
  }

template<typename T>
  void VVSPoseEstimator<T>::find_edges_from_descriptors(Object *o, const std::vector<vpColVector> &descriptors)
  {
    EdgesModel *m = dynamic_cast<EdgesModel*>(o->model->geometry);

    //project the selected model edges, find distance to real edges
    for (std::size_t i = 0; i < m->edges_list.size(); ++i)
    {
      if (m->edges_list[i]->active)
      {
        std::vector<vpColVector> edge_descriptors;
        std::cerr << std::endl << "Edges descriptors for edge " << i <<  std::endl;
        for (std::size_t d = 0; d  <descriptors.size(); ++d)
        {
          if (descriptors[d][0] == i)
          {
            std::cerr << "  - " << descriptors[d].t() << std::endl;
            edge_descriptors.push_back(descriptors[d]);
          }
        }

        m->edges_list[i]->split(camobject_->view.getCols(), camobject_->view.getRows(), camobject_->camera->cparams,
                                distance_);
        m->edges_list[i]->find_from_descriptor(camobject_->view, camobject_->camera->cparams, ival_, edge_descriptors);
      }
    }

    for (std::size_t i = 0; i < o->linkedTo.size(); ++i)
    {
      find_edges(o->linkedTo[i]->getNext());
    }
  }

template<typename T>
  void VVSPoseEstimator<T>::discard_bad_confidence_points(Object *o, vpMatrix &D, int index)
  {
    EdgesModel *m = dynamic_cast<EdgesModel*>(o->model->geometry);

    //bad confidence points
    for (std::size_t i = 0; i < m->edges_list.size(); ++i)
    {
      if (m->edges_list[i]->active)
      {
        for (std::size_t j = 0; j < m->edges_list[i]->points.size(); ++j)
        {
          if (m->edges_list[i]->points[j].maxgrad < mingrad_)
          {
            ROS_WARN_STREAM("Discarding " << i << " " << j << " because of small gradient " << m->edges_list[i]->points[j].maxgrad);
            D[index][index] = 0;
          }
          index++;
        }
      }
    }

    for (std::size_t i = 0; i < o->linkedTo.size(); ++i)
    {
      discard_bad_confidence_points(o->linkedTo[i]->getNext(), D, index);
    }
  }

template<typename T>
  vpMatrix &VVSPoseEstimator<T>::compute_s(Object *o)
  {
    //distance feature vector
    static vpMatrix s;

    s.resize(0, 1);
    stack_s_matrix(o, s);

    return s;
  }

template<typename T>
  void VVSPoseEstimator<T>::stack_s_matrix(Object *o, vpMatrix &s)
  {
    EdgesModel *m = dynamic_cast<EdgesModel*>(o->model->geometry);
    static vpMatrix v(1, 1);
    for (std::size_t i = 0; i < m->edges_list.size(); ++i)
    {
      for (std::size_t j = 0; j < m->edges_list[i]->points.size(); ++j)
      {
        v[0][0] = m->edges_list[i]->points[j].id;
        s = vpMatrix::stackMatrices(s, v);
      }
    }
    for (std::size_t i = 0; i < o->linkedTo.size(); i++)
    {
      stack_s_matrix(o->linkedTo[i]->getNext(), s);
    }
  }

template<typename T>
  vpMatrix &VVSPoseEstimator<T>::compute_Ls(Object *o)
  {
    //Estimation of the interaction matrix related to the distance feature
    static vpMatrix Ls;

    Ls.resize(0, 6);
    stack_Ls_matrix(o, Ls);

    return Ls;
  }

template<typename T>
  void VVSPoseEstimator<T>::stack_Ls_matrix(Object *o, vpMatrix &Ls)
  {
    EdgesModel *m = dynamic_cast<EdgesModel*>(o->model->geometry);

    double a, b, c, d, theta, rho, lrho, ltheta, ctheta, stheta, alpha, ldl;
    static vpMatrix Ldl(1, 6);
    for (std::size_t i = 0; i < m->edges_list.size(); ++i)
    {
      a = m->edges_list[i]->line.cP[4];
      b = m->edges_list[i]->line.cP[5];
      c = m->edges_list[i]->line.cP[6];
      d = m->edges_list[i]->line.cP[7];
      theta = m->edges_list[i]->line.getTheta();
      rho = m->edges_list[i]->line.getRho();
      ctheta = cos(theta);
      stheta = sin(theta);
      ltheta = (a * stheta - b * ctheta) / d;
      lrho = (a * rho * ctheta + b * rho * stheta + c) / d;

      for (std::size_t j = 0; j < m->edges_list[i]->points.size(); ++j)
      {
        alpha = m->edges_list[i]->points[j].irp[0] * stheta - m->edges_list[i]->points[j].irp[1] * ctheta;
        ldl = lrho + alpha * ltheta;

        Ldl[0][0] = ldl * ctheta;
        Ldl[0][1] = ldl * stheta;
        Ldl[0][2] = -ldl * rho;
        Ldl[0][3] = (1 + rho * rho) * stheta - alpha * rho * ctheta;
        Ldl[0][4] = -(1 + rho * rho) * ctheta - alpha * rho * stheta;
        Ldl[0][5] = -alpha;
        Ls = vpMatrix::stackMatrices(Ls, Ldl);
      }
    }

    for (std::size_t i = 0; i < o->linkedTo.size(); ++i)
    {
      stack_Ls_matrix(o->linkedTo[i]->getNext(), Ls);
    }
  }

template<typename T>
  void VVSPoseEstimator<T>::drawModel(Object *o, const vpImage<T> &I)
  {
    EdgesModel *m = dynamic_cast<EdgesModel*>(o->model->geometry);

    for (std::size_t i = 0; i < m->edges_list.size(); i++)
    {
      if (m->edges_list[i]->v1->v.get_Z() > 0 && m->edges_list[i]->v2->v.get_Z() > 0 && m->edges_list[i]->active)
      {
        double ax = 0.0, ay = 0.0, bx = 0.0, by = 0.0;
        vpMeterPixelConversion::convertPoint(camobject_->camera->cparams, m->edges_list[i]->v1->v.get_x(),
                                             m->edges_list[i]->v1->v.get_y(), ax, ay);
        vpMeterPixelConversion::convertPoint(camobject_->camera->cparams, m->edges_list[i]->v2->v.get_x(),
                                             m->edges_list[i]->v2->v.get_y(), bx, by);
        vpDisplay::displayLine(I, (int)round(ay), (int)round(ax), (int)round(by), (int)round(bx), vpColor::green, 2);
      }
    }

    for (std::size_t i = 0; i < o->linkedTo.size(); ++i)
    {
      drawModel(o->linkedTo[i]->getNext(), I);
    }
  }

template<typename T>
  void VVSPoseEstimator<T>::drawSearchPixels(Object *o, const vpImage<T> &I)
  {
    EdgesModel *m = dynamic_cast<EdgesModel*>(o->model->geometry);

    int cindex = 0;
    for (std::size_t i = 0; i < m->edges_list.size(); ++i)
    {
      if (m->edges_list[i]->active)
      {
        for (std::size_t j = 0; j < m->edges_list[i]->points.size(); ++j)
        {
          if ( ! detect_outliers_ || D_.getRows() == 0 || D_[cindex][cindex] > 0.5)
          {
            for (std::size_t k = 0; k < m->edges_list[i]->points[j].spoints.size(); ++k)
            {
              vpDisplay::displayPoint(I, (int)m->edges_list[i]->points[j].spoints[k][1],
                                      (int)m->edges_list[i]->points[j].spoints[k][0], vpColor::yellow);
            }
          }
          else
          {
            for (std::size_t k = 0; k < m->edges_list[i]->points[j].spoints.size(); ++k)
            {
              vpDisplay::displayPoint(I, (int)m->edges_list[i]->points[j].spoints[k][1],
                                      (int)m->edges_list[i]->points[j].spoints[k][0], vpColor::red);
            }
          }
          cindex++;
        }
      }
    }

    for (std::size_t i = 0; i < o->linkedTo.size(); ++i)
    {
      drawSearchPixels(o->linkedTo[i]->getNext(), I);
    }
  }

template<typename T>
  void VVSPoseEstimator<T>::drawMatchedPixels(Object *o, const vpImage<T> &I)
  {
    EdgesModel *m = dynamic_cast<EdgesModel*>(o->model->geometry);

    int cindex = 0;
    for (std::size_t i = 0; i < m->edges_list.size(); ++i)
    {
      if (m->edges_list[i]->active)
      {
        for (std::size_t j = 0; j < m->edges_list[i]->points.size(); ++j)
        {
          vpColVector rp = m->edges_list[i]->points[j].rp;
          if (! detect_outliers_ || D_.getRows() == 0 ||  D_[cindex][cindex] > 0.5)
          {
            //if there is confidence on this point, draw it in blue
            vpDisplay::displayCross(I, (int)rp[1], (int)rp[0], 8, vpColor::blue);
          }
          else
          {
            //if there is no confidence on this point, draw it in red
            vpDisplay::displayCross(I, (int)rp[1], (int)rp[0], 8, vpColor::red);
          }
          cindex++;
        }
      }
    }

    for (std::size_t i = 0; i < o->linkedTo.size(); ++i)
    {
      drawMatchedPixels(o->linkedTo[i]->getNext(), I);
    }
  }

#endif
