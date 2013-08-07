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

#include <lima/EdgesModel.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <ros/ros.h>

ConvolutionMatrix::ConvolutionMatrix(int tam, int num)
{
  /// Matrix size must be always an odd number
  tam_matrix = tam;
  if (tam % 2 == 0)
    tam_matrix++;

  nmatrices = num;
  radians_per_matrix = M_PI / nmatrices;
  int center = (tam - 1) / 2;
  float proximity = 0.5;

  double theta = -M_PI / 2;

  for (int i = 0; i < nmatrices; i++)
  {
    vpMatrix matrix(tam, tam);
// 		int matrix[tam][tam];
    for (int x = -(tam - 1) / 2; x <= (tam - 1) / 2; x++)
      for (int y = -(tam - 1) / 2; y <= (tam - 1) / 2; y++)
      {
        if ((y * cos(theta) + x * sin(theta)) <= proximity && (y * cos(theta) + x * sin(theta)) >= -proximity)
          matrix[center + x][center + y] = 0;
        else if ((y * cos(theta) + x * sin(theta)) < 0)
          matrix[center + x][center + y] = -100;
        else
          matrix[center + x][center + y] = 100;
      }
    matrices.push_back(matrix);
    theta = theta + radians_per_matrix;
  }
}

vpMatrix ConvolutionMatrix::getMatrix(double theta)
{
  int index = (int)round((theta + M_PI_2) / radians_per_matrix);
  if (index == 180)
    index++;

  return matrices[index];
}

void point2D::convert_to_meters(vpCameraParameters &c)
{
  vpPixelMeterConversion::convertPoint(c, p[0], p[1], ip[0], ip[1]);
  vpPixelMeterConversion::convertPoint(c, rp[0], rp[1], irp[0], irp[1]);
}

void point2D::compute_distance(vpLine &line)
{
  //line.p[1] is line theta
  id = ip[0] * cos(line.p[1]) + ip[1] * sin(line.p[1]) - irp[0] * cos(line.p[1]) - irp[1] * sin(line.p[1]);
}

void point2D::find_rp(vpImage<unsigned char> &I, vpColVector &n, int ival)
{
  //compute a set of pixels where to search for the gradient or image discontinuity
  spoints.clear();
  int pindex = -1;
  static vpColVector cp(2); //current point
  for (int i = -ival; i <= ival; ++i)
  {
    cp = p + i * n;
    cp[0] = vpMath::round(cp[0]);
    cp[1] = vpMath::round(cp[1]);
    if (cp[0] < I.getWidth() && cp[0] > 0 && cp[1] > 0 && cp[1] < I.getHeight())
    {
      spoints.push_back(cp);
      if (i < 0)
        pindex++;
    }
  }
  pindex--;

  //looks for the image discontinuity along the previous pixels
  maxgrad = abs(
      I[(int)spoints[pindex + 1][1]][(int)spoints[pindex + 1][0]]
          - I[(int)spoints[pindex - 1][1]][(int)spoints[pindex - 1][0]]);
  static vpColVector maxp(2);
  maxp = p; //initially, max gradient is at expected point position

  static double g = 0;
  for (std::size_t i = 1; i < spoints.size() - 1; ++i)
  {
    g = abs(I[(int)spoints[i + 1][1]][(int)spoints[i + 1][0]] - I[(int)spoints[i - 1][1]][(int)spoints[i - 1][0]]);
    if (g > maxgrad)
    {
      maxgrad = g;
      maxp = spoints[i];
    }
  }
  rp = maxp;
}

void point2D::find_rp(vpImage<unsigned char> &I, vpColVector &n, int ival, vpColVector &descriptor)
{
  //compute a set of pixels where to search for the descriptor
  spoints.clear();
  static vpColVector cp(2); //current point
  for (int i = -ival; i <= ival; ++i)
  {
    cp = p + i * n;
    cp[0] = vpMath::round(cp[0]);
    cp[1] = vpMath::round(cp[1]);
    if (cp[0] < I.getWidth() && cp[0] > 0 && cp[1] > 0 && cp[1] < I.getHeight())
    {
      spoints.push_back(cp);
    }
  }

  static double g = 0;
  double min_ssd = std::numeric_limits<double>::max();
  std::size_t min_ssd_index = 0;
  for (std::size_t i = 0; i < spoints.size(); ++i)
  {
    double ssd_i = 0;
    for (std::size_t d = 0 ; d < descriptor.getRows(); ++d)
    {
      cp = spoints[i] + ((int)d - (int)descriptor.getRows() / 2.0) * n;
      cp[0] = vpMath::round(cp[0]);
      cp[1] = vpMath::round(cp[1]);
      if (cp[0] < I.getWidth() && cp[0] > 0 && cp[1] > 0 && cp[1] < I.getHeight())
      {
        ssd_i += pow( I[(int)cp[1]][(int)cp[0]] - descriptor[d], 2 );
      }
      else
      {
        ssd_i += pow( 0 - descriptor[d], 2 );
      }
    }
    ssd_i = sqrt(ssd_i);

    if (ssd_i < min_ssd)
    {
      min_ssd = ssd_i;
      min_ssd_index = i;
    }
  }
  maxgrad = std::numeric_limits<double>::max();
  rp = spoints[min_ssd_index];
}

void point2D::find_rp(vpImage<vpRGBa> &I, vpColVector &n, int ival, vpColVector &descriptor)
{
  //compute a set of pixels where to search for the descriptor
  spoints.clear();
  static vpColVector cp(2); //current point
  for (int i = -ival; i <= ival; ++i)
  {
    cp = p + i * n;
    cp[0] = vpMath::round(cp[0]);
    cp[1] = vpMath::round(cp[1]);
    if (cp[0] < I.getWidth() && cp[0] > 0 && cp[1] > 0 && cp[1] < I.getHeight())
    {
      spoints.push_back(cp);
    }
  }

  static double g = 0;
  double min_ssd = std::numeric_limits<double>::max();
  std::size_t min_ssd_index = 0;
  for (std::size_t i = 0; i < spoints.size(); ++i)
  {
    double ssd_i = 0;
    for (std::size_t d = 2 ; d < descriptor.getRows(); ++d)
    {
      cp = spoints[i] + (d - (descriptor.getRows() - 2) / 2.0) * n;
      cp[0] = vpMath::round(cp[0]);
      cp[1] = vpMath::round(cp[1]);
      if (cp[0] < I.getWidth() && cp[0] > 0 && cp[1] > 0 && cp[1] < I.getHeight())
      {
        unsigned int grey_value = (unsigned int)((I[(int)cp[1]][(int)cp[0]].R + I[(int)cp[1]][(int)cp[0]].G + I[(int)cp[1]][(int)cp[0]].B) / 3.0);
        ssd_i += pow( grey_value - descriptor[d], 2 );
      }
      else
      {
        ssd_i += pow( 0 - descriptor[d], 2 );
      }
    }
    ssd_i = sqrt(ssd_i);
    if (ssd_i < min_ssd)
    {
      min_ssd = ssd_i;
      min_ssd_index = i;
    }
  }

  maxgrad = std::numeric_limits<double>::max();
  rp = spoints[min_ssd_index];
}

void point2D::find_rp(vpImage<vpRGBa> &I, vpColVector &n, int ival)
{
  //compute a set of pixels where to search for the gradient or image discontinuity
  spoints.clear();
  int pindex = -1;
  static vpColVector cp(2); //current point
  for (int i = -ival; i <= ival; ++i)
  {
    cp = p + i * n;
    cp[0] = vpMath::round(cp[0]);
    cp[1] = vpMath::round(cp[1]);
    if (cp[0] < I.getWidth() && cp[0] > 0 && cp[1] > 0 && cp[1] < I.getHeight())
    {
      spoints.push_back(cp);
      if (i < 0)
        pindex++;
    }
  }
  pindex--;

  //looks for the image discontinuity along the previous pixels
  maxgrad = (I[(int)spoints[pindex + 1][1]][(int)spoints[pindex + 1][0]]
             - I[(int)spoints[pindex - 1][1]][(int)spoints[pindex - 1][0]]).euclideanNorm();
  static vpColVector maxp(2);
  maxp = p; //initially, max gradient is at expected point position

  static double g = 0;
  for (std::size_t i = 1; i < spoints.size() - 1; ++i)
  {
    g = (I[(int)spoints[i + 1][1]][(int)spoints[i + 1][0]] - I[(int)spoints[i - 1][1]][(int)spoints[i - 1][0]]).euclideanNorm();
    if (g > maxgrad)
    {
      maxgrad = g;
      maxp = spoints[i];
    }
  }
  rp = maxp;
}

void point2D::find_rp_ConvolutionMatrix(vpImage<unsigned char> &I, ConvolutionMatrix * CM, vpColVector &n, double theta,
                                        int ival)
{
  //compute a set of pixels where to search for the gradient or image discontinuity
  spoints.clear();
  int pindex = -1;
  int tam = CM->getSize();

  vpColVector cp(2); //current point
  for (int i = -ival; i <= ival; i++)
  {
    cp = p + i * n;
    cp[0] = vpMath::round(cp[0]);
    cp[1] = vpMath::round(cp[1]);
    if (cp[0] < I.getWidth() && cp[0] > 0 && cp[1] > 0 && cp[1] < I.getHeight())
    {
      spoints.push_back(cp);
      if (i < 0)
        pindex++;
    }
  }
  pindex--;

  int center = (tam - 1) / 2;
  vpColVector maxp(2);
  maxgrad = 0;
  vpMatrix Matrix(CM->getMatrix(theta));

  double value = 0;
  double image_value = 0.0;

  vector<double> gradients;
  vector<int> positions;

  for (std::size_t k = 1; k < spoints.size() - 1; k++)
  {
    double res = 0.0;
    for (int i = -(tam - 1) / 2; i <= (tam - 1) / 2; i++)
      for (int j = -(tam - 1) / 2; j <= (tam - 1) / 2; j++)
      {

        if (((int)spoints[k][1] + i >= 0) && ((int)spoints[k][1] + i < (int)I.getHeight())
            && ((int)spoints[k][0] + j >= 0) && ((int)spoints[k][0] + j < (int)I.getWidth()))
        {
          image_value = I[(int)spoints[k][1] + i][(int)spoints[k][0] + j];
          value = Matrix[center + i][center + j];
          res += image_value * value;
        }
      }
    unsigned int fabsres = fabs(res);
    gradients.push_back(fabsres);
    positions.push_back(k);

    if (fabsres > maxgrad)
    {
      maxgrad = fabsres;
      maxp = spoints[k];
    }
  }

  int centerpos = (int)spoints.size() / 2;
  int pos = centerpos;
  int maxpotential = 0;
  for (std::size_t i = 0; i < gradients.size(); ++i)
  {
    int potential = gradients[i] / (1 + 0.5 * abs(centerpos - positions[i]));

    if (potential > maxpotential)
    {
      pos = i;
      maxpotential = potential;
    }
  }

  rp = spoints[positions[pos]];
}

vpColVector point2D::compute_edge_descriptor(vpImage<unsigned char> &I, vpColVector &n, unsigned int size)
{
  vpColVector output;
  static vpColVector cp(2); //current point
  unsigned int max_intensity = 0;
  unsigned int min_intensity = 255;
  for (int i = -size / 2.0; i <= size / 2.0; ++i)
  {
    cp = p + i * n;
    cp[0] = vpMath::round(cp[0]);
    cp[1] = vpMath::round(cp[1]);
    if (cp[0] < I.getWidth() && cp[0] > 0 && cp[1] > 0 && cp[1] < I.getHeight())
    {
      unsigned int image_intensity = I[(int)cp[1]][(int)cp[0]];
      output.stack(image_intensity);
      if (image_intensity < min_intensity) min_intensity = image_intensity;
      if (image_intensity > max_intensity) max_intensity = image_intensity;
    }
  }
//  for (std::size_t i = 0; i < output.getRows(); ++i)
//  {
//    output[i] = (output[i] - min_intensity) / (max_intensity - min_intensity);
//  }

  return output;
}

vpColVector point2D::compute_edge_descriptor(vpImage<vpRGBa> &I, vpColVector &n, unsigned int size)
{
  vpColVector output(0);
  static vpColVector cp(2); //current point
  unsigned int max_intensity = 0;
  unsigned int min_intensity = 255;
  for (int i = -(int)size / 2.0; i <= size / 2.0; ++i)
  {
    cp = p + i * n;
    cp[0] = vpMath::round(cp[0]);
    cp[1] = vpMath::round(cp[1]);
    if (cp[0] < I.getWidth() && cp[0] > 0 && cp[1] > 0 && cp[1] < I.getHeight())
    {
      unsigned int image_intensity = (unsigned int)((I[(int)cp[1]][(int)cp[0]].R + I[(int)cp[1]][(int)cp[0]].G + I[(int)cp[1]][(int)cp[0]].B) / 3.0);
      output.resize(output.getRows() + 1, false);
      output[output.getRows() - 1] = image_intensity;
      if (image_intensity < min_intensity) min_intensity = image_intensity;
      if (image_intensity > max_intensity) max_intensity = image_intensity;
    }
  }

//  for (std::size_t i = 0; i < output.getRows(); ++i)
//  {
//    output[i] = (output[i] - min_intensity) / (max_intensity - min_intensity);
//  }

  return output;
}

void edge::project(const vpHomogeneousMatrix &cMo)
{
  //the line corresponding to the edge, along with the edge vertex are transformed to 
  //camera coordinates and projected.
  line.changeFrame(cMo);
  line.project();
  v1->v.changeFrame(cMo);
  v1->v.project();
  v2->v.changeFrame(cMo);
  v2->v.project();
}

void edge::split(unsigned int cols, unsigned int rows, vpCameraParameters &c, int distance)
{
  if (v1->v.get_Z() > 0 || v2->v.get_Z() > 0)
  {
    vpColVector p(2), q(2), evector(2), evector_unit(2);

    vpPoint at, bt;
    at = v1->v;
    bt = v2->v;

    //if vertex a is behind the camera, we compute a new point in the same edge, but
    //0.1m in front of the camera
    if (v1->v.get_Z() < 0)
    {
      double a1, b1, c1, d1;
      double a2, b2, c2, d2;
      a1 = line.cP[0];
      b1 = line.cP[1];
      c1 = line.cP[2];
      d1 = line.cP[3];
      a2 = line.cP[4];
      b2 = line.cP[5];
      c2 = line.cP[6];
      d2 = line.cP[7];

      double dd1 = c1 * 0.1 + d1;
      double dd2 = c2 * 0.1 + d2;

      double x, y;
      y = ((a2 * dd1) / a1 - dd2) / (b2 - (a2 * b1) / a1);
      x = -(b1 * y + dd1) / a1;
      at.set_X(x);
      at.set_Y(y);
      at.set_Z(0.1);
      at.set_W(1);
      at.projection();
    }

    //if vertex b is behind the camera, we compute a new point in the same edge, but
    //0.1m in front of the camera.
    if (v2->v.get_Z() < 0)
    {
      double a1, b1, c1, d1;
      double a2, b2, c2, d2;
      a1 = line.cP[0];
      b1 = line.cP[1];
      c1 = line.cP[2];
      d1 = line.cP[3];
      a2 = line.cP[4];
      b2 = line.cP[5];
      c2 = line.cP[6];
      d2 = line.cP[7];

      double dd1 = c1 * 0.1 + d1;
      double dd2 = c2 * 0.1 + d2;

      double x, y;
      y = ((a2 * dd1) / a1 - dd2) / (b2 - (a2 * b1) / a1);
      x = -(b1 * y + dd1) / a1;
      bt.set_X(x);
      bt.set_Y(y);
      bt.set_Z(0.1);
      bt.set_W(1);
      bt.projection();
    }

    //p and q are at and bt converted to image coordinates
    vpMeterPixelConversion::convertPoint(c, at.get_x(), at.get_y(), p[0], p[1]);
    vpMeterPixelConversion::convertPoint(c, bt.get_x(), bt.get_y(), q[0], q[1]);

    //vector and unit vector from p to q
    evector = q - p;
    evector_unit = evector;
    evector_unit.normalize();

    //along the vector from p to q, we take one pixel each 'distance' pixels
    points.clear();
    int npoints = int(evector.euclideanNorm() / distance);

    vpColVector point(2);
    for (int pi = 1; pi <= npoints; pi++)
    {
      point[0] = vpMath::round(p[0] + evector_unit[0] * pi * distance);
      point[1] = vpMath::round(p[1] + evector_unit[1] * pi * distance);
      if (point[0] < cols - 5 && point[0] > 5 && point[1] > 5 && point[1] < rows - 5)
      {
        points.push_back(point2D(point));
      }
    }
  }
}

void edge::find(vpImage<unsigned char> &I, vpCameraParameters &c, int ival)
{
  //we compute the normal to the edge
  vpColVector normal(2);
  double theta = line.getTheta();
  normal[0] = cos(theta);
  normal[1] = sin(theta);

  //for each sampled point in the edge, search along the normal for the real image point
  //and compute the distance feature point-to-line
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    points[i].find_rp(I, normal, ival);
    points[i].convert_to_meters(c);
    points[i].compute_distance(line);
  }
}

void edge::find(vpImage<vpRGBa> &I, vpCameraParameters &c, int ival)
{
  //we compute the normal to the edge
  vpColVector normal(2);
  double theta = line.getTheta();
  normal[0] = cos(theta);
  normal[1] = sin(theta);

  //for each sampled point in the edge, search along the normal for the real image point
  //and compute the distance feature point-to-line
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    points[i].find_rp(I, normal, ival);
    points[i].convert_to_meters(c);
    points[i].compute_distance(line);
  }
}

void edge::find_from_descriptor(vpImage<unsigned char> &I, vpCameraParameters &c, int ival, const std::vector<vpColVector> &edge_descriptors)
{
  //we compute the normal to the edge
  vpColVector normal(2);
  double theta = line.getTheta();
  normal[0] = cos(theta);
  normal[1] = sin(theta);

  //for each sampled point in the edge, search along the normal for the real image point
  //and compute the distance feature point-to-line
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    //get the closest point descriptor
    vpColVector descriptor = edge_descriptors[(int)(edge_descriptors.size() * i / points.size())];

    points[i].find_rp(I, normal, ival, descriptor);
    points[i].convert_to_meters(c);
    points[i].compute_distance(line);
  }
}

void edge::find_from_descriptor(vpImage<vpRGBa> &I, vpCameraParameters &c, int ival, const std::vector<vpColVector> &edge_descriptors)
{
  //we compute the normal to the edge
  vpColVector normal(2);
  double theta = line.getTheta();
  normal[0] = cos(theta);
  normal[1] = sin(theta);

  //for each sampled point in the edge, search along the normal for the real image point
  //and compute the distance feature point-to-line
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    //get the closest point descriptor
    vpColVector descriptor = edge_descriptors[(int)(edge_descriptors.size() * i / points.size())];

    points[i].find_rp(I, normal, ival, descriptor);
    points[i].convert_to_meters(c);
    points[i].compute_distance(line);
  }
}

void edge::find(vpImage<unsigned char> &I, vpCameraParameters &c, int ival, ConvolutionMatrix * CM)
{
  //we compute the normal to the edge
  vpColVector normal(2);
  double theta = line.getTheta();
  normal[0] = cos(theta);
  normal[1] = sin(theta);

  //for each sampled point in the edge, search along the normal for the real image point
  //and compute the distance feature point-to-line
  for (std::size_t i = 0; i < points.size(); i++)
  {
    points[i].find_rp_ConvolutionMatrix(I, CM, normal, theta, ival);
    points[i].convert_to_meters(c);
    points[i].compute_distance(line);
  }
}

EdgesModel::EdgesModel() :
    GeomModel()
{
}

bool EdgesModel::compareVertex(vertex* v1, vertex *v2)
{
  if (v1->v.get_oX() == v2->v.get_oX() && v1->v.get_oY() == v2->v.get_oY() && v1->v.get_oZ() == v2->v.get_oZ())
  {
    return true;
  }

  return false;
}

vertex* EdgesModel::addVertex(vpPoint v)
{

  // Check vertex_list to find v
  vertex * vp = new vertex(v);
  for (std::size_t i = 0; i < vertex_list.size(); ++i)
  {
    if (compareVertex(vertex_list[i], vp))
      return vertex_list[i];
  }

  // If vertex doesn't exist into vertex_list, create it
  vertex_list.push_back(vp);

  return vp;
}

vpColVector EdgesModel::faceNormal(vector<vpPoint> vlist)
{
  vpColVector v1(3);
  v1[0] = vlist[1].get_oX() - vlist[0].get_oX();
  v1[1] = vlist[1].get_oY() - vlist[0].get_oY();
  v1[2] = vlist[1].get_oZ() - vlist[0].get_oZ();

  vpColVector v2(3);
  v2[0] = vlist[2].get_oX() - vlist[0].get_oX();
  v2[1] = vlist[2].get_oY() - vlist[0].get_oY();
  v2[2] = vlist[2].get_oZ() - vlist[0].get_oZ();

  vpColVector cross(3);
  cross = vpColVector::cross(v1, v2);
  cross.normalize();

  return cross;
}

vpColVector EdgesModel::faceNormal(vector<vertex*> vlist)
{
  vpColVector v1(3);
  v1[0] = vlist[1]->v.get_oX() - vlist[0]->v.get_oX();
  v1[1] = vlist[1]->v.get_oY() - vlist[0]->v.get_oY();
  v1[2] = vlist[1]->v.get_oZ() - vlist[0]->v.get_oZ();

  vpColVector v2(3);
  v2[0] = vlist[2]->v.get_oX() - vlist[0]->v.get_oX();
  v2[1] = vlist[2]->v.get_oY() - vlist[0]->v.get_oY();
  v2[2] = vlist[2]->v.get_oZ() - vlist[0]->v.get_oZ();

  vpColVector cross(3);
  cross = vpColVector::cross(v1, v2);
  cross.normalize();

  return cross;
}

int EdgesModel::existsEdge(vertex * v1, vertex * v2)
{
  for (std::size_t i = 0; i < edges_list.size(); ++i)
    if ((compareVertex(edges_list[i]->v1, v1) && compareVertex(edges_list[i]->v2, v2))
        || (compareVertex(edges_list[i]->v1, v2) && compareVertex(edges_list[i]->v2, v1)))
      return i;

  return -1;
}

edge *EdgesModel::addEdge(vpColVector fnormal, vertex* v1, vertex* v2)
{

  int pos = existsEdge(v1, v2);

  if (pos != -1)
    return edges_list[pos];

  vpColVector lvector(3);
  lvector[0] = v2->v.get_oX() - v1->v.get_oX();
  lvector[1] = v2->v.get_oY() - v1->v.get_oY();
  lvector[2] = v2->v.get_oZ() - v1->v.get_oZ();

  vpColVector ovector = vpColVector::cross(fnormal, lvector);

  vpColVector plano1(4), plano2(4);
  plano1[0] = fnormal[0];
  plano1[1] = fnormal[1];
  plano1[2] = fnormal[2];
  plano1[3] = -plano1[0] * v1->v.get_oX() - plano1[1] * v1->v.get_oY() - plano1[2] * v1->v.get_oZ();

  plano2[0] = ovector[0];
  plano2[1] = ovector[1];
  plano2[2] = ovector[2];
  plano2[3] = -plano2[0] * v1->v.get_oX() - plano2[1] * v1->v.get_oY() - plano2[2] * v1->v.get_oZ();

  edge * e = new edge(plano1, plano2, v1, v2);
  edges_list.push_back(e);
  return e;
}

void EdgesModel::addFace(vector<vpPoint> vlist, vpRGBa color)
{
  vpColVector normal = faceNormal(vlist);
  vector<vertex*> face_vertex_list;
  vector<edge*> edges;
  int v1_in, v2_in;

  //TODO: This should be rewriten. It can be done in a much more easier way
  for (std::size_t i = 1; i <= vlist.size(); ++i)
  {
    vertex * v1 = addVertex(vlist[i - 1]);
    vertex * v2;
    if (i == vlist.size())
    {
      v2 = addVertex(vlist[0]);
      edges.push_back(addEdge(normal, v1, v2));
    }
    else
    {
      v2 = addVertex(vlist[i]);
      edges.push_back(addEdge(normal, v1, v2));
    }

    v1_in = 0;
    v2_in = 0;

    int size = face_vertex_list.size();
    for (int j = 0; j < size; j++)
    {
      if (compareVertex(face_vertex_list[j], v1))
      {
        v1_in = 1;
      }
      if (compareVertex(face_vertex_list[j], v2))
      {
        v2_in = 1;
      }
    }

    if (v1_in == 0)
    {
      face_vertex_list.push_back(v1);
    }

    if (v2_in == 0)
    {
      face_vertex_list.push_back(v2);
    }
  }

  face * f = new face(normal, edges, face_vertex_list, color);

  faces_list.push_back(f);
}

#ifdef _USE_GL_
void EdgesModel::GLRender()
{
  //Tesellation
  GLUtriangulatorObj * tess;
  tess = gluNewTess();

  gluTessCallback(tess, GLU_BEGIN, (GLvoid ( *) ( )) glBegin);
  gluTessCallback(tess, GLU_VERTEX, (GLvoid ( *) ( )) glVertex3dv);
  gluTessCallback(tess, GLU_END, (GLvoid ( *) ( )) glEnd);
  gluTessProperty(tess, GLU_TESS_WINDING_RULE,GLU_TESS_WINDING_ODD);

  glPushMatrix();

  for (int i=0; i<faces_list.size(); i+=1)
  {
    glColor4f(faces_list[i]->color.R/255.0,faces_list[i]->color.G/255.0,faces_list[i]->color.B/255.0,1);

    GLdouble listado[vertex_list.size()][3];
    for(int j=0;j<faces_list[i]->vertex_list.size();j+=1)
    {
      listado[j][0]=(GLdouble) faces_list[i]->vertex_list[j]->v.get_oX();
      listado[j][1]=(GLdouble) faces_list[i]->vertex_list[j]->v.get_oY();
      listado[j][2]=(GLdouble) faces_list[i]->vertex_list[j]->v.get_oZ();
    }

    gluBeginPolygon(tess);
    for(int j=0;j<faces_list[i]->vertex_list.size();j++)
    {
      gluTessVertex(tess, listado[j],listado[j]);
    }
    gluEndPolygon(tess);
  }
  glPopMatrix();
  gluDeleteTess(tess);
}
#endif
