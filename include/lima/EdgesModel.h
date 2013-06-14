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

#ifndef EDGESMODEL_H
#define EDGESMODEL_H

#include <lima/GeomModel.h>
#include <visp/vpColVector.h>
#include <visp/vpRGBa.h>

#include <visp/vpPoint.h>
#include <visp/vpLine.h>

#include <vector>

using namespace std;

/** Holds a vertex of a 3D model */
class vertex
{
public:
  vpPoint v;
  vertex(vpPoint vert)
  {
    v = vert;
  }
  ;

  ~vertex()
  {
  }
  ;
};

/** A set of convolution matrices for oriented edge detection */
class ConvolutionMatrix
{
  int tam_matrix; ///< size of matrix ( tam_matrix x tam_matrix)
  int nmatrices; ///< number of matrices that will be created
  float radians_per_matrix; ///< increase in radians from one matrix to another
public:
  vector<vpMatrix> matrices; ///< vector of convolution matrices

  ConvolutionMatrix(int tam = 7, int num = 180);

  vpMatrix getMatrix(double theta);
  int getSize()
  {
    return tam_matrix;
  }
};

class point2D
{
public:
  vpColVector p; ///< point at its expected position (image coords)
  vpColVector rp; ///< point at its real position (image coords)
  vpColVector ip; ///< point at its expected position (projective camera coords)
  vpColVector irp; ///< point at its real position (projective camera coords)
  float id; ///< distance between expected and real position (projective camera coords)

  vector<vpColVector> spoints; ///< points where to search for the image discontinuity
  double maxgrad; ///< maximum gradient found in the search path

  point2D(vpColVector p)
  {
    this->p = p;
    rp.resize(p.getRows());
    ip.resize(p.getRows());
    irp.resize(p.getRows());
  }

  /** finds the real point in the image along the normal n */
  void find_rp(vpImage<unsigned char> &I, vpColVector &n, int ival);
  void find_rp(vpImage<vpRGBa> &I, vpColVector &n, int ival);
  void find_rp(vpImage<unsigned char> &I, vpColVector &n, int ival, vpColVector &descriptor);
  void find_rp(vpImage<vpRGBa> &I, vpColVector &n, int ival, vpColVector &descriptor);

  /** finds the real point in the image along the normal n using an oriented mask */
  void find_rp_ConvolutionMatrix(vpImage<unsigned char> & I, ConvolutionMatrix *CM, vpColVector &n, double theta,
                                 int ival);

  /** Converts the estimated and real points to projective camera coordinates */
  void convert_to_meters(vpCameraParameters &c);

  /** Computes the distance feature in projective coordinates */
  void compute_distance(vpLine &line);

  /** Compute edge descriptor */
  vpColVector compute_edge_descriptor(vpImage<unsigned char> &I, vpColVector &n, unsigned int size);
  vpColVector compute_edge_descriptor(vpImage<vpRGBa> &I, vpColVector &n, unsigned int size);

  ~point2D()
  {
  }
};

class edge
{
public:
  vpColVector p1, p2; ///< parameters A,B,C,D of two planes (in object frame) which intersection is the edge
  vertex *v1, *v2; ///< 3D points which limit the 3D edge
  vpLine line; ///<Visp objet for 3D/2D lines
  vector<point2D> points; ///< a set of 2D points along the 2D edge
  bool active; ///< whether to consider this edge for pose estimation or not

  edge(vpColVector pl1, vpColVector pl2, vertex * a, vertex * b)
  {
    p1 = pl1;
    p2 = pl2;
    line.setWorldCoordinates(p1, p2);
    v1 = a;
    v2 = b;
  }

  void project(const vpHomogeneousMatrix &cMo); ///< transform the edge to projected camera coordinates

  /** Samples the 2D edge in a set of image points */
  void split(unsigned int cols, unsigned int rows, vpCameraParameters &c, int distance);

  /** Find the edge in the image using simple search along the normal*/
  void find(vpImage<unsigned char> &I, vpCameraParameters &c, int ival);
  void find(vpImage<vpRGBa> &I, vpCameraParameters &c, int ival);

  void find_from_descriptor(vpImage<unsigned char> &I, vpCameraParameters &c, int ival, const std::vector<vpColVector> &edge_descriptors);
  void find_from_descriptor(vpImage<vpRGBa> &I, vpCameraParameters &c, int ival, const std::vector<vpColVector> &edge_descriptors);

  /** Find the edge in the image using oriented gradient masks*/
  void find(vpImage<unsigned char> &I, vpCameraParameters &c, int ival, ConvolutionMatrix * CM);

  ~edge()
  {
  }
};

class face
{
public:
  vector<vertex*> vertex_list;
  vpRGBa color; ///< Color of the face
  vector<edge*> edges; ///< edges that compose the face
  vpColVector normal; ///< parameters A,B,C,D of the plane that contains the face (normals must match)

  face(vpColVector plane, vector<edge*> list, vector<vertex*> vlist, vpRGBa color)
  {
    normal = plane;
    edges = list;
    vertex_list = vlist;
    setColor(color);
  }

  void setColor(vpRGBa col)
  {
    this->color = col;
  }

  ~face()
  {
  }
};

#ifdef _USE_GL_
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glext.h>
#endif

#include <stdlib.h>

/**
 A edges model. The reference frame is in the center of mass.
 @author Mario Prats <mprats@icc.uji.es>
 */

class EdgesModel : virtual public GeomModel
{
public:
  vector<face*> faces_list;
  vector<edge*> edges_list;
  vector<vertex*> vertex_list;

//   vpRGBa color;		//< Color of the box

  /** Default constructor */
  EdgesModel();
//   
  /** Gets the type of geometric model (returns "edges")*/
  virtual string getType()
  {
    return "edges";
  }

  /** Puts a new face into EdgeModel */
  void addFace(vector<vpPoint> vlist, vpRGBa color);

#ifdef _USE_GL_
  virtual void GLRender();
#endif

  ~EdgesModel()
  {
  }

protected:
  /** Puts a new edge into EdgeModel */
  edge* addEdge(vpColVector fnormal, vertex* v1, vertex* p2);

  /** Check if edge exists into edge list structure and return its index */
  int existsEdge(vertex * v1, vertex * v2);

  /** Computes the normal vector of the face composed by a set of vertex */
  vpColVector faceNormal(vector<vpPoint> vlist);
  vpColVector faceNormal(vector<vertex*> vlist);

  /** Puts a new vertex into EdgeModel*/
  vertex *addVertex(vpPoint v);

  /** Compare two vertex */
  bool compareVertex(vertex* v1, vertex *v2);

};

#endif

