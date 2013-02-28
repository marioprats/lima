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

#include <lima/CAOEdgesModel.h>

/*!
 Load a 3D model contained in a ViSP .cao file.

 Adapted from:
 http://www.irisa.fr/lagadic/visp/documentation/visp-2.7.0/a00299.html#a68b9020b5f30677f0b0fd17640292ae7
 */
void CAOEdgesModel::computeEdges()
{
  std::ifstream file_id;
  file_id.open(cao_file_.c_str(), std::ifstream::in);
  if (file_id.fail())
  {
    std::cout << "cannot read CAO model file: " << cao_file_ << std::endl;
    throw vpException(vpException::ioError, "cannot read CAO model file");
  }

  char c;
  // Extraction of the version (remove empty line and commented ones (comment
  // line begin with the #)).
  while ((file_id.get(c) != NULL) && (c == '#'))
    file_id.ignore(256, '\n');
  file_id.unget();

  int caoVersion;
  file_id.get(c);
  if (c == 'V')
  {
    file_id >> caoVersion;
  }
  else
  {
    std::cout << "in vpMbEdgeTracker::loadCAOModel -> Bad parameter header file : use V0, V1, ...";
    throw vpException(vpException::badValue,
                      "in vpMbEdgeTracker::loadCAOModel -> Bad parameter header file : use V0, V1, ...");
  }

  while ((file_id.get(c) != NULL) && (c != '\n'))
    ;
  while ((file_id.get(c) != NULL) && (c == '#'))
    file_id.ignore(256, '\n');
  file_id.unget();

  //Read the points
  unsigned int caoNbrPoint;
  file_id >> caoNbrPoint;
  std::cout << "> " << caoNbrPoint << " points" << std::endl;
  vpPoint *caoPoints = NULL;
  if (caoNbrPoint > 0)
    caoPoints = new vpPoint[caoNbrPoint];

  double x; // 3D coordinates
  double y;
  double z;

  int i; // image coordinate (used for matching)
  int j;

  for (unsigned int k = 0; k < caoNbrPoint; k++)
  {
    file_id >> x;
    file_id >> y;
    file_id >> z;
    if (caoVersion == 2)
    {
      file_id >> i;
      file_id >> j;
    }

    if (k != caoNbrPoint - 1)
    { // the rest of the line is removed (not the last one due to the need to remove possible comments).
      file_id.ignore(256, '\n');
    }
    caoPoints[k].setWorldCoordinates(x, y, z);
  }

  while ((file_id.get(c) != NULL) && (c != '\n'))
    ;
  while ((file_id.get(c) != NULL) && (c == '#'))
    file_id.ignore(256, '\n');
  file_id.unget();

  //Read the lines
  unsigned int caoNbrLine;
  file_id >> caoNbrLine;
  unsigned int *caoLinePoints = NULL;
  std::cout << "> " << caoNbrLine << " lines" << std::endl;
  if (caoNbrLine > 0)
    caoLinePoints = new unsigned int[2 * caoNbrLine];

  unsigned int index1, index2;

  for (unsigned int k = 0; k < caoNbrLine; k++)
  {
    file_id >> index1;
    file_id >> index2;

    caoLinePoints[2 * k] = index1;
    caoLinePoints[2 * k + 1] = index2;

    if (index1 < caoNbrPoint && index2 < caoNbrPoint)
    {
      std::vector<vpPoint> extremities;
      extremities.push_back(caoPoints[index1]);
      extremities.push_back(caoPoints[index2]);
    }
    else
    {
      vpTRACE(" line %d has wrong coordinates.", k);
    }

    if (k != caoNbrLine - 1)
    { // the rest of the line is removed (not the last one due to the need to remove possible comments).
      file_id.ignore(256, '\n');
    }
  }

  while ((file_id.get(c) != NULL) && (c != '\n'))
    ;
  while ((file_id.get(c) != NULL) && (c == '#'))
    file_id.ignore(256, '\n');
  file_id.unget();

  /* Load polygon from the lines extracted earlier
   (the first point of the line is used)*/
  unsigned int caoNbrPolygonLine;
  file_id >> caoNbrPolygonLine;
  std::cout << "> " << caoNbrPolygonLine << " polygon line" << std::endl;
  unsigned int index;
  for (unsigned int k = 0; k < caoNbrPolygonLine; k++)
  {
    unsigned int nbLinePol;
    file_id >> nbLinePol;
    std::vector<vpPoint> corners;
    for (unsigned int i = 0; i < nbLinePol; i++)
    {
      file_id >> index;
      corners.push_back(caoPoints[caoLinePoints[2 * index]]);
    }
    if (k != caoNbrPolygonLine - 1)
    { // the rest of the line is removed (not the last one due to the need to remove possible comments).
      file_id.ignore(256, '\n');
    }
    addFace(corners, vpRGBa(1, 1, 1, 1));
  }

  while ((file_id.get(c) != NULL) && (c != '\n'))
    ;
  while ((file_id.get(c) != NULL) && (c == '#'))
    file_id.ignore(256, '\n');
  file_id.unget();

  /* Extract the polygon using the point coordinates (top of the file) */
  unsigned int caoNbrPolygonPoint;
  file_id >> caoNbrPolygonPoint;
  std::cout << "> " << caoNbrPolygonPoint << " polygon point" << std::endl;
  for (unsigned int k = 0; k < caoNbrPolygonPoint; k++)
  {
    int nbPointPol;
    file_id >> nbPointPol;
    std::vector<vpPoint> corners;
    for (int i = 0; i < nbPointPol; i++)
    {
      file_id >> index;
      corners.push_back(caoPoints[index]);
    }
    if (k != caoNbrPolygonPoint - 1)
    { // the rest of the line is removed (not the last one due to the need to remove possible comments).
      file_id.ignore(256, '\n');
    }
    addFace(corners, vpRGBa(1, 1, 1, 1));
  }

  while ((file_id.get(c) != NULL) && (c != '\n'))
    ;
  while ((file_id.get(c) != NULL) && (c == '#'))
    file_id.ignore(256, '\n');
  file_id.unget();

  if (file_id.eof())
  { // check if not at the end of the file (for old style files)
    delete[] caoPoints;
    delete[] caoLinePoints;
    return;
  }

  delete[] caoPoints;
  delete[] caoLinePoints;
}
