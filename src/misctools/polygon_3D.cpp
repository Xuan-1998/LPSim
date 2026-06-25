//---------------------------------------------------------------------------------------------------------------------
// Copyright 2017, 2018 Purdue University, Ignacio Garcia Dorado, Daniel Aliaga
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
// following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
// following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
// following disclaimer in the documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
// products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
// USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//---------------------------------------------------------------------------------------------------------------------

#include "polygon_3D.h"
#include "qvector2d.h"
#include <float.h>

namespace LC {
namespace misctools {
QVector3D calculateNormal(QVector3D &p0, QVector3D &p1, QVector3D &p2);

QVector3D Polygon3D::getLoopNormalVector(Loop3D &pin) {
  if (pin.size() >= 3) {
    return (calculateNormal(pin[0], pin[1], pin[2]));
  }

  return (QVector3D(0, 0, 0));
}

QVector3D Polygon3D::getNormalVector() {
  if (this->normalVec.isNull()) {
    normalVec = getLoopNormalVector(this->contour);
  }

  return normalVec;
}

bool segmentSegmentIntersectXY(QVector2D &a, QVector2D &b, QVector2D &c,
                               QVector2D &d,
                               float *tab, float *tcd, bool segmentOnly, QVector2D &intPoint);

//Only works for polygons with no holes in them
bool Polygon3D::splitMeWithPolyline(std::vector<QVector3D> &pline,
                                    Loop3D &pgon1, Loop3D &pgon2) {
  bool polylineIntersectsPolygon = false;

  int plineSz = pline.size();
  int contourSz = this->contour.size();

  if (plineSz < 2 || contourSz < 3) {
    //std::cout << "ERROR: Cannot split if polygon has fewer than three vertices of if polyline has fewer than two points\n.";
    return false;
  }

  QVector2D tmpIntPt;
  QVector2D firstIntPt;
  QVector2D secondIntPt;
  float tPline, tPgon;
  int firstIntPlineIdx    = -1;
  int secondIntPlineIdx   = -1;
  int firstIntContourIdx  = -1;
  int secondIntContourIdx = -1;
  int intCount = 0;


  //iterate along polyline
  for (int i = 0; i < plineSz - 1; ++i) {
    int iNext = i + 1;

    for (int j = 0; j < contourSz; ++j) {
      int jNext = (j + 1) % contourSz;

      QVector2D plinei(pline[i]);
      QVector2D plineinext(pline[iNext]);
      QVector2D contourj(contour[j]);
      QVector2D contourjnext(contour[jNext]);

      if (LC::misctools::segmentSegmentIntersectXY(plinei, plineinext,
          contourj, contourjnext,
          &tPline, &tPgon, true,
          tmpIntPt)) {
        polylineIntersectsPolygon = true;

        //first intersection
        if (intCount == 0) {
          firstIntPlineIdx = i;
          firstIntContourIdx = j;
          firstIntPt = tmpIntPt;
        } else if (intCount == 1) {
          secondIntPlineIdx = i;
          secondIntContourIdx = j;
          secondIntPt = tmpIntPt;
        } else {
          //std::cout << "Cannot split - Polyline intersects polygon at more than two points.\n";
          return false;
        }

        intCount++;
      }
    }
  }

  if (intCount != 2) {
    //std::cout << "Cannot split - Polyline intersects polygon at " << intCount <<" points\n";
    return false;
  }

  //Once we have intersection points and indexes, we reconstruct the two polygons
  pgon1.clear();
  pgon2.clear();
  int pgonVtxIte;
  int plineVtxIte;

  //If first polygon segment intersected has an index greater
  //	than second segment, modify indexes for correct reconstruction
  if (firstIntContourIdx > secondIntContourIdx) {
    secondIntContourIdx += contourSz;
  }

  //==== Reconstruct first polygon
  //-- append polygon contour
  pgon1.push_back(firstIntPt);
  pgonVtxIte = firstIntContourIdx;

  while (pgonVtxIte < secondIntContourIdx) {
    pgon1.push_back(contour[(pgonVtxIte + 1) % contourSz]);
    pgonVtxIte++;
  }

  pgon1.push_back(secondIntPt);
  //-- append polyline points
  plineVtxIte = secondIntPlineIdx;

  while (plineVtxIte > firstIntPlineIdx) {
    pgon1.push_back(pline[(plineVtxIte)]);
    plineVtxIte--;
  }

  //==== Reconstruct second polygon
  //-- append polygon contour
  pgon2.push_back(secondIntPt);
  pgonVtxIte = secondIntContourIdx;

  while (pgonVtxIte < firstIntContourIdx + contourSz) {
    pgon2.push_back(contour[(pgonVtxIte + 1) % contourSz]);
    pgonVtxIte++;
  }

  pgon2.push_back(firstIntPt);
  //-- append polyline points
  plineVtxIte = firstIntPlineIdx;

  while (plineVtxIte < secondIntPlineIdx) {
    pgon2.push_back(pline[(plineVtxIte + 1)]);
    plineVtxIte++;
  }


  //verify that two polygons are created after the split. If not, return false
  /////
  if (pgon1.size() < 3 || pgon2.size() < 3) {
    //std::cout << "Invalid split - Resulting polygons have fewer than three vertices.\n";
    return false;
  }

  return polylineIntersectsPolygon;
}

/**
* @brief: Reorient polygon faces so that they are CCW
* @in: If only check is true, the polygon is not modified
* @out: True if polygon had to be reoriented
**/
bool Polygon3D::reorientFace(Loop3D &pface, bool onlyCheck) {
  int pfaceSz = pface.size();
  int next;
  float tmpSum = 0.0f;

  for (int i = 0; i < pfaceSz; ++i) {
    next = (i + 1) % pfaceSz;
    tmpSum = tmpSum + (pface[next].x() - pface[i].x()) * (pface[next].y() +
             pface[i].y());
  }

  if (tmpSum > 0.0f) {
    if (!onlyCheck) {
      std::reverse(pface.begin(), pface.end());
    }

    return true;
  }

  return false;
}


/**
* @brief: Given a polygon, this function computes the polygon's inwards offset. The offset distance
* is not assumed to be constant and must be specified in the vector offsetDistances. The size of this
* vector must be equal to the number of vertices of the polygon.
* Note that the i-th polygon segment is defined by vertices i and i+1.
* The polygon vertices are assumed to be oriented clockwise
* @param[in] offsetDistances: Perpendicular distance from offset segment i to polygon segment i.
* @param[out] pgonInset: The vertices of the polygon inset
* @return insetArea: Returns the area of the polygon inset
**/
float Polygon3D::computeInset(float offsetDistance, Loop3D &pgonInset,
                              bool computeArea) {
  if (contour.size() < 3) {
    return 0.0f;
  }

  std::vector<float> offsetDistances(contour.size(), offsetDistance);

  return computeInset(offsetDistances, pgonInset, computeArea);
}

}
}
