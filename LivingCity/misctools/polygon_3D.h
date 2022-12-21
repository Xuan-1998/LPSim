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

#pragma once
/************************************************************************************************
*
*		Polygon 3D
*
*
*		@desc Code of a polygon and some operations using boost
*		@author igarciad
*
************************************************************************************************/
#pragma once

#include <vector>
#include "qvector3d.h"
#ifndef Q_MOC_RUN
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/multi/multi.hpp>
#include <boost/polygon/polygon.hpp>
#endif

class Vector3D : public QVector3D {
 public:
  Vector3D(void) {
  }

  Vector3D(const QVector3D &v)
    : QVector3D(v) {
  }

  Vector3D(const QVector2D &v)
    : QVector3D(v) {
  }

  operator boost::geometry::model::d2::point_xy<double>() const {
    boost::geometry::model::d2::point_xy<double> p;
    p.x(this->x());
    p.y(this->y());
    return p;
  }
};


namespace LC {
namespace misctools {

/**
* Geometry.
* Classes and functions for geometric data
**/


struct Loop3D : std::vector<Vector3D> {
};

bool isPointWithinLoop(std::vector<QVector3D> &loop, QVector3D &pt);

//bool isPointWithinLoop(boost::geometry::ring_type<LC::misctools::Polygon3D>::type &bg_loop, QVector3D &pt);

/**
* Stores a polygon in 3D represented by its
*     exterior contour.
**/
class Polygon3D {
 public:
  /**
  * Constructor.
  **/
  Polygon3D() {
    normalVec = QVector3D(0.0f, 0.0f, 0.0f);
    centroid = QVector3D(FLT_MAX, FLT_MAX, FLT_MAX);
  }

  /**
  * Destructor.
  **/
  ~Polygon3D() {
    contour.clear();
  }

  /**
  * Copy constructor.
  **/
  Polygon3D(const Polygon3D &ref) {
    contour = ref.contour;
    normalVec = ref.normalVec;
    centroid = ref.centroid;
  }

  /**
  * Assignment operator.
  **/
  inline Polygon3D &operator=(const Polygon3D &ref) {
    contour = ref.contour;
    normalVec = ref.normalVec;
    centroid = ref.centroid;
    return (*this);
  }

  /**
  * Acessor to point at index idx
  **/
  inline QVector3D &operator[](const int idx) {
    return contour.at(idx);
  }

  /**
  * Get normal vector
  **/
  QVector3D getNormalVector(void);

  /**
  * Get center of vertices
  **/
  QVector3D getCentroid(void) {
    if (centroid.x() != FLT_MAX) {
      return centroid;
    } else {
      QVector3D newCentroid(0.0f, 0.0f, 0.0f);
      int cSz = contour.size();

      for (int i = 0; i < cSz; ++i) {
        newCentroid = newCentroid + contour[i];
      }

      if (cSz > 0) {
        centroid = (newCentroid / cSz);
      }

      return centroid;
    }
  }


  inline void getBBox(QVector3D &ptMin, QVector3D &ptMax) {
    ptMin.setX(FLT_MAX);
    ptMin.setY(FLT_MAX);
    ptMin.setZ(FLT_MAX);
    ptMax.setX(-FLT_MAX);
    ptMax.setY(-FLT_MAX);
    ptMax.setZ(-FLT_MAX);

    for (size_t i = 0; i < contour.size(); ++i) {
      if (contour[i].x() < ptMin.x()) {
        ptMin.setX(contour[i].x());
      }

      if (contour[i].y() < ptMin.y()) {
        ptMin.setY(contour[i].y());
      }

      if (contour[i].z() < ptMin.z()) {
        ptMin.setZ(contour[i].z());
      }

      if (contour[i].x() > ptMax.x()) {
        ptMax.setX(contour[i].x());
      }

      if (contour[i].y() > ptMax.y()) {
        ptMax.setY(contour[i].y());
      }

      if (contour[i].z() > ptMax.z()) {
        ptMax.setZ(contour[i].z());
      }
    }
  }

  inline float getMeanZValue(void) {
    float zVal = 0.0f;

    if (this->contour.size() > 0) {
      for (size_t i = 0; i < contour.size(); ++i) {
        zVal += (contour[i].z());
      }

      return (zVal / ((float)contour.size()));
    } else {
      return zVal;
    }
  }

  //Is self intersecting
  bool isSelfIntersecting(void);

  //Only works for polygons with no holes in them
  bool splitMeWithPolyline(std::vector<QVector3D> &pline, Loop3D &pgon1,
                           Loop3D &pgon2);

  //Only works for polygons with no holes in them
  float computeInset(float offsetDistance, Loop3D &pgonInset,
                     bool computeArea = true);
  float computeInset(std::vector<float> &offsetDistances, Loop3D &pgonInset,
                     bool computeArea = true);

  float computeArea(bool parallelToXY = false);

  /**
  * Vector containing 3D points of polygon contour
  **/
  Loop3D contour;

  static QVector3D getLoopNormalVector(Loop3D &pin);

  static bool reorientFace(Loop3D &pface, bool onlyCheck = false);

  static int cleanLoop(Loop3D &pin, Loop3D &pout, float threshold);

  static void transformLoop(Loop3D &pin, Loop3D &pout, QMatrix4x4 &transformMat);

  static float computeLoopArea(Loop3D &pin, bool parallelToXY = false);

  static void sampleTriangularLoopInterior(Loop3D &pin,
      std::vector<QVector3D> &pts, float density);

  static QVector3D getLoopAABB(Loop3D &pin, QVector3D &minCorner,
                               QVector3D &maxCorner);

  static void getLoopOBB(Loop3D &pin, QVector3D &size, QMatrix4x4 &xformMat);

  void getMyOBB(QVector3D &size, QMatrix4x4 &xformMat);

  static void extrudePolygon(LC::misctools::Polygon3D &basePgon, float height,
                             std::vector<LC::misctools::Polygon3D> &pgonExtrusion);

  //Shortest distance from a point to a polygon
  static float distanceXYToPoint(Loop3D &pin, QVector3D &pt);

  //minimum distance from a loop to another loop (this considers the contour only)
  static float distanceXYfromContourAVerticesToContourB(Loop3D &pA, Loop3D &pB);


 private:
  QVector3D normalVec;
  QVector3D centroid;
};

}
}


