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

/************************************************************************************************
*
*		MTC Project - Misc tools
*
*
*		@desc Class containing miscellaneous tools
*          useful to more than one project
*		@author cvanegas
*
************************************************************************************************/
#ifndef MTC_MISCTOOLS_H
#define MTC_MISCTOOLS_H

#ifndef BOOST_TYPEOF_SILENT
#define BOOST_TYPEOF_SILENT
#endif//BOOST_TYPEOF_SILENT

#include "common.h"
#include "global.h"

#include "polygon_3D.h"
#include "bounding_box.h"
#include <algorithm>

namespace LC {
namespace misctools {

#ifdef B18_RUN_WITH_GUI
/**
	drawCone Funtion to draw a cone
	d--> axis vector of the cone (direction)
	a--> apex (end of cone)
	h--> height
	r--> radious
	n--> number of divisions
**/
void drawCone(const QVector3D &d, const QVector3D &a,
	      const qreal h, const qreal rd, const int n);

//********************
// Rendering of geometric primitives
//********************

/**
* Render circle parallel to XY plane, centered in X, Y, Z, and with radius r
**/
void renderCircle(float x, float y, float z, float radius);
void renderPolyline(Loop3D &loopIn, bool closed);
#endif

bool loadPolygonsFromFile(QString filename, std::vector<Polygon3D> &polygons);

bool loadPolygonFromFile(QString filename, Polygon3D &polygon);

//bool isIdxWithinBMatrix(int r, int c, LC::misctools::tBoostMatrix &m);

float deg2rad(float deg);

float rad2deg(float rad);

//double sumElementsinRow(int r, LC::misctools::tBoostMatrix &m);

//double sumElementsinColumn(int c, LC::misctools::tBoostMatrix &m);

//void printBMatrix(LC::misctools::tBoostMatrix &m);

//void initBMatrix(LC::misctools::tBoostMatrix &m);

static const float MTC_FLOAT_TOL = 1e-6f;

static const int MAX_PARCEL_ID = 1000000;

static const float MEAN_BUILDING_FLOOR_HEIGHT = 3.0f;

float getClosestPointInArray(std::vector<QVector3D> &pointArray,
                             QVector3D &point, int &idx);
float getClosestPointInArrayXY(std::vector<QVector3D> &pointArray,
                               QVector3D &point, int &idx);

QVector3D calculateNormal(QVector3D &p0, QVector3D &p1, QVector3D &p2);

float pointSegmentDistanceXY(QVector3D &a, QVector3D &b, QVector3D &c,
                             QVector3D &closestPtInAB);

float pointSegmentDistanceXY(QVector3D &s0, QVector3D &s1, QVector3D &pt);

bool segmentSegmentIntersectXY(QVector2D &a, QVector2D &b, QVector2D &c,
                               QVector2D &d,
                               float *tab, float *tcd, bool segmentOnly, QVector2D &intPoint);

bool rayTriangleIntersect(QVector3D &rayPivot, QVector3D &rayDirection,
                          QVector3D &p0, QVector3D &p1, QVector3D &p2);

bool rayTriangleIntersect(QVector3D &rayPivot, QVector3D &rayDirection,
                          QVector3D &p0, QVector3D &p1, QVector3D &p2, QVector3D &intPt);

float angleThreePoints(QVector3D &pa, QVector3D &pb, QVector3D &pc);

QVector3D getColorFromIdx(int i);

int planeIntersectWithLine(QVector3D &p1, QVector3D &p2, QVector3D &n,
                           QVector3D &p0, double &t, QVector3D &x);

unsigned long mix(unsigned long a, unsigned long b, unsigned long c);

template <class T>
T getVectorAverage(std::vector<T> &vec) {
  if (vec.size() == 0) {
    return 0;
  }

  T _sum = 0;

  for (int i = 0; i < vec.size(); ++i) {
    _sum += (vec.at(i));
  }

  return (_sum / vec.size());
}

template <class T>
T getVectorVariance(std::vector<T> &vec, T mean) {
  if (vec.size() == 0) {
    return 0;
  }

  T _sum = 0;
  T _dif;

  for (int i = 0; i < vec.size(); ++i) {
    _dif = vec.at(i) - mean;
    _sum += (_dif * _dif);
  }

  return (_sum / vec.size());
}

template <class T>
void printVector(std::vector<T> &vec, QTextStream &stream) {
  for (int i = 0; i < vec.size(); ++i) {
    stream << vec.at(i) << " ";
  }
}

template <class T>
void printVector(std::vector<T> &vec) {
  printVector(vec, QTextStream(stdout));
}

/**
* Convert Colors
**/
//cs.rit.edu/~ncs/color/t_convert.html
// r,g,b values are from 0 to 1
// h = [0,360], s = [0,1], v = [0,1]
//		if s == 0, then h = -1 (undefined)
void HSVtoRGB(float *r, float *g, float *b, float h, float s, float v);

//********************
// Random numbers.
//********************
float genRand();
float genRand(float a, float b);
float genRandNormal(float mean, float variance);


//*****************
// Color maps
//*****************
//Colormap flags
enum { LINEAR_SCL = 0, SQRT_SCL, LOG_SCL };

void colorMapRainbowGet(float value, float minVal, float maxVal,
                        float &r, float &g, float &b, bool invert, int flag);
void colorMapHotGet(float value, float min, float max,
                    float &r, float &g, float &b, bool invert, int flag);
void colorMapJetGet(float value, float min, float max,
                    float &r, float &g, float &b, bool invert, int flag);
void colorMapColdGet(float value, float min, float max,
                     float &r, float &g, float &b, bool invert, int flag);
void colorMapHotPathGet(float value, float min, float max,
                        float &r, float &g, float &b, bool invert, int flag);
void colorMapYiGnBuGet(float value, float min, float max,
                       float &r, float &g, float &b, bool invert, int flag);
void colorMapTerrainGet(float value, float min, float max,
                        float &r, float &g, float &b, bool isPark, int flag);
void colorMapGrayscaleGet(float value, float min, float max,
                          float &r, float &g, float &b, int flag);
void colorMapGoogleMapsGet(float pop, float elev, float &r, float &g, float &b);

}
}


// We can conveniently use macro's to register point and ring
//BOOST_GEOMETRY_REGISTER_POINT_3D_GET_SET (QVector3D, double, boost::geometry::cs::cartesian, x, y, z, setX, setY, setZ)


BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(Vector3D, double,
    boost::geometry::cs::cartesian, x, y, setX, setY)
BOOST_GEOMETRY_REGISTER_RING(LC::misctools::Loop3D)



// There is currently no registration macro for polygons
// and besides that a boost::array<T,N> in a macro would
// be very specific, so we show it "by hand":
namespace boost {
namespace geometry {
namespace traits {

template<> struct tag<LC::misctools::Polygon3D> {
  typedef polygon_tag type;
};
//template<> struct ring_type<LC::misctools::Polygon3D> { typedef LC::misctools::Loop3D type; };

}
}
} // namespace boost::geometry::traits

namespace boost {
namespace geometry {
template<> struct ring_type<LC::misctools::Polygon3D> {
  typedef LC::misctools::Loop3D type;
};

}
} // namespace boost::geometry



#endif // MTC_MISCTOOLS_H
