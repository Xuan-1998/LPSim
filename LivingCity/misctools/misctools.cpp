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


#include "misctools.h"
#include "qstringlist.h"
#ifndef Q_MOC_RUN
#include <boost/random.hpp>
#endif
#include <qtimer.h>
#include <qfile.h>

namespace LC {
namespace misctools {

//#define MISCTOOLS_TIMER

#define _PI 3.14159265


const QVector3D perp(const QVector3D &v) {
  qreal min = fabs(v.x());
  QVector3D cardinalAxis(1.0, 0.0, 0.0);

  if (fabs(v.y()) < min) {
    min = fabs(v.y());
    cardinalAxis = QVector3D(0.0, 1.0, 0.0);
  }

  if (fabs(v.z()) < min) {
    cardinalAxis = QVector3D(0.0, 0.0, 1.0);
  }

  return QVector3D::crossProduct(v, cardinalAxis);
}//

/**
	drawCone Funtion to draw a cone
	d--> axis vector of the cone (direction)
	a--> apex (end of cone)
	h--> height
	r--> radious
	n--> number of divisions
**/
#ifdef B18_RUN_WITH_GUI
void drawCone(const QVector3D &d, const QVector3D &a,
              const qreal h, const qreal rd, const int n) {
  QVector3D c = a + (-d * h);
  QVector3D e0 = perp(d);
  QVector3D e1 = QVector3D::crossProduct(e0, d);
  qreal angInc = 360.0 / n * (M_PI / 180.0f); //M_PI_DIV180;

  // draw cone top
  glBegin(GL_TRIANGLE_FAN);
  glVertex3f(a.x(), a.y(), a.z());

  for (int i = 0; i <= n; i++) {
    qreal rad = angInc * i;
    QVector3D p = c + (((e0 * cos(rad)) + (e1 * sin(rad))) * rd);
    glVertex3f(p.x(), p.y(), p.z());
  }

  glEnd();

  // draw cone bottom
  glBegin(GL_TRIANGLE_FAN);
  glVertex3f(c.x(), c.y(), c.z());

  for (int i = n - 1; i >= 0; i--) {
    qreal rad = angInc * i;
    QVector3D p = c + (((e0 * cos(rad)) + (e1 * sin(rad))) * rd);
    glVertex3f(p.x(), p.y(), p.z());
  }

  glEnd();
}//
#endif
float deg2rad(float deg) {
  return ((deg * _PI) / 180.0f);
}

float rad2deg(float rad) {
  return ((rad * 180.0f) / _PI);
}


//bool isIdxWithinBMatrix(int r, int c, tBoostMatrix &m)
//{
//	return (r < m.size1() && c < m.size2() && r>=0 && c>=0);
//}

//double sumElementsinRow(int r, LC::misctools::tBoostMatrix &m)
//{
//	assert(r < m.size1());
//	double sum = 0.0f;

//	for(int i=0; i<m.size2(); ++i){
//		sum += (m(r,i));
//	}
//	return sum;
//}

//double sumElementsinColumn(int c, LC::misctools::tBoostMatrix &m)
//{
//	assert(c < m.size2());
//	double sum = 0.0f;

//	for(int i=0; i<m.size1(); ++i){
//		sum += (m(i,c));
//	}
//	return sum;
//}

//void printBMatrix(tBoostMatrix &m)
//{
//	for(int i=0; i<m.size1(); ++i){
//		for(int j=0; j<m.size2(); ++j){
//			std::cout << m(i,j) << " ";
//		}
//		std::cout << "\n";
//	}
//	std::cout << "\n";
//}

void printQMatrix4x4(QMatrix4x4 &m) {
  for (int i = 0; i < 4; ++i) {
    std::cout << "\n";

    for (int j = 0; j < 4; ++j) {
      std::cout << m(i, j) << "\t";
    }
  }

  std::cout << "\n";
}

//void initBMatrix(tBoostMatrix &m)
//{
//	for(int i=0; i<m.size1(); ++i){
//		for(int j=0; j<m.size2(); ++j){
//			m(i,j) = 0;
//		}
//	}
//}


//********************
// Geometry.
// Classes and functions for geometric data
//********************

/**
* Returns index of the point in pointArray that is closest to point
**/
float getClosestPointInArray(std::vector<QVector3D> &pointArray,
                             QVector3D &point, int &idx) {
  idx = -1;
  float minDist = FLT_MAX;
  float curDist;

  for (int i = 0; i < pointArray.size(); ++i) {
    curDist = (point - pointArray[i]).lengthSquared();

    if (curDist < minDist) {
      minDist = curDist;
      idx = i;
    }
  }

  return sqrt(minDist);
}

float getClosestPointInArrayXY(std::vector<QVector3D> &pointArray,
                               QVector3D &point, int &idx) {
  idx = -1;
  float minDist = FLT_MAX;
  float curDist;

  for (int i = 0; i < pointArray.size(); ++i) {
    curDist = (point.x() - pointArray[i].x()) * (point.x() - pointArray[i].x()) +
              (point.y() - pointArray[i].y()) * (point.y() - pointArray[i].y());

    if (curDist < minDist) {
      minDist = curDist;
      idx = i;
    }
  }

  return sqrt(minDist);
}

QVector3D calculateNormal(QVector3D &p0, QVector3D &p1, QVector3D &p2) {
  return (QVector3D::normal((p1 - p0), (p2 - p1)));
}

bool isPointWithinLoop(std::vector<QVector3D> &loop, QVector3D &pt) {
  /*boost::geometry::ring_2d bg_loop;
  boost::geometry::point_2d bg_testPt;

  boost::geometry::assign(bg_loop, loop);
  boost::geometry::correct(bg_loop);
  bg_testPt.x(pt.x());
  bg_testPt.y(pt.y());

  return boost::geometry::within(bg_testPt, bg_loop);*/

  int i, j, c = 0;

  for (i = 0, j = loop.size() - 1; i < loop.size(); j = i++) {
    if (((loop[i].y() > pt.y()) != (loop[j].y() > pt.y())) &&
        (pt.x() < (loop[j].x() - loop[i].x()) * (pt.y() - loop[i].y()) /
         (loop[j].y() - loop[i].y()) + loop[i].x())) {
      c = !c;
    }
  }

  return c;

}

/*bool isPointWithinLoop(boost::geometry::ring_type<LC::misctools::Polygon3D>::type &bg_loop, QVector3D &pt)
{
	boost::geometry::ring_type<LC::misctools::Polygon3D>::type bg_testPt;

	bg_testPt.x(pt.x());
	bg_testPt.y(pt.y());

	return boost::geometry::within(bg_testPt, bg_loop);
}*/

//Distance from segment ab to point c
float pointSegmentDistanceXY(QVector3D &a, QVector3D &b, QVector3D &c,
                             QVector3D &closestPtInAB) {
  float dist;

  float r_numerator = (c.x() - a.x()) * (b.x() - a.x()) + (c.y() - a.y()) *
                      (b.y() - a.y());
  float r_denomenator = (b.x() - a.x()) * (b.x() - a.x()) + (b.y() - a.y()) *
                        (b.y() - a.y());
  float r = r_numerator / r_denomenator;
  //
  float px = a.x() + r * (b.x() - a.x());
  float py = a.y() + r * (b.y() - a.y());
  //
  float s = ((a.y() - c.y()) * (b.x() - a.x()) - (a.x() - c.x()) *
             (b.y() - a.y())) / r_denomenator;

  float distanceLine = fabs(s) * sqrt(r_denomenator);

  //
  // (xx,yy) is the point on the lineSegment closest to (cx,cy)
  //
  closestPtInAB.setX(px);
  closestPtInAB.setY(py);
  closestPtInAB.setZ(0.0f);

  if ((r >= 0) && (r <= 1)) {
    dist = distanceLine;
  } else {
    float dist1 = (c.x() - a.x()) * (c.x() - a.x()) + (c.y() - a.y()) *
                  (c.y() - a.y());
    float dist2 = (c.x() - b.x()) * (c.x() - b.x()) + (c.y() - b.y()) *
                  (c.y() - b.y());

    if (dist1 < dist2) {
      dist = sqrt(dist1);
    } else {
      dist = sqrt(dist2);
    }
  }

  return abs(dist);
}


float pointSegmentDistanceXY(QVector3D &a, QVector3D &b, QVector3D &c) {
  QVector3D closestPt;
  return pointSegmentDistanceXY(a, b, c, closestPt);

}

/**
* Computes the intersection between two line segments on the XY plane
* Segments must intersect within their extents for the intersection to be valid
* z coordinate is ignored
**/
bool segmentSegmentIntersectXY(QVector2D &a, QVector2D &b, QVector2D &c,
                               QVector2D &d,
                               float *tab, float *tcd, bool segmentOnly, QVector2D &intPoint) {
  QVector2D u = b - a;
  QVector2D v = d - c;

  if (u.lengthSquared() < MTC_FLOAT_TOL  ||  v.lengthSquared() < MTC_FLOAT_TOL) {
    return false;
  }

  float numer = v.x() * (c.y() - a.y()) + v.y() * (a.x() - c.x());
  float denom = u.y() * v.x() - u.x() * v.y();

  if (denom == 0.0f)  {
    // they are parallel
    *tab = 0.0f;
    *tcd = 0.0f;
    return false;
  }

  float t0 = numer / denom;

  QVector2D ipt = a + t0 * u;
  QVector2D tmp = ipt - c;
  float t1;

  if (QVector2D::dotProduct(tmp, v) > 0.0f) {
    t1 = tmp.length() / v.length();
  } else {
    t1 = -1.0f * tmp.length() / v.length();
  }

  //Check if intersection is within segments
  if (!((t0 >= MTC_FLOAT_TOL) && (t0 <= 1.0f - MTC_FLOAT_TOL) &&
        (t1 >= MTC_FLOAT_TOL) && (t1 <= 1.0f - MTC_FLOAT_TOL))) {
    return false;
  }

  *tab = t0;
  *tcd = t1;
  QVector2D dirVec = b - a;

  intPoint = a + (*tab) * dirVec;
  return true;
}


bool rayTriangleIntersect(QVector3D &rayPivot, QVector3D &rayDirection,
                          QVector3D &p0, QVector3D &p1, QVector3D &p2, QVector3D &intPt) {
  //int rayIntersectsTriangle(float *p, float *d, float *v0, float *v1, float *v2) {

  QVector3D e1, e2, h, s, q;
  float a, f, u, v;

  //vector(e1,v1,v0);
  //vector(e2,v2,v0);
  e1 = p1 - p0;
  e2 = p2 - p0;

  //crossProduct(h,d,e2);
  h = QVector3D::crossProduct(rayDirection, e2);
  //a = innerProduct(e1,h);
  a = QVector3D::dotProduct(e1, h);

  if (a > -0.00001 && a < 0.00001) {
    return (false);
  }

  f = 1 / a;

  //vector(s,p,v0);
  s = rayPivot - p0;

  //u = f * (innerProduct(s,h));
  u = f * (QVector3D::dotProduct(s, h));

  if (u < 0.0 || u > 1.0) {
    return (false);
  }

  //crossProduct(q,s,e1);
  q = QVector3D::crossProduct(s, e1);
  //v = f * innerProduct(d,q);
  v = f * QVector3D::dotProduct(rayDirection, q);

  if (v < 0.0 || u + v > 1.0) {
    return (false);
  }

  // at this stage we can compute t to find out where
  // the intersection point is on the line
  //t = f * innerProduct(e2,q);
  float t = f * QVector3D::dotProduct(e2, q);

  if (t > 0.00001) { // ray intersection
    return (true);
  }

  else // this means that there is a line intersection
    // but not a ray intersection
  {
    return (false);
  }
}

bool rayTriangleIntersect(QVector3D &rayPivot, QVector3D &rayDirection,
                          QVector3D &p0, QVector3D &p1, QVector3D &p2) {
  QVector3D intPt;
  return rayTriangleIntersect(rayPivot, rayDirection, p0, p1, p2, intPt);
}

/**
* Angle between 3 points A-B-C
**/
float angleThreePoints(QVector3D &pa, QVector3D &pb, QVector3D &pc) {
  float a = (pb - pc).length();
  float b = (pa - pc).length();
  float c = (pa - pb).length();
  return acos(0.999f * (a * a + c * c - b * b) / (2.0f * a * c));
}

QVector3D getColorFromIdx(int i) {
  QVector3D colorOut;
  colorOut.setX(((float)((i * 300) % 255)) / 255.0f);
  colorOut.setY(((float)((i * 400) % 255)) / 255.0f);
  colorOut.setZ(((float)((i * 500)  % 255)) / 255.0f);

  return colorOut;
}

int planeIntersectWithLine(QVector3D &p1, QVector3D &p2, QVector3D &n,
                           QVector3D &p0, double &t, QVector3D &x) {
  double num, den, p21[3];
  double fabsden, fabstolerance;

  // Compute line vector
  p21[0] = p2.x() - p1.x();
  p21[1] = p2.y() - p1.y();
  p21[2] = p2.z() - p1.z();

  // Compute denominator.  If ~0, line and plane are parallel.
  num = QVector3D::dotProduct(n,
                              p0) - (n.x() * p1.x() + n.y() * p1.y() + n.z() * p1.z()) ;
  den = n.x() * p21[0] + n.y() * p21[1] + n.z() * p21[2];

  // If denominator with respect to numerator is "zero", then the line and
  // plane are considered parallel.

  // trying to avoid an expensive call to fabs()
  if (den < 0.0) {
    fabsden = -den;
  } else {
    fabsden = den;
  }

  if (num < 0.0) {
    fabstolerance = -num * 1.0e-06;
  } else {
    fabstolerance = num * 1.0e-06;
  }

  if (fabsden <= fabstolerance) {
    t = DBL_MAX;
    return 0;
  }

  // valid intersection
  t = num / den;

  x.setX(p1.x() + t * p21[0]);
  x.setY(p1.y() + t * p21[1]);
  x.setZ(p1.z() + t * p21[2]);

  if (t >= 0.0 && t <= 1.0) {
    return 1;
  } else {
    return 0;
  }
}//


double angleBetweenVectors(QVector3D &vec1, QVector3D &vec2) {
  return acos(0.999 * (QVector3D::dotProduct(vec1,
                       vec2)) / (vec1.length() * vec2.length()));
}


/**
* Given three non colinear points p0, p1, p2, this function computes
* the intersection between the lines A and B. Line A is the line parallel to the segment p0-p1
* and at a distance d01 from that segment. Line B is the line parallel to the segment
* p1-p2 at a distance d12 from that segment.
* Returns true if point is successfully computed
**/

bool getIrregularBisector(QVector3D &p0,
                          QVector3D &p1, QVector3D &p2, float d01, float d12,
                          QVector3D &intPt) {
  double alpha;
  double theta;
  double L;

  QVector3D p1_p0;
  QVector3D p1_p2;
  QVector3D p1_p2_perp;
  QVector3D crossP;

  p1_p0 = p0 - p1;
  p1_p0.setZ(0.0f);

  p1_p2 = p2 - p1;
  p1_p2.setZ(0.0f);

  p1_p2_perp.setX(-p1_p2.y());
  p1_p2_perp.setY(p1_p2.x());
  p1_p2_perp.setZ(0.0f);

  alpha = angleBetweenVectors(p1_p0, p1_p2);

  if (!(alpha == alpha)) {
    return false;
  }

  theta = atan2(sin(alpha), (d01 / d12) + cos(alpha));
  L = d12 / sin(theta);

  //This is done to handle convex vs. concave angles in polygon
  crossP = QVector3D::crossProduct(p1_p2, p1_p0);

  if (crossP.z() > 0) {
    //CCW polygon (A + B + C)
    //CW  polygon (A - B - C)
    intPt = p1 + (p1_p2.normalized()) * L * cos(theta) +
            (p1_p2_perp.normalized()) * d12;
  } else {
    //CCW polygon (A - B + C)
    //CW  polygon (A + B - C)
    intPt = p1 - (p1_p2.normalized()) * L * cos(theta) +
            (p1_p2_perp.normalized()) * d12;
  }

  return true;
}

/**
* Checks if contour A is within contour B
**/
bool is2DRingWithin2DRing(
  boost::geometry::ring_type<LC::misctools::Polygon3D>::type &contourA,
  boost::geometry::ring_type<LC::misctools::Polygon3D>::type &contourB) {
  for (int i = 0; i < contourA.size(); ++i) {
    if (!boost::geometry::within(contourA[i], contourB)) {
      return false;
    }
  }

  return true;
}


/**
* @brief: Merge consecutive vertices that are within a distance threshold to each other
**/
int Polygon3D::cleanLoop(Loop3D &pin, Loop3D &pout, float threshold = 1.0f) {
  float thresholdSq = threshold * threshold;

  if (pin.size() < 3) {
    return 1;
  }

  boost::geometry::ring_type<LC::misctools::Polygon3D>::type bg_pin;
  boost::geometry::ring_type<LC::misctools::Polygon3D>::type bg_pout;
  boost::geometry::assign(bg_pin, pin);
  boost::geometry::correct(bg_pin);
  boost::geometry::simplify(bg_pin, bg_pout, threshold);

  //strategy::simplify::douglas_peucker

  //copy points back
  QVector3D tmpPt;

  for (size_t i = 0; i < bg_pout.size(); ++i) {
    tmpPt.setX(bg_pout[i].x());
    tmpPt.setY(bg_pout[i].y());
    pout.push_back(tmpPt);
  }

  //remove last point
  if ((pout[0] - pout[pout.size() - 1]).lengthSquared() < thresholdSq) {
    pout.pop_back();
  }

  //clean angles
  int next, prev;
  QVector3D cur_prev, cur_next;
  float ang;
  float angleThreshold = 0.01f;

  for (size_t i = 0; i < pout.size(); ++i) {
    next = (i + 1) % pout.size();
    prev = (i - 1 + pout.size()) % pout.size();
    cur_prev = pout[prev] - pout[i];
    cur_next = pout[next] - pout[i];

    ang = angleBetweenVectors(cur_prev, cur_next);

    if ((fabs(ang) < angleThreshold)
        || (fabs(ang - _PI) < angleThreshold)
        || (!(ang == ang))) {
      //std::cout << ang << " ";
      pout.erase(pout.begin() + i);
      --i;
    }
  }


  return 0;
}

float Polygon3D::computeInset(std::vector<float> &offsetDistances,
                              Loop3D &pgonInset, bool computeArea) {
  Loop3D cleanPgon;
  double tol = 0.01f;

  cleanPgon = this->contour;

  int prev, next;
  int cSz = cleanPgon.size();

  if (cSz < 3) {
    return 0.0f;
  }

  if (reorientFace(cleanPgon)) {
    std::reverse(offsetDistances.begin(), offsetDistances.end() - 1);
  }

  //if offsets are zero, add a small epsilon just to avoid division by zero
  for (size_t i = 0; i < offsetDistances.size(); ++i) {
    if (fabs(offsetDistances[i]) < tol) {
      offsetDistances[i] = tol;
    }
  }

  pgonInset.resize(cSz);

  QVector3D intPt;


  for (int cur = 0; cur < cSz; ++cur) {
    //Some geometry and trigonometry

    //point p1 is the point with index cur
    prev = (cur - 1 + cSz) % cSz; //point p0
    next = (cur + 1) % cSz;	 //point p2

    getIrregularBisector(cleanPgon[prev], cleanPgon[cur], cleanPgon[next],
                         offsetDistances[prev], offsetDistances[cur], intPt);

    pgonInset[cur] = intPt;
  }

  //temp
  //pgonInset = cleanPgon;

  //Compute inset area
  if (computeArea) {

    boost::geometry::ring_type<LC::misctools::Polygon3D>::type bg_contour;
    boost::geometry::ring_type<LC::misctools::Polygon3D>::type bg_contour_inset;
    float contArea;
    float contInsetArea;

    if (pgonInset.size() > 0) {
      boost::geometry::assign(bg_contour_inset, pgonInset);
      boost::geometry::correct(bg_contour_inset);

      if (boost::geometry::intersects(bg_contour_inset)) {
        pgonInset.clear();
        return 0.0f;
      } else {

        boost::geometry::assign(bg_contour, cleanPgon);
        boost::geometry::correct(bg_contour);

        //if inset is not within polygon
        if (!is2DRingWithin2DRing(bg_contour_inset, bg_contour)) {
          pgonInset.clear();
          return 0.0f;
        } else {
          contArea = fabs(boost::geometry::area(bg_contour));
          contInsetArea = fabs(boost::geometry::area(bg_contour_inset));

          if (contInsetArea < contArea) {
            //return boost::geometry::area(bg_contour_inset);
            return contInsetArea;
          } else {
            pgonInset.clear();
            return 0.0f;
          }
        }
      }
    } else {
      pgonInset.clear();
      return 0.0f;
    }
  }

  return 0.0f;

}

bool Polygon3D::isSelfIntersecting(void) {
  boost::geometry::ring_type<LC::misctools::Polygon3D>::type bg_pgon;
  boost::geometry::assign(bg_pgon, this->contour);
  boost::geometry::correct(bg_pgon);
  return boost::geometry::intersects(bg_pgon);
}

//===================================================================
// area3D_Polygon(): computes the area of a 3D planar polygon
//    Input:  int n = the number of vertices in the polygon
//            Point* V = an array of n+2 vertices in a plane
//                       with V[n]=V[0] and V[n+1]=V[1]
//            Point N = unit normal vector of the polygon's plane
//    Return: the (float) area of the polygon
//===================================================================
//float area3D_Polygon( int n, Point* V, Point N )
float area3D_Polygon(Loop3D &pin) {
  int n = pin.size();

  if (n < 3) {
    return 0.0f;
  }

  QVector3D normVec = Polygon3D::getLoopNormalVector(pin);

  float area = 0.0f;
  float an, ax, ay, az;  // abs value of normal and its coords
  int   coord;           // coord to ignore: 1=x, 2=y, 3=z
  int   i, j, k;         // loop indices

  // select largest abs coordinate to ignore for projection
  ax = (normVec.x() > 0 ? normVec.x() : -normVec.x());   // abs x-coord
  ay = (normVec.y() > 0 ? normVec.y() : -normVec.y());   // abs y-coord
  az = (normVec.z() > 0 ? normVec.z() : -normVec.z());   // abs z-coord

  coord = 3;                     // ignore z-coord

  if (ax > ay) {
    if (ax > az) {
      coord = 1;  // ignore x-coord
    }
  } else if (ay > az) {
    coord = 2;  // ignore y-coord
  }

  // compute area of the 2D projection
  //for (i=1, j=2, k=0; i<=n; i++, j++, k++)
  for (k = 0; k < n; ++k) {
    i = (k + 1) % n;
    j = (k + 2) % n;

    switch (coord) {
    case 1:
      area += (pin[i].y() * (pin[j].z() - pin[k].z()));
      continue;

    case 2:
      area += (pin[i].x() * (pin[j].z() - pin[k].z()));
      continue;

    case 3:
      area += (pin[i].x() * (pin[j].y() - pin[k].y()));
      continue;
    }
  }

  // scale to get area before projection
  an = sqrt(ax * ax + ay * ay + az * az); // length of normal vector

  switch (coord) {
  case 1:
    area *= (an / (2 * ax));
    break;

  case 2:
    area *= (an / (2 * ay));
    break;

  case 3:
    area *= (an / (2 * az));
  }

  return fabs(area);
}
//===================================================================


float Polygon3D::computeLoopArea(Loop3D &pin, bool parallelToXY) {
  float _area = 0.0f;

  //if polygon is parallel to XY plane
  if (parallelToXY) {
    boost::geometry::ring_type<LC::misctools::Polygon3D>::type bg_pgon;
    boost::geometry::assign(bg_pgon, pin);
    _area =  fabs(boost::geometry::area(bg_pgon));
  } else {
    _area = fabs(area3D_Polygon(pin));
  }

  return _area;
}

//density: number of points to be generated per square meter
void Polygon3D::sampleTriangularLoopInterior(Loop3D &pin,
    std::vector<QVector3D> &pts, float density) {
  if (pin.size() == 3) {

    QVector3D tmpPt;
    QVector3D v1_minus_v0;
    QVector3D v2_minus_v0;
    QVector3D crossP;

    v1_minus_v0 = pin.at(1) - pin.at(0);
    v2_minus_v0 = pin.at(2) - pin.at(0);

    //float loopArea = computeLoopArea(pin);
    float loopArea = 0.5f * (QVector3D::crossProduct(v1_minus_v0,
                             v2_minus_v0).length());

    int numSamples = (int)(density * loopArea);

    //std::cout << numSamples << " ";

    float rand1, rand2;

    for (int i = 0; i < numSamples; ++i) {
      rand1 = LC::misctools::genRand();
      rand2 = LC::misctools::genRand();

      if (rand1 + rand2 > 1.0f) {
        rand1 = 1.0 - rand1;
        rand2 = 1.0 - rand2;
      }

      tmpPt = pin.at(0) + rand1 * v1_minus_v0 + rand2 * v2_minus_v0;

      pts.push_back(tmpPt);
    }
  }
}

float Polygon3D::computeArea(bool parallelToXY) {
  return Polygon3D::computeLoopArea(this->contour, parallelToXY);
}

/**
* @brief: Get polygon axis aligned bounding box
* @return: The dimensions of the AABB
**/
QVector3D Polygon3D::getLoopAABB(Loop3D &pin, QVector3D &minCorner,
                                 QVector3D &maxCorner) {
  maxCorner = QVector3D(-FLT_MAX, -FLT_MAX, -FLT_MAX);
  minCorner = QVector3D(FLT_MAX,  FLT_MAX,  FLT_MAX);

  QVector3D curPt;

  for (int i = 0; i < pin.size(); ++i) {
    curPt = pin.at(i);

    if (curPt.x() > maxCorner.x()) {
      maxCorner.setX(curPt.x());
    }

    if (curPt.y() > maxCorner.y()) {
      maxCorner.setY(curPt.y());
    }

    if (curPt.z() > maxCorner.z()) {
      maxCorner.setZ(curPt.z());
    }

    //------------
    if (curPt.x() < minCorner.x()) {
      minCorner.setX(curPt.x());
    }

    if (curPt.y() < minCorner.y()) {
      minCorner.setY(curPt.y());
    }

    if (curPt.z() < minCorner.z()) {
      minCorner.setZ(curPt.z());
    }
  }

  return QVector3D(maxCorner - minCorner);
}

void Polygon3D::transformLoop(Loop3D &pin, Loop3D &pout,
                              QMatrix4x4 &transformMat) {
  pout = pin;

  for (int i = 0; i < pin.size(); ++i) {
    pout.at(i) = transformMat * pin.at(i);
  }
}

/**
* Get polygon oriented bounding box
**/
void Polygon3D::getLoopOBB(Loop3D &pin, QVector3D &size, QMatrix4x4 &xformMat) {
  float alpha = 0.0f;
  float deltaAlpha = 0.05 * _PI;
  float bestAlpha;

  LC::misctools::Loop3D rotLoop;
  QMatrix4x4 rotMat;
  QVector3D minPt, maxPt;
  QVector3D origMidPt;
  QVector3D boxSz;
  QVector3D bestBoxSz;
  float curArea;
  float minArea = FLT_MAX;

  rotLoop = pin;
  Polygon3D::getLoopAABB(rotLoop, minPt, maxPt);
  origMidPt = 0.5f * (minPt + maxPt);

  //while(alpha < 0.5f*_PI){
  int cSz = pin.size();
  QVector3D difVec;

  for (int i = 0; i < pin.size(); ++i) {
    difVec = (pin.at((i + 1) % cSz) - pin.at(i)).normalized();
    alpha = atan2(difVec.x(), difVec.y());
    rotMat.setToIdentity();
    rotMat.rotate(rad2deg(alpha), 0.0f, 0.0f, 1.0f);

    transformLoop(pin, rotLoop, rotMat);
    boxSz = Polygon3D::getLoopAABB(rotLoop, minPt, maxPt);
    curArea = boxSz.x() * boxSz.y();

    if (curArea < minArea) {
      minArea = curArea;
      bestAlpha = alpha;
      bestBoxSz = boxSz;
    }

    //alpha += deltaAlpha;
  }

  xformMat.setToIdentity();
  xformMat.rotate(rad2deg(bestAlpha), 0.0f, 0.0f, 1.0f);
  xformMat.setRow(3, QVector4D(origMidPt.x(), origMidPt.y(), origMidPt.z(),
                               1.0f));
  size = bestBoxSz;
}


void Polygon3D::getMyOBB(QVector3D &size, QMatrix4x4 &xformMat) {
  Polygon3D::getLoopOBB(this->contour, size, xformMat);
}



void Polygon3D::extrudePolygon(LC::misctools::Polygon3D &basePgon, float height,
                               std::vector<LC::misctools::Polygon3D> &pgonExtrusion) {
  QVector3D zTransV(0.0f, 0.0f, height);
  int iNext;
  int pgonContourSz = basePgon.contour.size();

  for (int i = 0; i < pgonContourSz; ++i) {
    iNext = (i + 1) % pgonContourSz;

    //construct face
    LC::misctools::Polygon3D tmpPgon1;
    LC::misctools::Polygon3D tmpPgon2;

    tmpPgon1.contour.reserve(3); //pre allocate capacity for polygon contour
    tmpPgon1.contour.push_back(basePgon.contour[i]);
    tmpPgon1.contour.push_back(basePgon.contour[iNext]);
    tmpPgon1.contour.push_back(basePgon.contour[iNext] + zTransV);

    tmpPgon2.contour.reserve(3); //pre allocate capacity for polygon contour
    tmpPgon2.contour.push_back(basePgon.contour[i]);
    tmpPgon2.contour.push_back(basePgon.contour[iNext] + zTransV);
    tmpPgon2.contour.push_back(basePgon.contour[i]   + zTransV);

    //add two triangular faces to solid
    pgonExtrusion.push_back(tmpPgon1);
    pgonExtrusion.push_back(tmpPgon2);
  }
}


//Shortest distance from a point to a polygon
float Polygon3D::distanceXYToPoint(Loop3D &pin, QVector3D &pt) {
  float minDist = FLT_MAX;
  float dist;
  int idxNext;

  for (size_t i = 0; i < pin.size(); ++i) {
    idxNext = (i + 1) % (pin.size());
    dist = pointSegmentDistanceXY(pin.at(i), pin.at(idxNext), pt);

    if (dist < minDist) {
      minDist = dist;
    }
  }

  return minDist;
}

//this function measures the minimum distance from the vertices of a contour A
//	to the edges of a contour B, i.e., it measures the distances from each vertex of A
//  to all the edges in B, and returns the minimum of such distances
float Polygon3D::distanceXYfromContourAVerticesToContourB(Loop3D &pA,
    Loop3D &pB) {
  float minDist = FLT_MAX;
  float dist;

  for (size_t i = 0; i < pA.size(); ++i) {
    dist = Polygon3D::distanceXYToPoint(pB, pA.at(i));

    if (dist < minDist) {
      minDist = dist;
    }
  }

  return minDist;
}


//********************
// Random numbers.
//********************

float genRand(void) {
  return (rand() / (float(RAND_MAX) + 1));
}

float genRand(float a, float b) {
  return (genRand()) * (b - a) + a;
}

float genRandNormal(float mean, float variance) {

  float m = mean;
  float s = sqrt(variance);

  /* mean m, standard deviation s */
  float x1, x2, w, y1;
  static float y2;
  static int use_last = 0;

  if (use_last) {	        /* use value from previous call */
    y1 = y2;
    use_last = 0;
  } else {
    do {
      x1 = 2.0 * genRand(0.0f, 1.0f) - 1.0;
      x2 = 2.0 * genRand(0.0f, 1.0f) - 1.0;
      w = x1 * x1 + x2 * x2;
    } while (w >= 1.0);

    w = sqrt((-2.0 * log(w)) / w);
    y1 = x1 * w;
    y2 = x2 * w;
    use_last = 1;
  }

  return (m + y1 * s);

}


//********************
// Rendering of geometric primitives
//********************

/**
* Render circle parallel to XY plane, centered in X, Y, Z, and with radius r
**/
#ifdef B18_RUN_WITH_GUI
void renderCircle(float x, float y, float z, float radius) {
  int circle_points = 100;
  float angle;
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  glBegin(GL_POLYGON);

  for (int i = 0; i < circle_points; i++) {
    angle = 2 * _PI * i / circle_points;
    glVertex3f(x + radius * cos(angle), y + radius * sin(angle), z);
  }

  glEnd();

  glLineWidth(1.0);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void renderPolyline(Loop3D &loopIn, bool closed = true) {
  if (closed) {
    glBegin(GL_LINE_LOOP);
  } else {
    glBegin(GL_LINE_STRIP);
  }

  for (int i = 0; i < loopIn.size(); ++i) {
    glVertex3f(loopIn.at(i).x(),
               loopIn.at(i).y(),
               loopIn.at(i).z());
  }

  glEnd();
}
#endif

//http://www.concentric.net/~Ttwang/tech/inthash.htm
unsigned long mix(unsigned long a, unsigned long b, unsigned long c) {
  a = a - b;
  a = a - c;
  a = a ^ (c >> 13);
  b = b - c;
  b = b - a;
  b = b ^ (a << 8);
  c = c - a;
  c = c - b;
  c = c ^ (b >> 13);
  a = a - b;
  a = a - c;
  a = a ^ (c >> 12);
  b = b - c;
  b = b - a;
  b = b ^ (a << 16);
  c = c - a;
  c = c - b;
  c = c ^ (b >> 5);
  a = a - b;
  a = a - c;
  a = a ^ (c >> 3);
  b = b - c;
  b = b - a;
  b = b ^ (a << 10);
  c = c - a;
  c = c - b;
  c = c ^ (b >> 15);
  return c;
}


//********************
// Color maps
//********************

/**
* Convert Colors
**/
//cs.rit.edu/~ncs/color/t_convert.html
// r,g,b values are from 0 to 1
// h = [0,360], s = [0,1], v = [0,1]
//		if s == 0, then h = -1 (undefined)
void HSVtoRGB(float *r, float *g, float *b, float h, float s, float v) {
  int i;
  float f, p, q, t;

  if (s == 0) {
    // achromatic (grey)
    *r = *g = *b = v;
    return;
  }

  h /= 60;			// sector 0 to 5
  i = floor(h);
  f = h - i;			// factorial part of h
  p = v * (1 - s);
  q = v * (1 - s * f);
  t = v * (1 - s * (1 - f));

  switch (i) {
  case 0:
    *r = v;
    *g = t;
    *b = p;
    break;

  case 1:
    *r = q;
    *g = v;
    *b = p;
    break;

  case 2:
    *r = p;
    *g = v;
    *b = t;
    break;

  case 3:
    *r = p;
    *g = q;
    *b = v;
    break;

  case 4:
    *r = t;
    *g = p;
    *b = v;
    break;

  default:		// case 5:
    *r = v;
    *g = p;
    *b = q;
    break;
  }
}

/*****************************************************************************
*****************************************************************************/
void colorMapRainbowGet(float value, float minVal, float maxVal,
                        float &r, float &g, float &b, bool invert, int flag) {
  float rangeVal;
  rangeVal = maxVal - minVal;
  value = value - minVal;

  if (flag != LINEAR_SCL) {
    switch (flag) {
    case SQRT_SCL:
      value = sqrt(value);
      rangeVal = sqrt(rangeVal);
      break;

    case LOG_SCL:
      value = log(value);
      rangeVal = log(rangeVal);
      break;
    }
  }

  value = value / rangeVal;

  if (invert) {
    value = 1.0 - value;
  }

  if (value < 0.0) {
    value = 0.0;
  } else if (value > 1.0) {
    value = 1.0;
  }

  const float dx = 0.8;
  value = (6 - 2 * dx) * value + dx;

  r = std::max<float>(0.0, (3 - fabs(value - 4) - fabs(value - 5)) / 2);
  g = std::max<float>(0.0, (4 - fabs(value - 2) - fabs(value - 4)) / 2);
  b = std::max<float>(0.0, (3 - fabs(value - 1) - fabs(value - 2)) / 2);
}

/*****************************************************************************
*****************************************************************************/
void colorMapHotGet(float value, float min, float max,
                    float &r, float &g, float &b, bool invert, int flag) {
  float max3 = (max - min) / 3.0;
  value -= min;

  if (value == FLT_MAX) {
    r = g = b = 255;
  } else if (value < 0) {
    r = g = b = 0;
  } else if (value < max3) {
    r = (255 * value / max3);
    g = 0;
    b = 0;
  } else if (value < 2 * max3) {
    r = 255;
    g = (255 * (value - max3) / max3);
    b = 0;
  } else if (value < max) {
    r = 255;
    g = 255;
    b = (255 * (value - 2 * max3) / max3);
  } else {
    r = g = b = 255;
  }

  r /= 255.0;
  g /= 255.0;
  b /= 255.0;
}

/*****************************************************************************
*****************************************************************************/
void colorMapJetGet(float value, float min, float max,
                    float &r, float &g, float &b, bool invert, int flag) {
  if (flag != LINEAR_SCL) {
    switch (flag) {
    case SQRT_SCL:
      value = sqrt(value);
      max = sqrt(max);
      break;

    case LOG_SCL:
      value = log(value);
      max = log(max);
      break;
    }
  }


  unsigned char c1 = 144;
  float max4 = (max - min) / 4;
  value -= min;

  if (value == FLT_MAX) {
    r = g = b = 255;
  } else if (value < 0) {
    r = g = b = 0;
  } else if (value < max4) {
    r = 0;
    g = 0;
    b = c1 + (float)((255 - c1) * value / max4);
  } else if (value < 2 * max4) {
    r = 0;
    g = (float)(255 * (value - max4) / max4);
    b = 255;
  } else if (value < 3 * max4) {
    r = (unsigned char)(255 * (value - 2 * max4) / max4);
    g = 255;
    b = 255 - r;
  } else if (value < max) {
    r = 255;
    g = (float)(255 - 255 * (value - 3 * max4) / max4);
    b = 0;
  } else {
    r = 255;
    g = b = 0;
  }

  r /= 255.0;
  g /= 255.0;
  b /= 255.0;

}

/*****************************************************************************
*****************************************************************************/
void colorMapColdGet(float value, float min, float max,
                     float &r, float &g, float &b, bool invert, int flag) {
  float max3 = (max - min) / 3;
  value -= min;

  if (value == FLT_MAX) {
    r = g = b = 255;
  } else if (value < 0) {
    r = g = b = 0;
  } else if (value < max3) {
    r = 0;
    g = 0;
    b = (float)(255 * value / max3);
  } else if (value < 2 * max3) {
    r = 0;
    g = (float)(255 * (value - max3) / max3);
    b = 255;
  } else if (value < max) {
    r = (float)(255 * (value - 2 * max3) / max3);
    g = 255;
    b = 255;
  } else {
    r = g = b = 255;
  }

  r /= 255.0;
  g /= 255.0;
  b /= 255.0;
}

/*****************************************************************************
*****************************************************************************/
void colorMapHotPathGet(float value, float min, float max,
                        float &r, float &g, float &b, bool invert, int flag) {

  float rangeVal;
  rangeVal = max - min;
  value = value - min;

  if (flag != LINEAR_SCL) {
    switch (flag) {
    case SQRT_SCL:
      value = sqrt(value);
      rangeVal = sqrt(rangeVal);
      break;

    case LOG_SCL:
      value = log(value);
      rangeVal = log(rangeVal);
      break;
    }
  }

  value = value / rangeVal;

  if (invert) {
    value = 1.0 - value;
  }

  if (value < 0.0f) {
    r = 1.0f;
    g = b = r;
  } else if (value < 0.5f) {
    r = 2.0f * (0.5f - value);
    g = b = r;
  } else if (value < 1.0f) {
    r = value - 0.5f;
    g = b = 0.0f;
  } else {
    r = 1.0f;
    g = b = 0.0f;
  }
}

/*****************************************************************************
*****************************************************************************/
void colorMapGoogleMapsGet(float pop, float elev, float &r, float &g,
                           float &b) {
  if (pop > 200.0) {
    r = 235.0 / 255.0;
    g = 230.0 / 255.0;
    b = 220.0 / 255.0;
  } else if (pop > 25.0) {

    r = 242.0 / 255.0;
    g = 239.0 / 255.0;
    b = 233.0 / 255.0;
  } else {
    r = 167.0 / 255.0;
    g = 204.0 / 255.0;
    b = 149.0 / 255.0;
  }

  //render water
  if (elev == 0.0) {
    r = 153.0 / 255.0;
    g = 179.0 / 255.0;
    b = 204.0 / 255.0;
  }
}

void colorMapYiGnBuGet(float value, float min, float max,
                       float &r, float &g, float &b, bool invert, int flag) {

  int numIntervals = 9;
  float delta = (max - min) / ((float)numIntervals);
  //value = value-min;
  value = max - (value - min);

  //float x;
  //x = 255.0*((value-min)/(max-min));

  if (value == FLT_MAX) {
    r = g = b = 255;
  } else if (value <= 0) {
    r = 0.0;
    g = 0.0;
    b = 0.0;
  } else if (value < 1.0 * delta) {
    r = 8.0, g = 29.0, b = 88.0;
  } else if (value < 2.0 * delta) {
    r = 37.0, g = 52.0, b = 148.0;
  } else if (value < 3.0 * delta) {
    r = 34.0, g = 94.0, b = 168.0;
  } else if (value < 4.0 * delta) {
    r = 29.0, g = 145.0, b = 192.0;
  } else if (value < 5.0 * delta) {
    r = 65.0, g = 182.0, b = 196.0;
  } else if (value < 6.0 * delta) {
    r = 127.0, g = 205.0, b = 187.0;
  } else if (value < 7.0 * delta) {
    r = 199.0, g = 233.0, b = 180.0;
  } else if (value < 8.0 * delta) {
    r = 237.0, g = 248.0, b = 177.0;
  } else if (value < FLT_MAX) {
    r = 255.0, g = 255.0, b = 217.0;
  }

  //else{r=g=b=x;}

  r /= 255.0;
  g /= 255.0;
  b /= 255.0;
}

/*****************************************************************************
*****************************************************************************/
void colorMapTerrainGet(float value, float min, float max,
                        float &r, float &g, float &b, bool isPark, int flag) {
  int numIntervals = 14;
  float delta = (max - min) / ((float)numIntervals);
  //float delta = (MAX_TERRAIN_HEIGHT-MIN_TERRAIN_HEIGHT)/((float)numIntervals);
  value -= min;

  if (value == FLT_MAX) {
    r = g = b = 255;
  } else if (value <= 0.0f) {
    r = 60.0;
    g = 80.0;
    b = 100.0;
  } else if (value < 1.0 * delta) {
    r = 139.0, g = 146.0, b = 112.0;
  } else if (value < 2.0 * delta) {
    r = 158.0, g = 159.0, b = 117.0;
  } else if (value < 3.0 * delta) {
    r = 177.0, g = 173.0, b = 123.0;
  } else if (value < 4.0 * delta) {
    r = 196.0, g = 186.0, b = 129.0;
  } else if (value < 5.0 * delta) {
    r = 215.0, g = 200, b = 135.0;
  } else if (value < 6.0 * delta) {
    r = 208.0, g = 190.0, b = 128.0;
  } else if (value < 7.0 * delta) {
    r = 202.0, g = 180.0, b = 121.0;
  } else if (value < 8.0 * delta) {
    r = 195.0, g = 170.0, b = 114.0;
  } else if (value < 9.0 * delta) {
    r = 189.0, g = 160.0, b = 107.0;
  } else if (value < 10.0 * delta) {
    r = 183.0, g = 150.0, b = 101.0;
  } else if (value < 11.0 * delta) {
    r = 179.0, g = 154.0, b = 113.0;
  } else if (value < 12.0 * delta) {
    r = 175.0, g = 158.0, b = 126.0;
  } else if (value < 13.0 * delta) {
    r = 171.0, g = 162.0, b = 138.0;
  } else if (value < FLT_MAX) {
    r = 167.0, g = 167.0, b = 151.0;
  }

  r /= 255.0;
  g /= 255.0;
  b /= 255.0;
}

/*****************************************************************************
*****************************************************************************/
void colorMapGrayscaleGet(float value, float min, float max,
                          float &r, float &g, float &b, int flag) {
  r = 1.0f - (value - min) / (max - min);
  g = r;
  b = r;
}


bool readSegmentsFromFile(QString filename, std::vector<QVector3D> &pts,
                          std::vector< std::vector<int> > &idxsSets) {
  //read initial edges from file
  QFile initialEdgesFile(filename);

  if (!initialEdgesFile.open(QIODevice::ReadOnly | QIODevice::Text))			{
    std::cout << "ERROR: Cannot open the file " << filename.toLocal8Bit().data() <<
              " for reading\n";
    return false;
  }

  idxsSets.clear();
  pts.clear();

  QTextStream stream(&initialEdgesFile);

  QString line;

  int numVertices;
  int numEdges;
  int numObjects;

  //read number of objects
  line = stream.readLine();

  if (!line.isNull()) {
    numObjects = line.toInt();
  }

  for (int objIdx = 0; objIdx < numObjects; ++objIdx) {

    std::vector<int> tmpIdxVector;

    //read number of vertices
    line = stream.readLine();

    if (!line.isNull()) {
      numVertices = line.toInt();
    }

    //read number of edges
    line = stream.readLine();

    if (!line.isNull()) {
      numEdges = line.toInt();
    }

    //read in the vertices
    for (int i = 0; i < numVertices; ++i) {
      line = stream.readLine();

      if (!line.isNull()) {
        QStringList attList = line.split(" ");

        if (attList.size() != 4) {
          std::cout <<
                    "ERROR: Point entry must contain point index and three coordinates\n";
          continue;
        }

        int pointIdx = attList.at(0).toInt();

        QVector3D tmpPt;
        tmpPt = QVector3D(attList.at(1).toFloat(),
                          attList.at(2).toFloat(),
                          attList.at(3).toFloat());

        pts.push_back(tmpPt);
      }
    }

    //read in the edges
    int vtxAIdx, vtxBIdx;

    for (int i = 0; i < numEdges; ++i) {
      line = stream.readLine();

      if (!line.isNull()) {
        QStringList attList = line.split(" ");

        if (attList.size() != 2) {
          std::cout << "ERROR: Edges must be defined by two vertex indexes\n";
          continue;
        }

        vtxAIdx = attList.at(0).toInt();
        vtxBIdx = attList.at(1).toInt();

        tmpIdxVector.push_back(vtxAIdx);
        tmpIdxVector.push_back(vtxBIdx);
      }
    }

    if (tmpIdxVector.size() % 2 != 0) {
      std::cout << "ERROR reading file " << filename.toLocal8Bit().data() <<
                ". Number of segment indexes must be even.\n";
      return false;
    }

    idxsSets.push_back(tmpIdxVector);
  }

  return true;
}

bool loadPolygonsFromFile(QString filename, std::vector<Polygon3D> &polygons) {
  std::vector<QVector3D> pts;
  std::vector< std::vector<int> > idxs;

  int numPgons;

  if (!readSegmentsFromFile(filename, pts, idxs)) {
    return false;
  } else {
    numPgons = idxs.size();

    if ((pts.size() < 1) || (numPgons < 1)) {
      std::cout << "ERROR reading file " << filename.toLocal8Bit().data() <<
                ". Not enough points or polygons.\n";
      return false;
    }
  }

  for (int i = 0; i < numPgons; ++i) {
    int numPts = (idxs.at(i).size()) / 2;

    if (numPts < 3) {
      std::cout << "WARNING reading file " << filename.toLocal8Bit().data() <<
                ". Polygon contains fewer than 2 points and has been ignored.\n";
      continue;
    }

    Polygon3D tmpPgon;
    QVector3D tmpPt;

    for (int j = 0; j < numPts; ++j) {
      tmpPt = pts.at(idxs.at(i).at(2 * j));
      tmpPgon.contour.push_back(tmpPt);
    }

    polygons.push_back(tmpPgon);
  }

  return true;
}

//**
// Returns only the first polygon in the file
//**
bool loadPolygonFromFile(QString filename, Polygon3D &polygon) {
  std::vector<Polygon3D> tmpPolygons;

  if (!loadPolygonsFromFile(filename, tmpPolygons)) {
    return false;
  }

  if (tmpPolygons.size() < 1) {
    std::cout << "ERROR: file " << filename.toLocal8Bit().data() <<
              " does not contain any polygons.\n";
    return false;
  }

  polygon = tmpPolygons.at(0);
  return true;
}
}
}
