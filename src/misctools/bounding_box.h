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
*		Bounding Box
*
*
*		@desc Main
*		@author igarciad
*
************************************************************************************************/
#ifndef MTC_MISCTOOLS_BBOX_H
#define MTC_MISCTOOLS_BBOX_H

#include "common.h"

namespace LC {
namespace misctools {

class BBox {
 public:

  BBox() {
    this->resetMe();
  }

  ~BBox() {
  }

  BBox(const BBox &ref) {
    minPt = ref.minPt;
    maxPt = ref.maxPt;
  }

  inline BBox &operator=(const BBox &ref) {
    minPt = ref.minPt;
    maxPt = ref.maxPt;
    return (*this);
  }

  inline void resetMe(void) {
    minPt.setX(FLT_MAX);
    minPt.setY(FLT_MAX);
    minPt.setZ(FLT_MAX);
    maxPt.setX(-FLT_MAX);
    maxPt.setY(-FLT_MAX);
    maxPt.setZ(-FLT_MAX);
  }

  inline bool overlapsWithBBox(BBox &other) {
    return
      ((this->minPt.x() <= other.maxPt.x()) &&
       (this->maxPt.x() >= other.minPt.x())) &&
      ((this->minPt.y() <= other.maxPt.y()) &&
       (this->maxPt.y() >= other.minPt.y())) &&
      ((this->minPt.z() <= other.maxPt.z()) && (this->maxPt.z() >= other.minPt.z()));
  }

  void combineWithBBox(BBox &other);

  void addPoint(QVector3D &newPt);

  inline bool overlapsWithBBoxXY(BBox &other) {
    return
      ((this->minPt.x() <= other.maxPt.x()) &&
       (this->maxPt.x() >= other.minPt.x())) &&
      ((this->minPt.y() <= other.maxPt.y()) && (this->maxPt.y() >= other.minPt.y()));
  }

  inline QVector3D midPt(void) {
    return (0.5 * (minPt + maxPt));
  }

  QVector3D minPt;
  QVector3D maxPt;
};
}
}

#endif
