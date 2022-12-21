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


#include "roadGraphVertex.h"
#include <math.h>


#define M_PI 3.1415926535897932

namespace LC {

RoadGraphVertex::RoadGraphVertex(void) {
  prio = 2;//AWSC
}

RoadGraphVertex::RoadGraphVertex(QVector3D inPt, float dU, float dV,
                                 float ref, int dirCount, int inRandSeed, float inDeltaTheta, bool inIsSeed,
                                 bool inIsBoundingPgonVertex, float inIrregularity, float inCurvature,
                                 float inWidth, int inMyPlaceTypeIdx, float inSpeed) {
  pt = inPt;
  distU = dU;
  distV = dV;
  initMyDirections(ref, dirCount);
  randSeed = inRandSeed;
  deltaTheta = inDeltaTheta;
  isSeed = inIsSeed;
  isBoundingPgonVertex = inIsBoundingPgonVertex;
  irregularity = inIrregularity;
  curvature = inCurvature;
  width = inWidth;
  myPlaceTypeIdx = inMyPlaceTypeIdx;
  speed = inSpeed;
}

RoadGraphVertex::~RoadGraphVertex(void) {

}

/**
* Adapt vertex to terrain
**/
void RoadGraphVertex::adaptRGVertexToTerrain(ElevationGrid *elGrid) {
}

/**
* Initialize directions based on reference direction and direction count
**/
void RoadGraphVertex::initMyDirections(float ref, int dirCount) {
  if (dirCount < 1) {
    return;
  }

  float delta = (2.0f * M_PI) / ((float)(dirCount));
  float newDir;

  for (int i = 0; i < dirCount; ++i) {
    newDir = ref + ((float)i) * delta;

    if (newDir > 2.0f * M_PI) {
      newDir = newDir - 2.0f * M_PI;
    }

    departingDirections.push_back(newDir);
  }
}

float RoadGraphVertex::getDistFromDirAngle(float ang, float inRef, bool &isU) {
  if (departingDirections.size() < 1) {
    return 0.0f;
  }

  if ((fabs(fabs(ang - inRef)) < 0.25f * M_PI) ||
      (fabs(fabs(ang - inRef) -      M_PI) < 0.25f * M_PI) ||
      (fabs(fabs(ang - inRef) - 2.0f * M_PI) < 0.25f * M_PI)) {
    isU = true;
    return this->distU;
  } else {
    isU = false;
    return this->distV;
  }
}


}//namespace LC
