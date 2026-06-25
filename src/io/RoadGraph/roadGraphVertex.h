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
*		MTC Project - Geometry Project - RoadGraph Class
*
*
*		@desc Class containing the road graph information
*		@author cvanegas
*
************************************************************************************************/
#ifndef GEOMETRY_RGVERTEX_H
#define GEOMETRY_RGVERTEX_H

#include <QVector3D>
#include <vector>

namespace LC {

class ElevationGrid;

/**
* RoadGraphVertex.
**/
class RoadGraphVertex {

 public:
  /**
  * Constructor.
  **/
  RoadGraphVertex();

  RoadGraphVertex(QVector3D pt, float dU, float dV, float ref, int dirCount,
                  int randSeed, float deltaTheta, bool isSeed, bool isBoundingPgonVertex,
                  float inIrregularity, float inCurvature,
                  float width,
                  int inMyPlaceTypeIdx, float inSpeed);

  /**
  * Destructor.
  **/
  ~RoadGraphVertex();

  /**
  * Copy constructor.
  **/
  RoadGraphVertex(const RoadGraphVertex &ref) {
    pt = ref.pt;
    departingDirections = ref.departingDirections;
    distU = ref.distU;
    distV = ref.distV;
    randSeed = ref.randSeed;
    deltaTheta = ref.deltaTheta;
    isSeed = ref.isSeed;
    isBoundingPgonVertex = ref.isBoundingPgonVertex;
    irregularity = ref.irregularity;
    curvature = ref.curvature;
    width = ref.width;
    myPlaceTypeIdx = ref.myPlaceTypeIdx;
    speed = ref.speed;

    prio = ref.prio;
    type = ref.type;
  }

  /**
  * Assignment operator.
  **/

  inline RoadGraphVertex &operator=(const RoadGraphVertex &ref) {
    pt = ref.pt;
    departingDirections = ref.departingDirections;
    distU = ref.distU;
    distV = ref.distV;
    randSeed = ref.randSeed;
    deltaTheta = ref.deltaTheta;
    isSeed = ref.isSeed;
    isBoundingPgonVertex = ref.isBoundingPgonVertex;
    irregularity = ref.irregularity;
    curvature = ref.curvature;
    width = ref.width;
    myPlaceTypeIdx = ref.myPlaceTypeIdx;
    speed = ref.speed;

    prio = ref.prio;
    type = ref.type;
    return (*this);
  }


  /**
  * Adapt edge to terrain
  **/
  void adaptRGVertexToTerrain(ElevationGrid *elGrid);

  /**
  * Initialize directions based on reference direction and direction count
  **/
  void initMyDirections(float ref, int dirCount);

  float getDistFromDirAngle(float ang, float inRef, bool &isU);

  /**
  * Index of my place type
  **/
  int myPlaceTypeIdx;

  /**
  *
  **/

  /**
  * Location of vertex
  **/
  QVector3D pt;

  //angles
  std::vector<float> departingDirections;

  //road segmemt distances
  float distU;
  float distV;

  //random seed
  int randSeed;

  //deltaTheta
  float deltaTheta;

  //is seed
  bool isSeed;
  bool isBoundingPgonVertex;

  //irregularity and curvature
  float irregularity;
  float curvature;

  //width
  float width;

  //speed
  float speed;

  // Dynameq
  uchar prio;//Priority Template: 0=None; 1=AWSC; 2=TWSC; 3=Roundabout; 4=Merge; 11=Signalized
  uchar type;// type Node Type : 1 = intersection, 2 = junction, 99 = virtual node

  // B2018
  uchar bType; // bType Node: 0 = Unknow 1 = motorway_junction 2 = traffic_signals 3 = stop 4 = turning_circle


 private:
};

}

#endif
