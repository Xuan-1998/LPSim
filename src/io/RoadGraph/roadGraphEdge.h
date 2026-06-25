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
#ifndef GEOMETRY_RGEDGE_H
#define GEOMETRY_RGEDGE_H

#include "misctools/misctools.h"
#include <vector>
#include <QVector3D>
#include <QSettings>

namespace LC {

/**
* RoadGraph.
**/
class RoadGraphEdge {

 public:
  /**
  * Constructor.
  **/
  RoadGraphEdge();

  /**
  * Destructor.
  **/
  ~RoadGraphEdge();

  /**
  * Initialize.
  **/
  void init();

  /**
  * Copy constructor.
  **/
  RoadGraphEdge(const RoadGraphEdge &ref) {
    // C
    edge_weight = ref.edge_weight;
    roadSegmentGeometry = ref.roadSegmentGeometry;
    roadSegmentWidth = ref.roadSegmentWidth;
    priority = ref.priority;

    // N
    edgeLength = ref.edgeLength;
    numberOfLanes = ref.numberOfLanes;
    cuda_design_state = ref.cuda_design_state;
    cuda_design_visited = ref.cuda_design_visited;
    cuda_design_modified = ref.cuda_design_modified;

    maxSpeedMperSec = ref.maxSpeedMperSec;

    averageSpeed = ref.averageSpeed;
    averageUtilization = ref.averageUtilization;

    // D
    label = ref.label;
    resfac = ref.resfac;
    faci = ref.faci;

    // CI
    inNum = ref.inNum;
    outNum = ref.outNum;
    inAngle = ref.inAngle;
    outAngle = ref.outAngle;
  }

  /**
  * Assignment operator.
  **/

  inline RoadGraphEdge &operator=(const RoadGraphEdge &ref) {
    // C
    edge_weight = ref.edge_weight;
    roadSegmentGeometry = ref.roadSegmentGeometry;
    roadSegmentWidth = ref.roadSegmentWidth;
    priority = ref.priority;

    // N
    edgeLength = ref.edgeLength;
    numberOfLanes = ref.numberOfLanes;
    cuda_design_state = ref.cuda_design_state;
    cuda_design_visited = ref.cuda_design_visited;
    cuda_design_modified = ref.cuda_design_modified;

    maxSpeedMperSec = ref.maxSpeedMperSec;

    averageSpeed = ref.averageSpeed;
    averageUtilization = ref.averageUtilization;

    // D
    label = ref.label;
    resfac = ref.resfac;
    faci = ref.faci;

    // CI
    inNum = ref.inNum;
    outNum = ref.outNum;
    inAngle = ref.inAngle;
    outAngle = ref.outAngle;

    return (*this);
  }

  /**
  * Geometry of the road along this graph edge
  **/
  std::vector<QVector3D> roadSegmentGeometry;

  /**
  * Normalized distance from the first geometrical vertex
  **/
  std::vector<float> normDistFirstVertex;

  /**
  * length is computed as the sum of lengths of roadSegentGeometry
  **/
  float edgeLength;
  float maxSpeedMperSec;

  float roadSegmentWidth;
  float edge_weight;
  int priority;


  unsigned int numberOfLanes;


  // Traffic
  std::vector<float> averageSpeed;
  std::vector<float> averageUtilization;

  // Design
  ushort cuda_design_state;
  ushort cuda_design_visited;
  ushort cuda_design_modified;

  // dynameq
  QString label;//name "St..."
  float resfac;//Response time factor
  uint faci;//priority (0 high)

  //complex intersections
  ushort inNum;//order in vertex of the angle
  ushort outNum;
  float inAngle;
  float outAngle;
 private:

};

}

#endif
