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


#include "roadGraphEdge.h"


namespace LC {

RoadGraphEdge::RoadGraphEdge(void) {
  edge_weight = 0.0;
  //maxSpeed=20000.0f;//20 km/h--> 20000 m/h

}


RoadGraphEdge::~RoadGraphEdge(void) {
  roadSegmentGeometry.clear();
}

/**
* Initialize.
**/
void RoadGraphEdge::init() {

  normDistFirstVertex.clear();

  unsigned int nVertices = roadSegmentGeometry.size();
  printf("init edge\n");
  edgeLength = 0;
  std::vector<float> segmentCumulativeTempLengths;

  for (unsigned int i = 0; i < nVertices - 1; i++) {
    QVector3D v(
      roadSegmentGeometry[i].x() - roadSegmentGeometry[i + 1].x(),
      roadSegmentGeometry[i].y() - roadSegmentGeometry[i + 1].y(),
      0);
    edgeLength += v.length();
    segmentCumulativeTempLengths.push_back(edgeLength);
  }

  //Distance from first vertex to first vertex is 0
  normDistFirstVertex.push_back(0);

  for (unsigned int i = 1; i < nVertices; i++) {
    normDistFirstVertex.push_back(segmentCumulativeTempLengths[i - 1] / edgeLength);
  }
}



}//namespace LC
