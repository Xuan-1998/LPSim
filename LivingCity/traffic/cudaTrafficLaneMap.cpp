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
*		LC Project - CUDA Traffic lane map
*
*
*		@desc Class that contains the structure of the lane maps
*		@author igaciad
*
************************************************************************************************/

#include "cudaTrafficLaneMap.h"

#define LANE_DEBUG 1

namespace LC {

CUDATrafficLaneMap::CUDATrafficLaneMap() {
}//

CUDATrafficLaneMap::~CUDATrafficLaneMap() {
}//

bool compareSecondPartTupleC(const
                             std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI, float> &i,
                             const std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI, float> &j) {
  return (i.second < j.second);
}

void CUDATrafficLaneMap::createLaneMap(
  RoadGraph &inRoadGraph,
  std::vector<uchar> &laneMap,
  std::vector<edgeData> &edgesData,
  std::vector<intersectionData> &intersections,
  std::vector<uchar> &trafficLights,
  std::map<uint, RoadGraph::roadGraphEdgeDesc_BI> &laneMapNumToEdgeDesc,
  std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum,
  ushort &maxWidth,
  float cellSize) {
  //////////////////////////////////////////////////////////
  // GENERATE LANE MAP
  if (LANE_DEBUG) {
    printf("  >> createLaneMap CellSize: %.2fm\n", cellSize);
  }

  // 1. Cretae edgesData and find requires sizes.
  RoadGraph::roadGraphEdgeIter_BI ei, ei_end;
  int edge_count = 0;
  int tNumLanes = 0;
  float maxLength = 0;
  int maxNumLanes = 0;

  //printf("edgesData %d\n",edgesData);
  edgesData.resize(boost::num_edges(inRoadGraph.myRoadGraph_BI) *
                   4); //3 to make sure it fits

  edgeDescToLaneMapNum.clear();
  laneMapNumToEdgeDesc.clear();

  // Distribution of length
  float binLength = 1.0f;//1km
  int numBins = 27 / binLength;//maxlength is 26km
  std::vector<int> bins(numBins);
  std::fill(bins.begin(), bins.end(), 0);
  for (boost::tie(ei, ei_end) = boost::edges(inRoadGraph.myRoadGraph_BI); ei != ei_end; ++ei) {
    float length = inRoadGraph.myRoadGraph_BI[*ei].edgeLength;
    int binN = (length / 1000.0f) / binLength;
    //printf("l %.2f binN %d\n", length, binN);
    if (binN < 0 || binN >= numBins) {
      printf("ERROR: Bin out of range %d of %f\n", binN, numBins);
    }
    bins[binN]++;
  }
  for (int binN = 0; binN < bins.size(); binN++) {
    printf("%.0fkm, %d\n", (binN * binLength+1.0f), bins[binN]);
  }


  // Create EdgeData
  
  for (boost::tie(ei, ei_end) = boost::edges(inRoadGraph.myRoadGraph_BI); ei != ei_end; ++ei) {

    // num lanes
    int numLanes = inRoadGraph.myRoadGraph_BI[*ei].numberOfLanes;

    if (numLanes == 0) {
      //printf("ERROR: GENERATE LANE MAP Num of lanes is 0\n");//chaning lanes could make this to be 0
      continue;//edges with zero lines just skip
    }

    //printf("Ed %d ei %d %d\n",edge_count,ei,*ei);
    // length
    edgesData[tNumLanes].length = inRoadGraph.myRoadGraph_BI[*ei].edgeLength;

    //printf("inRoadGraph.myRoadGraph_BI[*ei].edgeLength %f\n",inRoadGraph.myRoadGraph_BI[*ei].edgeLength);
    //edgesData[tNumLanes].lengthCells=ceil(length/cellSize);
    //length/=cellSize;
    if (edgesData[tNumLanes].length > 65535.0f) {
      printf("Edge longer than 65535, ushort logic in simulator will fail, update that code.");
      exit(-1);
    }
    if (maxLength < edgesData[tNumLanes].length) {
      maxLength = edgesData[tNumLanes].length;
    }
    if (maxNumLanes < numLanes) {
      maxNumLanes = numLanes;
    }

    //printf("numLanes %d\n",numLanes);
    edgesData[tNumLanes].numLines = numLanes;
    float maxSpeedMperSec = inRoadGraph.myRoadGraph_BI[*ei].maxSpeedMperSec;
    edgesData[tNumLanes].maxSpeedMperSec = maxSpeedMperSec;
    // next intersection
    edgesData[tNumLanes].nextInters = boost::target(*ei,
                                      inRoadGraph.myRoadGraph_BI);

    edgeDescToLaneMapNum.insert(std::make_pair(*ei, tNumLanes));
    laneMapNumToEdgeDesc.insert(std::make_pair(tNumLanes, *ei));

    tNumLanes += numLanes;
    edge_count++;
  }

  edgesData.resize(tNumLanes);

  if (LANE_DEBUG) {
    printf("Num edges %d Num Lanes %d Max Leng %f Max num lanes %d\n", edge_count, tNumLanes, maxLength, maxNumLanes);
  }

  // 2. RESIZE LANE MAP
  maxWidth = ceil(maxLength /
                  cellSize); // DIV/8 bitmap but NOT DEV TO PUT THE SPEED
  maxWidth = 4 * (((maxWidth - 1) / 4) + 1); //padding

  printf("Total Memory %d\n", maxWidth * tNumLanes * 2);
  laneMap.resize(maxWidth * tNumLanes * 2); //2 to keep both maps
  memset(laneMap.data(), -1, laneMap.size()*sizeof(unsigned char)); //

  //printf("aa\n");
  //////////////////////////////////////////////////////////
  // GENERATE INTERSECTION INFO
  RoadGraph::roadGraphVertexIter_BI vi, viEnd;
  RoadGraph::in_roadGraphEdgeIter_BI Iei, Iei_end;
  RoadGraph::out_roadGraphEdgeIter_BI Oei, Oei_end;
  intersections.resize(boost::num_vertices(
                         inRoadGraph.myRoadGraph_BI));//as many as vertices
  trafficLights.resize(tNumLanes);
  memset(trafficLights.data(), 0, trafficLights.size()*sizeof(uchar));

  if (LANE_DEBUG) {
    printf(">>Generate Intersection Info\n");
  }

  for (boost::tie(vi, viEnd) = boost::vertices(inRoadGraph.myRoadGraph_BI);
       vi != viEnd; ++vi) {
    //glVertex3f(inRoadGraph.myRoadGraph_BI[*vi].pt.x(),inRoadGraph.myRoadGraph_BI[*vi].pt.y(),inRoadGraph.myRoadGraph_BI[*vi].pt.z());
    intersections[*vi].state = 0;
    intersections[*vi].nextEvent = 0.0f;
    //printf("outIn intersections[*vi].totalInOutEd\n");
    intersections[*vi].totalInOutEdges = boost::degree(*vi,
                                         inRoadGraph.myRoadGraph_BI);//(*vi,inRoadGraph.myRoadGraph_BI)+boost::in_degree(*vi,inRoadGraph.myRoadGraph_BI);

    if (intersections[*vi].totalInOutEdges <= 0) {
      //printf("Vertex without in/out edges\n");
      continue;
    }

    if (intersections[*vi].totalInOutEdges >= 20) {
      printf("Vertex with more than 20 in/out edges\n");
      continue;
    }

    //printf("Total: %d\n",intersections[*vi].totalInOutEdges);
    //sort by angle
    QVector3D referenceVector(0, 1, 0);
    QVector3D p0, p1;
    std::vector<std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI, float>> edgeAngleOut;
    //printf("Out\n");
    int numOutEdges = 0;

    float angleRef = atan2(referenceVector.y(), referenceVector.x());

    for (boost::tie(Oei, Oei_end) = boost::out_edges(*vi,
                                    inRoadGraph.myRoadGraph_BI); Oei != Oei_end; ++Oei) {
      if (inRoadGraph.myRoadGraph_BI[*Oei].numberOfLanes == 0) {
        continue;
      }

      p0 = inRoadGraph.myRoadGraph_BI[boost::source(*Oei,
                                      inRoadGraph.myRoadGraph_BI)].pt;
      p1 = inRoadGraph.myRoadGraph_BI[boost::target(*Oei,
                                      inRoadGraph.myRoadGraph_BI)].pt;
      QVector3D edgeDir = (p1 - p0).normalized(); // NOTE p1-p0
      //float angle=QVector3D::dotProduct(referenceVector,edgeDir);
      float angle = angleRef - atan2(edgeDir.y(), edgeDir.x());
      /*LC::RoadGraph::roadGraphVertexDesc_BI sV=boost::source(*Oei, inRoadGraph.myRoadGraph_BI);
      LC::RoadGraph::roadGraphVertexDesc_BI tV=boost::target(*Oei, inRoadGraph.myRoadGraph_BI);
      std::pair<RoadGraph::roadGraphEdgeDesc,bool> edge_pair =
        boost::edge(sV,tV,inRoadGraph.myRoadGraph_BI);*/
      edgeAngleOut.push_back(std::make_pair(*Oei, angle));

      if (edgeDescToLaneMapNum.find(*Oei) == edgeDescToLaneMapNum.end()) {
        printf("->ERROR OUT\n");//edge desc not found in map
      }

      numOutEdges++;
      //edgeAngleOut.push_back(std::make_pair(edge_pair.first,angle));
    }

    //printf("Out %d\n",numOutEdges);
    std::vector<std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI, float>> edgeAngleIn;
    //printf("In\n");
    int numInEdges = 0;

    for (boost::tie(Iei, Iei_end) = boost::in_edges(*vi,
                                    inRoadGraph.myRoadGraph_BI); Iei != Iei_end; ++Iei) {
      if (inRoadGraph.myRoadGraph_BI[*Iei].numberOfLanes == 0) {
        continue;
      }

      p0 = inRoadGraph.myRoadGraph_BI[boost::source(*Iei,
                                      inRoadGraph.myRoadGraph_BI)].pt;
      p1 = inRoadGraph.myRoadGraph_BI[boost::target(*Iei,
                                      inRoadGraph.myRoadGraph_BI)].pt;
      QVector3D edgeDir = (p0 - p1).normalized(); // NOTE p0-p1
      //float angle=QVector3D::dotProduct(referenceVector,edgeDir);
      float angle = angleRef - atan2(edgeDir.y(), edgeDir.x());
      //std::pair<RoadGraph::roadGraphEdgeDesc, bool> edge_pair = boost::edge(boost::source(*Iei, inRoadGraph.myRoadGraph_BI),boost::target(*Iei, inRoadGraph.myRoadGraph_BI),inRoadGraph.myRoadGraph_BI);
      edgeAngleIn.push_back(std::make_pair(*Iei, angle));

      if (edgeDescToLaneMapNum.find(*Iei) == edgeDescToLaneMapNum.end()) {
        printf("->ERROR IN\n");//edge desc not found in map
        continue;
      }

      numInEdges++;
      //edgeAngleIn.push_back(std::make_pair(edge_pair.first,angle));
    }

    intersections[*vi].totalInOutEdges = numOutEdges + numInEdges;

    //printf("In %d\n",numInEdges);
    //printf("Sort\n");
    //save in sorterd way as lane number
    if (edgeAngleOut.size() > 0) {
      std::sort(edgeAngleOut.begin(), edgeAngleOut.end(), compareSecondPartTupleC);
    }

    if (edgeAngleIn.size() > 0) {
      std::sort(edgeAngleIn.begin(), edgeAngleIn.end(), compareSecondPartTupleC);
    }

    //!!!!
    int outCount = 0;
    int inCount = 0;
    int totalCount = 0;

    //printf("count %d\n",);
    // INTERSECTION
    //      0xFF00 0000 Num lines
    //      0x00FF 0000 in out
    //      0x0000 FFFF Edge number
    for (int iter = 0; iter < edgeAngleOut.size() + edgeAngleIn.size(); iter++) {
      if ((outCount < edgeAngleOut.size() && inCount < edgeAngleIn.size() &&
           edgeAngleOut[outCount].second <= edgeAngleIn[inCount].second) ||
          (outCount < edgeAngleOut.size() && inCount >= edgeAngleIn.size())) {
        intersections[*vi].edge[totalCount] = edgeDescToLaneMapNum[edgeAngleOut[outCount].first];
        intersections[*vi].edge[totalCount] |= (edgesData[intersections[*vi].edge[totalCount]].numLines << 24); //put the number of lines in each edge
        intersections[*vi].edge[totalCount] |= 0x00000; //mask to define out edge
        outCount++;
      } else {
        intersections[*vi].edge[totalCount] = edgeDescToLaneMapNum[edgeAngleIn[inCount].first];
        intersections[*vi].edge[totalCount] |= (edgesData[intersections[*vi].edge[totalCount]].numLines << 24); //put the number of lines in each edge
        intersections[*vi].edge[totalCount] |= 0x010000; //mask to define in edge
        inCount++;
      }

      totalCount++;
    }

    if (totalCount != intersections[*vi].totalInOutEdges) {
      printf("Error totalCount!=intersections[*vi].totalInOutEdges %d %d\n",
             totalCount, intersections[*vi].totalInOutEdges);
    }

  }

  if (LANE_DEBUG) {
    printf("<<Generate Intersection Info\n");
  }

  if (LANE_DEBUG) {
    printf("  << createLaneMap CS: %f maxWidth %d\n", cellSize, maxWidth);
  }
}//

void CUDATrafficLaneMap::resetIntersections(std::vector<intersectionData>
    &intersections, std::vector<uchar> &trafficLights) {
  for (int i = 0; i < intersections.size(); i++) {
    intersections[i].nextEvent =
      0.0f; //otherwise they not change until reach time again
    intersections[i].state = 0; //to make the system to repeat same execution
  }

  if (trafficLights.size() > 0) {
    memset(trafficLights.data(), 0, trafficLights.size()*sizeof(
             uchar));  //to make the system to repeat same execution
  }
}//

}
