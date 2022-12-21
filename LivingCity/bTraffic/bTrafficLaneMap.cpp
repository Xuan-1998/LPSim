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
*		@desc Class that contains the structure of the lane maps
*		@author igaciad
************************************************************************************************/


#include "bTrafficLaneMap.h"

#include "bTrafficConstants.h"

#define LANE_DEBUG 0

namespace LC {

/*bool compareSecondPartTuple (const std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI,float> &i, const std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI,float> &j) {
        return (i.second<j.second);
}*/

void BTrafficLaneMap::createLaneMap(
  RoadGraph &inRoadGraph,
  std::vector<unsigned long>
  (&laneMapL)[2], //std::vector<unsigned long>& laneMapL,
  BEdgesData &edgesData,
  std::map<uint, RoadGraph::roadGraphEdgeDesc_BI> &laneMapNumToEdgeDesc,
  std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum,
  ushort &maxWidthL) {
  //////////////////////////////////////////////////////////
  // GENERATE LANE MAP
  if (LANE_DEBUG) {
    printf("  >> createLaneMap CS: %f\n", cellSize);
  }

  // 1. Cretae edgesData and find requires sizes fro e
  RoadGraph::roadGraphEdgeIter_BI ei, ei_end;
  int edge_count = 0;
  int tNumLanes = 0;
  ushort maxLengthC = 0;

  //printf("edgesData %d\n",edgesData);
  edgesData.resize(boost::num_edges(inRoadGraph.myRoadGraph_BI) *
                   4); //3 to make sure it fits
  edgeDescToLaneMapNum.clear();
  laneMapNumToEdgeDesc.clear();

  for (boost::tie(ei, ei_end) = boost::edges(inRoadGraph.myRoadGraph_BI);
       ei != ei_end; ++ei) {

    // num lanes
    int numLanes = inRoadGraph.myRoadGraph_BI[*ei].numberOfLanes;

    if (numLanes == 0) {
      //printf("ERROR: GENERATE LANE MAP Num of lanes is 0\n");//chaning lanes could make this to be 0
      continue;//edges with zero lines just skip
    }

    //printf("Ed %d ei %d %d\n",edge_count,ei,*ei);
    // length
    edgesData.lengthC[tNumLanes] = inRoadGraph.myRoadGraph_BI[*ei].edgeLength /
                                   cellSize;

    //printf("inRoadGraph.myRoadGraph_BI[*ei].edgeLength %f\n",inRoadGraph.myRoadGraph_BI[*ei].edgeLength);
    //edgesData[tNumLanes].lengthCells=ceil(length/cellSize);
    //length/=cellSize;
    if (maxLengthC < edgesData.lengthC[tNumLanes]) {
      maxLengthC = edgesData.lengthC[tNumLanes];
    }

    //printf("numLanes %d\n",numLanes);
    edgesData.numLinesB[tNumLanes] = numLanes;
    float maxSpeedMperSec = inRoadGraph.myRoadGraph_BI[*ei].maxSpeedMperSec;
    edgesData.maxSpeedCpSec[tNumLanes] = maxSpeedMperSec / cellSize;
    // next intersection
    edgesData.nextInters[tNumLanes] = boost::target(*ei,
                                      inRoadGraph.myRoadGraph_BI);

    edgeDescToLaneMapNum.insert(std::make_pair(*ei, tNumLanes));
    laneMapNumToEdgeDesc.insert(std::make_pair(tNumLanes, *ei));

    tNumLanes += numLanes;
    edge_count++;
  }

  edgesData.resize(tNumLanes);

  if (LANE_DEBUG) {
    printf("Num edges %d Num Lanes %d Max Leng %u\n", edge_count, tNumLanes,
           maxLengthC);
  }

  // 2. RESIZE LANE MAP
  //maxWidth=ceil(maxLength/cellSize);// DIV/8 bitmap but NOT DEV TO PUT THE SPEED


  maxWidthL = (maxLengthC + 15) & ~15u;//round 16 chars
  maxWidthL = maxWidthL / 8;//number of long needed

  //laneMapL.resize(maxWidthL*tNumLanes * 2);//2 to keep both maps
  laneMapL[0].resize(maxWidthL * tNumLanes);
  memset(laneMapL[0].data(), -1, laneMapL[0].size()*sizeof(unsigned long));
  laneMapL[1].resize(maxWidthL * tNumLanes);
  memset(laneMapL[1].data(), -1, laneMapL[0].size()*sizeof(unsigned long));
  //memset(laneMapL.data(),-1,laneMap.size()*sizeof (unsigned char));//set FF

  ////printf("aa\n");
  ////////////////////////////////////////////////////////////
  //// GENERATE INTERSECTION INFO
  //RoadGraph::roadGraphVertexIter_BI vi, viEnd;
  //RoadGraph::in_roadGraphEdgeIter_BI Iei, Iei_end;
  //RoadGraph::out_roadGraphEdgeIter_BI Oei, Oei_end;
  //intersec.resize(boost::num_vertices(inRoadGraph.myRoadGraph_BI));//as many as vertices
  //stops.resize(boost::num_vertices(inRoadGraph.myRoadGraph_BI));//!!! this should not be the case
  //if(LANE_DEBUG)printf(">>Generate Intersection Info\n");
  //for(boost::tie(vi, viEnd) = boost::vertices(inRoadGraph.myRoadGraph_BI);
  //	vi != viEnd; ++vi)
  //{
  //	intersec.numIn[*vi] = boost::in_degree(*vi, inRoadGraph.myRoadGraph_BI);//(*vi,inRoadGraph.myRoadGraph_BI)+boost::in_degree(*vi,inRoadGraph.myRoadGraph_BI);
  //
  //	if (intersec.numIn[*vi] <= 1) {
  //		intersec.trafficLight[*vi] = -1;
  //		continue;//always green
  //	}
  //
  //	intersec.nextEvent[*vi] = 0.0f;
  //	intersec.req[*vi] = 0;
  //	intersec.trafficLight[*vi] = 0;

  //	int intersecType = 0;//stop
  //	if (intersecType==0) {
  //		intersec.type[*vi] = 0;
  //		stops.state[*vi] = 0;
  //		stops.intersNumber[*vi] = *vi;
  //	}
  //
  //	/*
  //	//sort by angle
  //	QVector3D referenceVector(0,1,0);
  //	QVector3D p0,p1;
  //	std::vector<std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI,float>> edgeAngleOut;
  //	//printf("Out\n");
  //	int numOutEdges=0;
  //
  //	float angleRef=atan2(referenceVector.y(),referenceVector.x());
  //	for(boost::tie(Oei, Oei_end) = boost::out_edges(*vi,inRoadGraph.myRoadGraph_BI); Oei != Oei_end; ++Oei){
  //		if(inRoadGraph.myRoadGraph_BI[*Oei].numberOfLanes==0)
  //			continue;
  //		p0 = inRoadGraph.myRoadGraph_BI[boost::source(*Oei, inRoadGraph.myRoadGraph_BI)].pt;
  //		p1 = inRoadGraph.myRoadGraph_BI[boost::target(*Oei, inRoadGraph.myRoadGraph_BI)].pt;
  //		QVector3D edgeDir=(p1-p0).normalized();// NOTE p1-p0
  //		//float angle=QVector3D::dotProduct(referenceVector,edgeDir);
  //		float angle=angleRef-atan2(edgeDir.y(),edgeDir.x());
  //		edgeAngleOut.push_back(std::make_pair(*Oei,angle));
  //		if(edgeDescToLaneMapNum.find(*Oei)==edgeDescToLaneMapNum.end()){
  //				printf("->ERROR OUT\n");//edge desc not found in map
  //		}
  //		numOutEdges++;
  //	}
  //	//printf("Out %d\n",numOutEdges);
  //	std::vector<std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI,float>> edgeAngleIn;
  //	//printf("In\n");
  //	int numInEdges=0;
  //	for(boost::tie(Iei, Iei_end) = boost::in_edges(*vi,inRoadGraph.myRoadGraph_BI); Iei != Iei_end; ++Iei){
  //		if(inRoadGraph.myRoadGraph_BI[*Iei].numberOfLanes==0)
  //			continue;
  //		p0 = inRoadGraph.myRoadGraph_BI[boost::source(*Iei, inRoadGraph.myRoadGraph_BI)].pt;
  //		p1 = inRoadGraph.myRoadGraph_BI[boost::target(*Iei, inRoadGraph.myRoadGraph_BI)].pt;
  //		QVector3D edgeDir=(p0-p1).normalized();// NOTE p0-p1
  //		float angle=angleRef-atan2(edgeDir.y(),edgeDir.x());
  //		edgeAngleIn.push_back(std::make_pair(*Iei,angle));
  //		if(edgeDescToLaneMapNum.find(*Iei)==edgeDescToLaneMapNum.end()){
  //				printf("->ERROR IN\n");//edge desc not found in map
  //				continue;
  //			}
  //		numInEdges++;
  //	}
  //	intersections[*vi].totalInOutEdges=numOutEdges+numInEdges;
  //	//printf("In %d\n",numInEdges);
  //	//printf("Sort\n");
  //	//save in sorterd way as lane number
  //	if(edgeAngleOut.size()>0){
  //		std::sort( edgeAngleOut.begin(), edgeAngleOut.end(), compareSecondPartTuple);
  //	}
  //	if(edgeAngleIn.size()>0){
  //		std::sort( edgeAngleIn.begin(), edgeAngleIn.end(), compareSecondPartTuple);
  //	}
  //	//!!!!
  //	int outCount=0;
  //	int inCount=0;
  //	int totalCount=0;
  //	//printf("count %d\n",);
  //	// INTERSECTION
  //	//      0xFF00 0000 Num lines
  //	//      0x00FF 0000 in out
  //	//      0x0000 FFFF Edge number
  //	for(int iter=0;iter<edgeAngleOut.size()+edgeAngleIn.size();iter++){
  //		if((outCount<edgeAngleOut.size()&&inCount<edgeAngleIn.size()&&edgeAngleOut[outCount].second<=edgeAngleIn[inCount].second)||
  //			(outCount<edgeAngleOut.size()&&inCount>=edgeAngleIn.size())){
  //			intersections[*vi].edge[totalCount]=edgeDescToLaneMapNum[edgeAngleOut[outCount].first];
  //			intersections[*vi].edge[totalCount]|=(edgesData[intersections[*vi].edge[totalCount]].numLines<<24);//put the number of lines in each edge
  //			intersections[*vi].edge[totalCount]|=0x00000;//mask to define out edge
  //			outCount++;
  //		}else{
  //			intersections[*vi].edge[totalCount]=edgeDescToLaneMapNum[edgeAngleIn[inCount].first];
  //			intersections[*vi].edge[totalCount]|=(edgesData[intersections[*vi].edge[totalCount]].numLines<<24);//put the number of lines in each edge
  //			intersections[*vi].edge[totalCount]|=0x010000;//mask to define in edge
  //			inCount++;
  //		}
  //		totalCount++;
  //	}
  //	if(totalCount!=intersections[*vi].totalInOutEdges){
  //		printf("Error totalCount!=intersections[*vi].totalInOutEdges %d %d\n",totalCount,intersections[*vi].totalInOutEdges);
  //	}*/
  //
  //}
  //if(LANE_DEBUG)printf("<<Generate Intersection Info\n");

  if (LANE_DEBUG) {
    printf("  << createLaneMap CS: maxWidthL %d\n", maxWidthL);
  }
}//

/*void BTrafficLaneMap::resetIntersections(std::vector<intersectionData>& intersections, std::vector<uchar>& trafficLights){
	for(int i=0;i<intersections.size();i++){
		intersections[i].nextEvent=0.0f;//otherwise they not change until reach time again
		intersections[i].state=0;//to make the system to repeat same execution
	}
	if(trafficLights.size()>0)
		memset(trafficLights.data(),0,trafficLights.size()*sizeof(uchar));//to make the system to repeat same execution
}//*/

}
