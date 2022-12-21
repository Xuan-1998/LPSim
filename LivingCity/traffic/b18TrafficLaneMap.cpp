/************************************************************************************************
*
*		LC Project - B18 Traffic lane map
*
*
*		@desc Class that contains the structure of the lane maps
*		@author igaciad
*
************************************************************************************************/

#include <ios>
#include <cassert>
#include <cmath>
#include "b18TrafficLaneMap.h"
#include "sp/graph.h"
#include "sp/config.h"


#define LANE_DEBUG 1

namespace LC {

B18TrafficLaneMap::B18TrafficLaneMap() {
}//

B18TrafficLaneMap::~B18TrafficLaneMap() {
}//

namespace {
  bool compareSecondPartTupleC(const
    std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI, float> &i,
    const std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI, float> &j) {
    return (i.second < j.second);
  }
  bool compareSecondPartTupleCSP(const std::pair<std::shared_ptr<abm::Graph::Edge>, float> &i,
		  	       const std::pair<std::shared_ptr<abm::Graph::Edge>, float> &j) {
    return (i.second < j.second);
  }

}

void B18TrafficLaneMap::createLaneMapSP(const std::shared_ptr<abm::Graph>& graph_, std::vector<uchar> &laneMap,
      std::vector<B18EdgeData> &edgesData, std::vector<B18IntersectionData> &intersections,
      std::vector<uchar> &trafficLights, 
      std::map<uint, std::shared_ptr<abm::Graph::Edge>> &laneMapNumToEdgeDescSP,
      std::map<std::shared_ptr<abm::Graph::Edge>, uint> &edgeDescToLaneMapNumSP,
      std::vector<uint> &edgeIdToLaneMapNum){
  // GENERATE LANE MAP
  if (LANE_DEBUG) {
    printf("  >> createLaneMap\n");
  }

  // 1. Cretae edgesData and find requires sizes.
  RoadGraph::roadGraphEdgeIter_BI ei, ei_end;
  int edge_count = 0;
  int tNumMapWidth = 0;
  int tNumLanes = 0;
  
  edgesData.resize(graph_->nedges() * 4); //4 to make sure it fits
  
  edgeDescToLaneMapNumSP.clear();
  laneMapNumToEdgeDescSP.clear();
  

  abm::graph::edge_id_t max_edge_id = 0;
  for (int v1_index = 0; v1_index < graph_->edge_ids_.size(); v1_index++){
    for (const std::pair<abm::graph::vertex_t, abm::graph::edge_id_t > v2_it: graph_->edge_ids_[v1_index]) {
      max_edge_id = std::max(max_edge_id, v2_it.second);
    }
  }

  edgeIdToLaneMapNum = std::vector<uint>(max_edge_id+1);
  
  // Check distribution of street length
  float binLength = 1.0f;//1km
  int numBins = 31 / binLength;//maxlength is 26km
  std::vector<int> bins(numBins, 0);
  for (auto const& x : graph_->edges_) {
	  const float metersLength = std::get<1>(x)->second.length;
	  const int binN = (metersLength / 1000.0f) / binLength;
	  assert(0 <= binN && binN < numBins && "Edge over max length");
	  bins[binN]++;
  }
  //for (int binN = 0; binN < bins.size(); binN++) {
  //  printf("%.0fkm, %d\n", (binN * binLength+1.0f), bins[binN]);
  //}

  /////////////////////////////////
  // Create EdgeData
  // Instead of having maxWidth (for b18 would have been 26km), we define a max width for the map and we wrap down.
  float maxLength = 0;
  int maxNumLanes = 0;
  for (auto const& x : graph_->edges_) {
	  const int numLanes = std::get<1>(x)->second.lanes;

    if (numLanes == 0) { continue; }

    edgesData[tNumMapWidth].length = std::get<1>(x)->second.length;
    edgesData[tNumMapWidth].maxSpeedMperSec = std::get<1>(x)->second.max_speed_limit_mps;

    if (maxLength < edgesData[tNumMapWidth].length) { maxLength = edgesData[tNumMapWidth].length; }
    if (maxNumLanes < numLanes) { maxNumLanes = numLanes; }

    const int numWidthNeeded = ceil(edgesData[tNumMapWidth].length / kMaxMapWidthM);
    edgesData[tNumMapWidth].numLines = numLanes;
    edgesData[tNumMapWidth].nextInters = std::get<1>(std::get<0>(x));
    //std::cout << " edge " << edge_count << " nextInters " << edgesData[tNumMapWidth].nextInters << "\n";
    //std::cout << " tNumMapWidth " <<tNumMapWidth << " nextInters " << edgesData[tNumMapWidth].nextInters << "\n";

  
    edgeDescToLaneMapNumSP.insert(std::make_pair(x.second, tNumMapWidth));
    laneMapNumToEdgeDescSP.insert(std::make_pair(tNumMapWidth, x.second));

    std::tuple<abm::graph::vertex_t, abm::graph::vertex_t> edge_vertices = std::get<1>(x)->first;
    abm::graph::edge_id_t edge_id = graph_->edge_ids_[get<0>(edge_vertices)][get<1>(edge_vertices)];
    if (edge_id >= edgeIdToLaneMapNum.size()){
      printf("edge_id exceeds upper bound. Edge_id: %i  vector size: %i\n",
              edge_id, edgeIdToLaneMapNum.size());
    }
    if (edge_id < 0){
      printf("edge_id exceeds lower bound\n");
    }
    edgeIdToLaneMapNum[edge_id] = tNumMapWidth;

    tNumMapWidth += numLanes * numWidthNeeded;
    tNumLanes += numLanes;
    //std::cout << "tNumMapWidth = " << tNumMapWidth << "\n";
    edge_count++;
  }

  if (edge_count != graph_->nedges()){
    printf("Error! Edge count does not match number of edges in graph.\n");
    printf("Edge count: %i  |   Number of edges in graph: %i\n", edge_count, graph_->nedges());
  }
  
  edgesData.resize(tNumMapWidth);

  if (LANE_DEBUG) {
    printf("Num edges %d Num Lanes %d Num Lanes Width %d Max Leng %f Max num lanes %d\n",
        edge_count, tNumLanes, tNumMapWidth, maxLength, maxNumLanes);
  }
    //printf("edgesData size = %d\n", edgesData.size());

  // 2. RESIZE LANE MAP

  printf("Total Memory %d\n", kMaxMapWidthM * tNumMapWidth * 2);
  laneMap.resize(kMaxMapWidthM * tNumMapWidth * 2); // 2: to have two maps.
  memset(laneMap.data(), -1, laneMap.size()*sizeof(unsigned char)); //

  //////////////////////////////////////////////////////////
  // GENERATE INTERSECTION INFO
  RoadGraph::roadGraphVertexIter_BI vi, viEnd;
  RoadGraph::in_roadGraphEdgeIter_BI Iei, Iei_end;
  RoadGraph::out_roadGraphEdgeIter_BI Oei, Oei_end;
  //intersections.resize(boost::num_vertices(inRoadGraph.myRoadGraph_BI));//as many as vertices
  intersections.resize(graph_->vertex_edges_.size());//as many as vertices
  //std::cout << "intersections size = " << intersections.size() << "\n";
  trafficLights.assign(tNumMapWidth, 0);
  //trafficLights.resize(tNumMapWidth); // we could use tNumLanes but then the edge number would not match and we would need to add logic.
  //memset(trafficLights.data(), 0, trafficLights.size()*sizeof(uchar));

  int index = 0;
  for (const auto& vertex : graph_->vertex_edges_) {
    //std::cout << "GET<0> VERTEX = " << std::get<0>(vertex) << "\n";
    //intersections[std::get<0>(vertex)].state = 0;
    //intersections[std::get<0>(vertex)].nextEvent = 0.0f;
    //intersections[std::get<0>(vertex)].totalInOutEdges = vertex.second.size();
    intersections[std::get<0>(vertex)].nextEvent = 0.0f;
    intersections[std::get<0>(vertex)].totalInOutEdges = vertex.second.size();
    if (intersections[std::get<0>(vertex)].totalInOutEdges <= 0) {
      printf("Vertex without in/out edges\n");
      continue;
    }

  if (intersections[std::get<0>(vertex)].totalInOutEdges >= 20) {
      printf("Vertex with more than 20 in/out edges\n");
      continue;
    }
    index++;

    
  //sort by angle
    QVector3D referenceVector(0, 1, 0);
    QVector3D p0, p1;
    //std::vector<std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI, float>> edgeAngleOut;
    std::vector<std::pair<std::shared_ptr<abm::Graph::Edge>, float>> edgeAngleOut;
    int numOutEdges = 0;

    float angleRef = atan2(referenceVector.y(), referenceVector.x());

    for (const auto& edge : graph_->vertex_out_edges_[std::get<0>(vertex)]) {
      //if (inRoadGraph.myRoadGraph_BI[*Oei].numberOfLanes == 0) { continue; }
      if (edge->second.lanes == 0) { continue; }

      p0 = graph_->vertices_data_[edge->first.first];
      p1 = graph_->vertices_data_[edge->first.second];
      

      QVector3D edgeDir = (p1 - p0).normalized();
      float angle = angleRef - atan2(edgeDir.y(), edgeDir.x());

      edgeAngleOut.push_back(std::make_pair(edge, angle));

      if (edgeDescToLaneMapNumSP.find(edge) == edgeDescToLaneMapNumSP.end()) {
        std::cout << "p0 = " << edge->first.first << "\n";
        std::cout << "p1 = " << edge->first.second << "\n";
        printf("->ERROR OUT\n");//edge desc not found in map
      }

      numOutEdges++;
      //edgeAngleOut.push_back(std::make_pair(edge_pair.first,angle));
    }
    
   
    std::vector<std::pair<std::shared_ptr<abm::Graph::Edge>, float>> edgeAngleIn;
    int numInEdges = 0;

    for (const auto& edge : graph_->vertex_in_edges_[std::get<0>(vertex)]) {
      if (edge->second.lanes == 0) { continue; }

      p0 = graph_->vertices_data_[edge->first.first];
      p1 = graph_->vertices_data_[edge->first.second];

      QVector3D edgeDir = (p0 - p1).normalized();
      float angle = angleRef - atan2(edgeDir.y(), edgeDir.x());
      
      edgeAngleIn.push_back(std::make_pair(edge, angle));

      if (edgeDescToLaneMapNumSP.find(edge) == edgeDescToLaneMapNumSP.end()) {
        printf("->ERROR IN\n");//edge desc not found in map
        continue;
      }

      numInEdges++;
    }

    intersections[std::get<0>(vertex)].totalInOutEdges = numOutEdges + numInEdges;
    //save in sorterd way as lane number
    if (edgeAngleOut.size() > 0) {
      std::sort(edgeAngleOut.begin(), edgeAngleOut.end(), compareSecondPartTupleCSP);
    }

    if (edgeAngleIn.size() > 0) {
      std::sort(edgeAngleIn.begin(), edgeAngleIn.end(), compareSecondPartTupleCSP);
    }
    //!!!!
    int outCount = 0;
    int inCount = 0;
    int totalCount = 0;

    // Use intersections[std::get<0>(vertex)] = blah blah to set those values for each vertex
    // Intersection data:
    //  Store the edges that go in or out of this intersection
    //  Said edges will be sorted by angle
    //  
    //      0xFF00 0000 Num lines
    //      0x0080 0000 in out (one bit)
    //      0x007F FFFF Edge number
    for (int iter = 0; iter < edgeAngleOut.size() + edgeAngleIn.size(); iter++) {
      if ((outCount < edgeAngleOut.size() && inCount < edgeAngleIn.size() && 
           edgeAngleOut[outCount] <= edgeAngleIn[inCount]) ||
          (outCount < edgeAngleOut.size() && inCount >= edgeAngleIn.size())) {
        assert(edgeDescToLaneMapNumSP[edgeAngleOut[outCount].first] < 0x007fffff && "Edge number is too high");
        intersections[std::get<0>(vertex)].edge[totalCount] = edgeDescToLaneMapNumSP[edgeAngleOut[outCount].first];
        intersections[std::get<0>(vertex)].edge[totalCount] |= (edgesData[intersections[std::get<0>(vertex)].edge[totalCount]].numLines << 24); //put the number of lines in each edge
        intersections[std::get<0>(vertex)].edge[totalCount] |= kMaskOutEdge; // 0x000000 mask to define out edge
        //std::cout << "edgeDesc " << edgeDescToLaneMapNumSP[edgeAngleOut[outCount].first] << "\n";
        outCount++;
      } else {
        assert(edgeDescToLaneMapNumSP[edgeAngleIn[inCount].first] < 0x007fffff && "Edge number is too high");
        intersections[std::get<0>(vertex)].edge[totalCount] = edgeDescToLaneMapNumSP[edgeAngleIn[inCount].first];
        intersections[std::get<0>(vertex)].edge[totalCount] |= (edgesData[intersections[std::get<0>(vertex)].edge[totalCount]].numLines << 24); //put the number of lines in each edge
        intersections[std::get<0>(vertex)].edge[totalCount] |= kMaskInEdge; // 0x800000 mask to define in edge
        inCount++;
      }

      totalCount++;
    }
    //std::cout << "outCount = " << outCount << " inCount = " << inCount << "\n";


    if (totalCount != intersections[std::get<0>(vertex)].totalInOutEdges) {
      printf("Error totalCount!=intersections[vertex].totalInOutEdges %d %d\n",
             totalCount, intersections[std::get<0>(vertex)].totalInOutEdges);
    }
  }
}

void B18TrafficLaneMap::createLaneMap(
    const RoadGraph &inRoadGraph,
    std::vector<uchar> &laneMap,
    std::vector<B18EdgeData> &edgesData,
    std::vector<B18IntersectionData> &intersections,
    std::vector<uchar> &trafficLights,
    std::map<uint, RoadGraph::roadGraphEdgeDesc_BI> &laneMapNumToEdgeDesc,
    std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum) {
  //////////////////////////////////////////////////////////
  // GENERATE LANE MAP
  if (LANE_DEBUG) {
    printf("  >> createLaneMap\n");
  }

  // 1. Cretae edgesData and find requires sizes.
  RoadGraph::roadGraphEdgeIter_BI ei, ei_end;
  int edge_count = 0;
  int tNumMapWidth = 0;
  int tNumLanes = 0;

  edgesData.resize(boost::num_edges(inRoadGraph.myRoadGraph_BI) * 4); //4 to make sure it fits

  edgeDescToLaneMapNum.clear();
  laneMapNumToEdgeDesc.clear();

  ////////////////////////////////
  // Check distribution of street length
  float binLength = 1.0f;//1km
  int numBins = 31 / binLength;//maxlength is 26km
  std::vector<int> bins(numBins, 0);
  for (boost::tie(ei, ei_end) = boost::edges(inRoadGraph.myRoadGraph_BI); ei != ei_end; ++ei) {
    const float metersLength = inRoadGraph.myRoadGraph_BI[*ei].edgeLength;
    const int binN = (metersLength / 1000.0f) / binLength;
    assert(0 <= binN && binN < numBins && "Edge over max length");
    bins[binN]++;
  }
  for (int binN = 0; binN < bins.size(); binN++) {
    printf("%.0fkm, %d\n", (binN * binLength+1.0f), bins[binN]);
  }

  /////////////////////////////////
  // Create EdgeData
  // Instead of having maxWidth (for b18 would have been 26km), we define a max width for the map and we wrap down.
  float maxLength = 0;
  int maxNumLanes = 0;
  for (boost::tie(ei, ei_end) = boost::edges(inRoadGraph.myRoadGraph_BI); ei != ei_end; ++ei) {
    const int numLanes = inRoadGraph.myRoadGraph_BI[*ei].numberOfLanes;

    if (numLanes == 0) { continue; }

    edgesData[tNumMapWidth].length = inRoadGraph.myRoadGraph_BI[*ei].edgeLength;
    edgesData[tNumMapWidth].maxSpeedMperSec = inRoadGraph.myRoadGraph_BI[*ei].maxSpeedMperSec;

    if (maxLength < edgesData[tNumMapWidth].length) { maxLength = edgesData[tNumMapWidth].length; }
    if (maxNumLanes < numLanes) { maxNumLanes = numLanes; }

    const int numWidthNeeded = ceil(edgesData[tNumMapWidth].length / kMaxMapWidthM);
    edgesData[tNumMapWidth].numLines = numLanes;
    edgesData[tNumMapWidth].nextInters = boost::target(*ei, inRoadGraph.myRoadGraph_BI);

    edgeDescToLaneMapNum.insert(std::make_pair(*ei, tNumMapWidth));
    laneMapNumToEdgeDesc.insert(std::make_pair(tNumMapWidth, *ei));

    tNumMapWidth += numLanes * numWidthNeeded;
    tNumLanes += numLanes;
    edge_count++;
  }

  edgesData.resize(tNumMapWidth);

  if (LANE_DEBUG) {
    printf("Num edges %d Num Lanes %d Num Lanes Width %d Max Leng %f Max num lanes %d\n",
        edge_count, tNumLanes, tNumMapWidth, maxLength, maxNumLanes);
  }

  // 2. RESIZE LANE MAP

  printf("Total Memory %d\n", kMaxMapWidthM * tNumMapWidth * 2);
  laneMap.resize(kMaxMapWidthM * tNumMapWidth * 2); // 2: to have two maps.
  memset(laneMap.data(), -1, laneMap.size()*sizeof(unsigned char)); //

  //////////////////////////////////////////////////////////
  // GENERATE INTERSECTION INFO
  RoadGraph::roadGraphVertexIter_BI vi, viEnd;
  RoadGraph::in_roadGraphEdgeIter_BI Iei, Iei_end;
  RoadGraph::out_roadGraphEdgeIter_BI Oei, Oei_end;
  intersections.resize(boost::num_vertices(inRoadGraph.myRoadGraph_BI));//as many as vertices
  trafficLights.assign(tNumMapWidth, 0);
  //trafficLights.resize(tNumMapWidth); // we could use tNumLanes but then the edge number would not match and we would need to add logic.
  //memset(trafficLights.data(), 0, trafficLights.size()*sizeof(uchar));

  for (boost::tie(vi, viEnd) = boost::vertices(inRoadGraph.myRoadGraph_BI); vi != viEnd; ++vi) {
    intersections[*vi].state = 0;
    intersections[*vi].nextEvent = 0.0f;
    intersections[*vi].totalInOutEdges = boost::degree(*vi, inRoadGraph.myRoadGraph_BI);
    if (intersections[*vi].totalInOutEdges <= 0) {
      printf("Vertex without in/out edges\n");
      continue;
    }

    if (intersections[*vi].totalInOutEdges >= 20) {
      printf("Vertex with more than 20 in/out edges\n");
      continue;
    }

    //sort by angle
    QVector3D referenceVector(0, 1, 0);
    QVector3D p0, p1;
    std::vector<std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI, float>> edgeAngleOut;
    int numOutEdges = 0;

    float angleRef = atan2(referenceVector.y(), referenceVector.x());

    for (boost::tie(Oei, Oei_end) = boost::out_edges(*vi, inRoadGraph.myRoadGraph_BI);
        Oei != Oei_end; ++Oei) {
      if (inRoadGraph.myRoadGraph_BI[*Oei].numberOfLanes == 0) { continue; }
      p0 = inRoadGraph.myRoadGraph_BI[boost::source(*Oei, inRoadGraph.myRoadGraph_BI)].pt;
      p1 = inRoadGraph.myRoadGraph_BI[boost::target(*Oei, inRoadGraph.myRoadGraph_BI)].pt;
      QVector3D edgeDir = (p1 - p0).normalized();
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

    for (boost::tie(Iei, Iei_end) = boost::in_edges(*vi, inRoadGraph.myRoadGraph_BI);
        Iei != Iei_end; ++Iei) {
      if (inRoadGraph.myRoadGraph_BI[*Iei].numberOfLanes == 0) { continue; }
      p0 = inRoadGraph.myRoadGraph_BI[boost::source(*Iei, inRoadGraph.myRoadGraph_BI)].pt;
      p1 = inRoadGraph.myRoadGraph_BI[boost::target(*Iei, inRoadGraph.myRoadGraph_BI)].pt;
      QVector3D edgeDir = (p0 - p1).normalized();
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

    // Intersection data:
    //  Store the edges that go in or out of this intersection
    //  Said edges will be sorted by angle
    //  
    //      0xFF00 0000 Num lines
    //      0x0080 0000 in out (one bit)
    //      0x007F FFFF Edge number
    for (int iter = 0; iter < edgeAngleOut.size() + edgeAngleIn.size(); iter++) {
      if ((outCount < edgeAngleOut.size() && inCount < edgeAngleIn.size() && 
           edgeAngleOut[outCount].second <= edgeAngleIn[inCount].second) ||
          (outCount < edgeAngleOut.size() && inCount >= edgeAngleIn.size())) {
        assert(edgeDescToLaneMapNum[edgeAngleOut[outCount].first] < 0x007fffff && "Edge number is too high");
        intersections[*vi].edge[totalCount] = edgeDescToLaneMapNum[edgeAngleOut[outCount].first];
        intersections[*vi].edge[totalCount] |= (edgesData[intersections[*vi].edge[totalCount]].numLines << 24); //put the number of lines in each edge
        intersections[*vi].edge[totalCount] |= kMaskOutEdge; // 0x000000 mask to define out edge
        outCount++;
      } else {
        assert(edgeDescToLaneMapNum[edgeAngleIn[inCount].first] < 0x007fffff && "Edge number is too high");
        intersections[*vi].edge[totalCount] = edgeDescToLaneMapNum[edgeAngleIn[inCount].first];
        intersections[*vi].edge[totalCount] |= (edgesData[intersections[*vi].edge[totalCount]].numLines << 24); //put the number of lines in each edge
        intersections[*vi].edge[totalCount] |= kMaskInEdge; // 0x800000 mask to define in edge
        inCount++;
      }

      totalCount++;
    }


    if (totalCount != intersections[*vi].totalInOutEdges) {
      printf("Error totalCount!=intersections[*vi].totalInOutEdges %d %d\n",
             totalCount, intersections[*vi].totalInOutEdges);
    }
  }
}//

void B18TrafficLaneMap::resetIntersections(std::vector<B18IntersectionData>
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
