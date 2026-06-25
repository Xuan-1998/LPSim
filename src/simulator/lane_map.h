/************************************************************************************************
*
*		LC Project - B18 Traffic lane map
*
*
*		@desc Class that contains the structure of the lane maps
*		@author igaciad
*
************************************************************************************************/

#ifndef LC_B18_TRAFFIC_LANEMAP_H
#define LC_B18_TRAFFIC_LANEMAP_H


#include "../misctools/misctools.h"
#include "RoadGraph/roadGraph.h"
#include "b18EdgeData.h"
#include "traffic/sp/graph.h"
#include "traffic/sp/config.h"

namespace LC {


class B18TrafficLaneMap {
 public:
   B18TrafficLaneMap();
   ~B18TrafficLaneMap();

  void createLaneMap(const RoadGraph &inRoadGraph, std::vector<uchar> &laneMap,
      std::vector<B18EdgeData> &edgesData, std::vector<B18IntersectionData> &intersections,
      std::vector<uchar> &trafficLights, std::map<uint,
      RoadGraph::roadGraphEdgeDesc_BI> &laneMapNumToEdgeDesc,
      std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum);

  void createLaneMapSP(const std::shared_ptr<abm::Graph>& graph_, std::vector<uchar> &laneMap,
      std::vector<B18EdgeData> &edgesData, std::vector<B18IntersectionData> &intersections,
      std::vector<uchar> &trafficLights, 
      std::map<uint, std::shared_ptr<abm::Graph::Edge>> &laneMapNumToEdgeDescSP,
      std::map<std::shared_ptr<abm::Graph::Edge>, uint> &edgeDescToLaneMapNumSP,
      std::vector<uint> &edgeIdToLaneMapNum);

void createLaneMapSP_n(int ngpus, const std::vector<int>vertexIdToPar,
    const std::shared_ptr<abm::Graph>& graph_, 
    std::vector<uchar> &laneMap,std::vector<uchar> laneMap_n[],
    std::vector<B18EdgeData> &edgesData, std::vector<B18EdgeData> edgesData_n[], 
    std::vector<B18IntersectionData> &intersections, std::vector<B18IntersectionData> intersections_n[],
    std::vector<uchar> &trafficLights, std::vector<uchar> trafficLights_n[], 
    std::map<uint, std::shared_ptr<abm::Graph::Edge>> &laneMapNumToEdgeDescSP,std::map<uint, std::shared_ptr<abm::Graph::Edge>> laneMapNumToEdgeDescSP_n[],
    std::map<std::shared_ptr<abm::Graph::Edge>, uint> &edgeDescToLaneMapNumSP,std::map<std::shared_ptr<abm::Graph::Edge>, uint> edgeDescToLaneMapNumSP_n[],
    std::vector<uint> &edgeIdToLaneMapNum, std::vector<uint> edgeIdToLaneMapNum_n[],
    std::map<uint, uint> LaneIdToLaneIdInGpu[]);

  void resetIntersections(std::vector<B18IntersectionData> &intersections,
                          std::vector<uchar> &trafficLights);

  
};
}

#endif  // LC_B18_TRAFFIC_LANEMAP_H
