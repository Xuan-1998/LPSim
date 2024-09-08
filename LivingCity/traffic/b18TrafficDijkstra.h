/************************************************************************************************
*
*		LC Project - B18 Dijkstra sortest pah.
*
*
*		@desc Class that finds the path for each Person
*		@author igaciad
*
************************************************************************************************/

#ifndef LC_B18_TRAFFIC_DIKKSTRA_H
#define LC_B18_TRAFFIC_DIKKSTRA_H

#include "../misctools/misctools.h"

#include "b18TrafficPerson.h"
#include "RoadGraph/roadGraph.h"

namespace LC {
class B18TrafficDijstra {

 public:

  static void calculateSeveralPeopleRoute(LC::RoadGraph::roadBGLGraph_BI &roadGraph,
                                   std::vector<B18TrafficPerson> &trafficPersonVec,
                                   std::vector<uint>& indexPathVec,
                                   uint &currIndexPath,
                                   std::vector<uint>& peopleStartInInter,
                                   std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum);
  // MULTI
  static void generateRoutesMulti(LC::RoadGraph::roadBGLGraph_BI &roadGraph,
                           std::vector<B18TrafficPerson> &trafficPersonVec,
                           std::vector<uint>& indexPathVec,
                           std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum,
                           int weigthMode = 0, float sample = 1.0f);
};
}

#endif  // LC_B18_TRAFFIC_DIKKSTRA_H
