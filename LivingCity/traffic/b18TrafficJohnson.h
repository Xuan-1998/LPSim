/************************************************************************************************
*		@desc Class that finds the path for each person using Johnsons
*		@author igaciad
************************************************************************************************/
#ifndef LC_B18_TRAFFIC_JOHNSON_H
#define LC_B18_TRAFFIC_JOHNSON_H

#include "b18TrafficPerson.h"
#include "../RoadGraph/roadGraph.h"

namespace LC {
class B18TrafficJohnson {

 public:

  static void generateRoutes(
      LC::RoadGraph::roadBGLGraph_BI &roadGraph,
      std::vector<B18TrafficPerson> &trafficPersonVec,
      std::vector<uint>& indexPathVec,
      std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum,
      int weigthMode = 0,
      float sample = 1.0f);
};
}

#endif  // LC_B18_TRAFFIC_JOHNSON_H
