#include "pollution.h"

namespace LC {

B18GridPollution::B18GridPollution() : initialized(0), clientMain(nullptr) {}

void B18GridPollution::initPollution(void *_clientMain) {
  clientMain = _clientMain;
}

void B18GridPollution::addValueToGrid(float currTime,
    std::vector<B18TrafficVehicle> &trafficPersonVec,
    std::vector<uint> &indexPathVec,
    RoadGraph *simRoadGraph,
    void *_clientMain,
    std::map<uint, RoadGraph::roadGraphEdgeDesc_BI> &laneMapNumToEdgeDesc) {
  // Pollution computation disabled in headless mode (requires GUI)
}

}
