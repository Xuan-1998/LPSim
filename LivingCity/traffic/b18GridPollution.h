/************************************************************************************************
*
*		LC Project - Grid Pollution
*
*		@desc Class that calculates and render pollution
*		@author igaciad
*
************************************************************************************************/

#ifndef LC_B18_GRID_POLLUTION_H
#define LC_B18_GRID_POLLUTION_H

#include "b18TrafficPerson.h"
#include "RoadGraph/roadGraph.h"


namespace LC {

class LCUrbanMain;

class B18GridPollution {


 public:
   B18GridPollution();
   ~B18GridPollution() {}

  float gridSize;
  int gridNumSide;

  void initPollution(LCUrbanMain *clientMain);
  int initialized;

  void addValueToGrid(float currTime,
                      std::vector<B18TrafficPerson> &trafficPersonVec,
                      std::vector<uint> &indexPathVec,
                      RoadGraph *simRoadGraph,
                      LCUrbanMain *clientMain,
                      std::map<uint, RoadGraph::roadGraphEdgeDesc_BI> &laneMapNumToEdgeDesc);
  void renderPollution(int valueToRender);

  std::vector<std::vector<float>> gridTimeValues;
  std::vector<float> lastPersonValue;
  std::vector<float> timeStamp;
  float maxValue;
  LCUrbanMain *clientMain;

  void saveToFile(QString fileName);
  void loadSimSave(QString fileName, LCUrbanMain *clientMain);
};
}

#endif  // LC_B18_GRID_POLLUTION_H
