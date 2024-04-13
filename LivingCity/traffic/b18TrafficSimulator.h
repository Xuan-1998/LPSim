/************************************************************************************************
*		@desc Class that contains the traffic simulator b2018.
*		@author igaciad
************************************************************************************************/
#pragma once
#ifndef LC_B18_TRAFFIC_SIMULATOR_H
#define LC_B18_TRAFFIC_SIMULATOR_H

#include "../misctools/misctools.h"

#include "b18TrafficOD.h"
#include "b18TrafficLaneMap.h"

#ifdef B18_RUN_WITH_GUI
#include "../VBOPeopleJobInfoLayer.h"
#include "../VBORenderManager.h"
#endif

#include "b18GridPollution.h"
#include "accessibility.h"


namespace LC {

class LCUrbanMain;

class B18TrafficSimulatorRender {
 public:
  std::vector<QVector3D> positions;
  std::vector<QVector3D> directions;
  std::vector<bool> goodPoints;
  int indexToRead;

  bool getInterpolated(bool goodPoint, QVector3D newPos, QVector3D newDir,
                       QVector3D &interPos, QVector3D &interDir);
};//

class B18TrafficLightRender {
 public:
  std::vector<uchar> trafficLight;
  int indexToRead;

  void getInterpolated(uchar newTrafficLight, uchar &interTrafficLight);
};//


class B18TrafficSimulator {

 public:
  B18TrafficSimulator(float deltaTime, RoadGraph *geoRoadGraph, const parameters & simParameters, LCUrbanMain *urbanMain = nullptr);
  ~B18TrafficSimulator();

  // init data
  RoadGraph *simRoadGraph;
  LCUrbanMain *clientMain;
  parameters simParameters;

  float deltaTime;
  int threadNumber;
  float avgTravelTime;

  //PM
  B18TrafficOD b18TrafficOD;
  B18TrafficLaneMap b18TrafficLaneMap;

  void printFullProgressBar(void) const;

  void printProgressBar(const float progress) const;

  void simulateInCPU_MultiPass(int numOfPasses,
                               float startTimeH, float endTimeH, bool useJohnsonRouting);
  void simulateInCPU_Onepass(float startTimeH, float endTimeH,
                             bool useJohnsonRouting);
  void simulateInCPU(float startTimeH, float endTimeH);

  void updateEdgeImpedances(const std::shared_ptr<abm::Graph>& graph_, const int increment_index);
  
  void simulateInGPU(const int numOfPasses, const float startTimeH, const float endTimeH,
    const bool useJohnsonRouting, const bool useSP, const std::shared_ptr<abm::Graph>& graph_,
    const parameters & simParameters, const int rerouteIncrementMins,
    const std::vector<std::array<abm::graph::vertex_t, 2>>& all_od_pairs,
    const std::vector<float>& dep_times, const std::string & networkPathSP);

  // Lanes
  std::vector<uint> edgeIdToLaneMapNum;
  std::vector<uchar> laneMap;
  std::vector<B18EdgeData> edgesData;
  std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> edgeDescToLaneMapNum;
  std::map<uint, RoadGraph::roadGraphEdgeDesc_BI> laneMapNumToEdgeDesc;
  std::map<uint, std::shared_ptr<abm::Graph::Edge>> laneMapNumToEdgeDescSP;
  std::map<std::shared_ptr<abm::Graph::Edge>, uint> edgeDescToLaneMapNumSP;
  void createLaneMap();
  void createLaneMapSP(const std::shared_ptr<abm::Graph>& graph_);

  // car path
  void generateCarPaths(bool useJohnsonRouting);

  // People
  std::vector<B18TrafficPerson> trafficPersonVec;
  std::vector<uint> indexPathVec;
  std::vector<uint> indexPathVecOrder;

#ifdef B18_RUN_WITH_GUI
  void createRandomPeople(float startTime, float endTime, int numberPeople,
                          PeopleJobInfoLayers &peopleJobInfoLayers);
#endif
  void createB2018People(float startTime, float endTime, int limitNumPeople, bool addRandomPeople, bool useSP);
  
  void createB2018PeopleSP(
    const float startTime, const float endTime, const int limitNumPeople, const bool addRandomPeople,
    const std::shared_ptr<abm::Graph>& graph_, const std::vector<float> & dep_times);

  void resetPeopleJobANDintersections();
  void saveODToFile() {}; // TODO
  void loadODFromFile() {};


  // Traffic lights
  std::vector<uchar> trafficLights;
  std::vector<B18IntersectionData> intersections;

  // measurements
  std::vector<float> accSpeedPerLinePerTimeInterval;
  std::vector<float> numVehPerLinePerTimeInterval;

  void calculateAndDisplayTrafficDensity(int numOfPass);
  void savePeopleAndRoutes(int numOfPass);
  void savePeopleAndRoutesSP(
    const std::vector<personPath>& allPaths,
    const std::vector<uint>& allPathsInEdgesCUDAFormat,
    const std::vector<uint>& edgeIdToLaneMapNum,
    const int numOfPass,
    const std::shared_ptr<abm::Graph>& graph_,
    const int start_time, const int end_time,
    const std::vector<LC::B18EdgeData>& edgesData);
    
#ifdef B18_RUN_WITH_GUI
  void render(VBORenderManager &rendManager);
#endif
  std::vector<B18TrafficSimulatorRender> b18TrafficSimulatorRender;
  std::vector<B18TrafficLightRender> b18TrafficLightRender;

  // pollution
  B18GridPollution gridPollution;
};
}

#endif  // LC_B18_TRAFFIC_SIMULATOR_H
