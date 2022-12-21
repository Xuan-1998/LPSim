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
*		@desc Class that contains the traffic simulator
*		@author igaciad
************************************************************************************************/

#pragma once

#define BOOST_TT_HAS_OPERATOR_HPP_INCLUDED
#include "bPMTrafficPerson.h"
#include "../RoadGraph/roadGraph.h"
#include "bEdgeIntersectionData.h"

// simulation
#include <qobject.h>
#include <qthread.h>
#include <qmutex.h>
#include "bCPUTrafficThread.h"

namespace LC {

class VBORenderManager;
class LCUrbanMain;

class BTrafficSimulator : public QObject {
  Q_OBJECT
 public:

  BTrafficSimulator();
  ~BTrafficSimulator();

  // init data
  LCUrbanMain *clientMain;
  RoadGraph *simRoadGraph;
  void init(RoadGraph *roadGraph, LCUrbanMain *clientMain);

  // People
  BTrafficPeople people;
  void createRandomPeople(float startTime, float endTime, int numberPeople,
                    PeopleJobInfoLayers &peopleJobInfoLayers);
  void createB2018People(float startTime, float endTime);

  SimulationSt simulationSt;

  bool initialized;
  int state;//1 simulating 0 no simulation

  void clearSimulator();

  // Lanes
  std::vector<unsigned long> laneMapL[2];
  BEdgesData edgesData;
  BIntersectionsData intersec;
  BStopsData stops;
  BTrafficLightData trafficLights;

  std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> edgeDescToLaneMapNum;
  std::map<uint, RoadGraph::roadGraphEdgeDesc_BI> laneMapNumToEdgeDesc;
  void createLaneMap();
  void createIntersections();

  // Routes
  void generateCarDijstra();
  void generateCarJohnson();
  ushort maxWidthL;
  //std::vector<ushort> nextEdgeM;


  // Render
  void printCurrentState();
  void render(VBORenderManager &rendManager);
  void renderRoutes(VBORenderManager &rendManger);
  bool routesChanged;//flag to see whether regenerate routes render

  // Simulate GPU
  void simulateGPU(float startTime, float endTime);

  // Simulate CPU
  void simulateCPU(float startTime, float endTime);
  int numThreadsFinished;
  int numThreads;
  std::vector<BCPUTrafficThread *> threads;
  QMutex mutex;
 public slots:
  void threadFinish(int);
  void tFinish();
  void msgString(QString msg);
  /*int threadNumber;
  float avgTravelTime;



  void initSimulator(float deltaTime,float cellSize,RoadGraph* geoRoadGraph,int numberPeople,PeopleJobInfoLayers& peopleJobInfoLayers,LCUrbanMain* urbanMain);

  //PM
  CUDAPMTrafficPersonJob cudaPMTrafficPersonJob;
  CUDATrafficLaneMap cudaTrafficLaneMap;
  CUDATrafficPersonShortestPath cudaTrafficPersonShortestPath;

  void simulateInCPU_MultiPass(int numOfPasses,bool genTrafficPersonJobBool=true);
  void simulateInCPU_Onepass();


  void simulateInCPU();
  void simulateInGPU();



  // car path
  void generateCarPaths();


  void generateTrafficPersonJob();
  void resetPeopleJobANDintersections();

  // Traffic lights
  std::vector<uchar> trafficLights;
  std::vector<intersectionData> intersections;

  // measurements
  std::vector<float> accSpeedPerLinePerTimeInterval;
  std::vector<float> numVehPerLinePerTimeInterval;

  void calculateAndDisplayTrafficDensity();
  void calculateAndDisplayTrafficDensity(std::vector<float>& accSpeedPerLinePerTimeInterval,std::vector<float>& numVehPerLinePerTimeInterval,std::map<RoadGraph::roadGraphEdgeDesc_BI,uint>& edgeDescToLaneMapNum,std::map<uint,RoadGraph::roadGraphEdgeDesc_BI>& laneMapNumToEdgeDesc,int tNumLanes);

  void render(VBORenderManager& rendManager);
  std::vector<CUDATrafficSimulatorRender> cudaTrafficSimulatorRender;
  std::vector<CUDATrafficLightRender> cudaTrafficLightRender;
  // pollution
  CUDAGridPollution cudaGridPollution;*/
};
}

