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

#ifndef LC_CUDA_TRAFFIC_SIMULATOR_H
#define LC_CUDA_TRAFFIC_SIMULATOR_H

#include "../misctools/misctools.h"

#include "cudaPmTrafficPersonJob.h"
#include "cudaTrafficLaneMap.h"
#include "cudaTrafficPersonShortestPath.h"

#include "../VBOPeopleJobInfoLayer.h"
#include "../VBORenderManager.h"

#include "cudaGridPollution.h"


namespace LC {

class LCUrbanMain;
//class VBORenderManager;

class CUDATrafficSimulatorRender {
 public:
  std::vector<QVector3D> positions;
  std::vector<QVector3D> directions;
  std::vector<bool> goodPoints;
  int indexToRead;

  bool getInterpolated(bool goodPoint, QVector3D newPos, QVector3D newDir,
                       QVector3D &interPos, QVector3D &interDir);
};//

class CUDATrafficLightRender {
 public:
  std::vector<uchar> trafficLight;
  int indexToRead;

  void getInterpolated(uchar newTrafficLight, uchar &interTrafficLight);
};//


class CUDATrafficSimulator {

 public:
  CUDATrafficSimulator();
  ~CUDATrafficSimulator();

  // init data
  RoadGraph *simRoadGraph;
  LCUrbanMain* clientMain;

  PeopleJobInfoLayers simPeopleJobInfoLayers;

  float deltaTime;
  float cellSize;
  ushort maxWidth;
  int numberPeople;
  int threadNumber;
  float avgTravelTime;


  bool initialized;
  void initSimulator(float deltaTime, float cellSize, RoadGraph *geoRoadGraph,
                     int numberPeople, PeopleJobInfoLayers &peopleJobInfoLayers,
                     LCUrbanMain *urbanMain);

  //PM
  CUDAPMTrafficPersonJob cudaPMTrafficPersonJob;
  CUDATrafficLaneMap cudaTrafficLaneMap;
  CUDATrafficPersonShortestPath cudaTrafficPersonShortestPath;

  void simulateInCPU_MultiPass(int numOfPasses,
                               bool genTrafficPersonJobBool = true);
  void simulateInCPU_Onepass();


  void simulateInCPU();
  void simulateInGPU();

  // Lanes
  std::vector<uchar> laneMap;
  std::vector<edgeData> edgesData;
  std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> edgeDescToLaneMapNum;
  std::map<uint, RoadGraph::roadGraphEdgeDesc_BI> laneMapNumToEdgeDesc;
  void createLaneMap();

  // car path
  void generateCarPaths();

  // People
  std::vector<CUDATrafficPerson> trafficPersonVec;
  void generateTrafficPersonJob();
  void resetPeopleJobANDintersections();

  // Traffic lights
  std::vector<uchar> trafficLights;
  std::vector<intersectionData> intersections;

  // measurements
  std::vector<float> accSpeedPerLinePerTimeInterval;
  std::vector<float> numVehPerLinePerTimeInterval;

  void calculateAndDisplayTrafficDensity();
  void calculateAndDisplayTrafficDensity(std::vector<float>
                                         &accSpeedPerLinePerTimeInterval,
                                         std::vector<float> &numVehPerLinePerTimeInterval,
                                         std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum,
                                         std::map<uint, RoadGraph::roadGraphEdgeDesc_BI> &laneMapNumToEdgeDesc,
                                         int tNumLanes);

  void render(VBORenderManager &rendManager);
  std::vector<CUDATrafficSimulatorRender> cudaTrafficSimulatorRender;
  std::vector<CUDATrafficLightRender> cudaTrafficLightRender;
  // pollution
  CUDAGridPollution cudaGridPollution;
};
}

#endif
