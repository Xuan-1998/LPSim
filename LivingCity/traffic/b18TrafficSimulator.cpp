/************************************************************************************************
*
*		LPSim Development - B18 trafficSimulator
*
*		@author xuan, jiaying, luze
*
************************************************************************************************/

#pragma once
#include "b18TrafficSimulator.h"
#include <assert.h>

#include "src/benchmarker.h"

#include "../global.h"
#ifdef B18_RUN_WITH_GUI
#include "../LC_GLWidget3D.h"
#include "../LC_UrbanMain.h"
#endif
#include <thread>

#include "b18TrafficDijkstra.h"
#include "b18TrafficJohnson.h"
#include "b18TrafficSP.h"
#include "b18CUDA_trafficSimulator.h"
#include "roadGraphB2018Loader.h"
#include <thread>
#include "accessibility.h"
#include <math.h>


#define DEBUG_TRAFFIC 0
#define DEBUG_SIMULATOR 0
#define DEBUG_T_LIGHT 0

#ifdef __linux__
#include <unistd.h>

void printPercentageMemoryUsed() {
  // TODO
}
#elif _WIN32
#include <windows.h>
void printPercentageMemoryUsed() {
  MEMORYSTATUSEX status;
  status.dwLength = sizeof(status);
  GlobalMemoryStatusEx(&status);
  printf("** Memory usage %ld%% --> Free %I64d MB\n", status.dwMemoryLoad,
         status.ullAvailPhys / (1024 * 1024));
}
#endif
namespace LC {

const float intersectionClearance = 7.8f;
const bool calculatePollution = true;

B18TrafficSimulator::B18TrafficSimulator(float _deltaTime, RoadGraph *originalRoadGraph,
    const parameters & inputSimParameters, LCUrbanMain *urbanMain) : deltaTime(_deltaTime), simParameters(inputSimParameters),
    b18TrafficOD (B18TrafficOD(simParameters)) {
  simRoadGraph = new RoadGraph(*originalRoadGraph);
  clientMain = urbanMain;
}

B18TrafficSimulator::~B18TrafficSimulator() {
  delete simRoadGraph;
}

#ifdef B18_RUN_WITH_GUI
void B18TrafficSimulator::createRandomPeople(float startTime, float endTime,
    int numberPeople,
    PeopleJobInfoLayers &peopleJobInfoLayers) {
  b18TrafficOD.resetTrafficVehicleJob(trafficVehicleVec);
  b18TrafficOD.createRandomPeople(numberPeople, startTime, endTime,
                                  trafficVehicleVec,
                                  peopleJobInfoLayers, simRoadGraph->myRoadGraph_BI);
}//
#endif

void B18TrafficSimulator::createB2018People(float startTime, float endTime, int limitNumPeople,
    bool addRandomPeople, bool useSP) {
  b18TrafficOD.resetTrafficVehicleJob(trafficVehicleVec);
  b18TrafficOD.loadB18TrafficPeople(startTime, endTime, trafficVehicleVec,
      simRoadGraph->myRoadGraph_BI, limitNumPeople, addRandomPeople);
}

void B18TrafficSimulator::createB2018PeopleSP(
  const float startTime, const float endTime, const int limitNumPeople, const bool addRandomPeople,
  const std::shared_ptr<abm::Graph>& graph_, const std::vector<float> & dep_times) {
  b18TrafficOD.resetTrafficVehicleJob(trafficVehicleVec);
  b18TrafficOD.loadB18TrafficPeopleSP(startTime, endTime, trafficVehicleVec,
      graph_, limitNumPeople, addRandomPeople, dep_times);
}

void B18TrafficSimulator::resetPeopleJobANDintersections() {
  b18TrafficOD.resetTrafficVehicleJob(trafficVehicleVec);
  b18TrafficLaneMap.resetIntersections(intersections, trafficLights);
}//


void B18TrafficSimulator::createLaneMap() {
	b18TrafficLaneMap.createLaneMap(*simRoadGraph, laneMap, edgesData, intersections, trafficLights, laneMapNumToEdgeDesc, edgeDescToLaneMapNum);
}//

void B18TrafficSimulator::createLaneMapSP(const std::shared_ptr<abm::Graph>& graph_) { //
	b18TrafficLaneMap.createLaneMapSP(graph_, laneMap, edgesData, intersections, trafficLights, laneMapNumToEdgeDescSP, edgeDescToLaneMapNumSP, edgeIdToLaneMapNum);
}

void B18TrafficSimulator::createLaneMapSP_n(int ngpus, const std::vector<int>& vertexIdToPar, const std::shared_ptr<abm::Graph>& graph_,
std::vector<uchar> laneMap_n[],std::vector<B18EdgeData> edgesData_n[],std::vector<B18IntersectionData> intersections_n[],std::vector<uchar> trafficLights_n[],std::map<uint, std::shared_ptr<abm::Graph::Edge>> laneMapNumToEdgeDescSP_n[],std::map<std::shared_ptr<abm::Graph::Edge>, uint> edgeDescToLaneMapNumSP_n[],
std::vector<uint> edgeIdToLaneMapNum_n[],std::map<uint, uint> laneIdToLaneIdInGpu[]) { //
	b18TrafficLaneMap.createLaneMapSP_n (ngpus, vertexIdToPar, graph_, laneMap, laneMap_n, edgesData, edgesData_n, intersections, intersections_n, trafficLights, trafficLights_n, laneMapNumToEdgeDescSP, laneMapNumToEdgeDescSP_n, edgeDescToLaneMapNumSP, edgeDescToLaneMapNumSP_n, edgeIdToLaneMapNum,edgeIdToLaneMapNum_n,laneIdToLaneIdInGpu);
}

void B18TrafficSimulator::generateCarPaths(bool useJohnsonRouting) { //
  if (useJohnsonRouting) {
    printf("***generateCarPaths Start generateRoute Johnson\n");
    B18TrafficJohnson::generateRoutes(simRoadGraph->myRoadGraph_BI,
                                      trafficVehicleVec, indexPathVec, edgeDescToLaneMapNum, 0);
  } else {
    printf("***generateCarPaths Start generateRoutesMulti Disktra\n");
    B18TrafficDijstra::generateRoutesMulti(simRoadGraph->myRoadGraph_BI,
                                           trafficVehicleVec, indexPathVec, edgeDescToLaneMapNum, 0);
  }

}//

void B18TrafficSimulator::printFullProgressBar() const {
  float progress = 1;
  int barWidth = 70;
  std::cout << "[";
  int pos = barWidth * progress;
  for (int i = 0; i < barWidth; ++i) {
    std::cout << "=";
  }
  std::cout << "] 100%" << std::endl;
}

void B18TrafficSimulator::printProgressBar(const float progress) const {
  int barWidth = 70;
  std::cout << "[";
  int pos = barWidth * progress;
  for (int i = 0; i < barWidth; ++i) {
      if (i < pos) std::cout << "=";
      else if (i == pos) std::cout << ">";
      else std::cout << " ";
  }
  std::cout << "] " << int(progress * 100.0);
  if (fabs(progress - 1) < FLOAT_EPSILON){
    std::cout << std::endl;
  } else {
    std::cout << "%\r";
    std::cout.flush();
  }
}
//TODO: updateEdgeImpedances_n
void B18TrafficSimulator::updateEdgeImpedances(
  const std::shared_ptr<abm::Graph>& graph_,
  const int increment_index) {

  int index = 0;
  int avg_edge_vel_size = graph_->edges_.size();
  std::vector<float> avg_edge_vel(avg_edge_vel_size);
  for (auto const& x : graph_->edges_) {
    int ind = edgeDescToLaneMapNumSP[x.second];
    float new_impedance;
    if (edgesData.at(ind).curr_cum_vel != 0) {
      avg_edge_vel[index] = edgesData.at(ind).curr_cum_vel / edgesData.at(ind).curr_iter_num_cars;// * 2.23694;
      new_impedance = edgesData.at(ind).length / avg_edge_vel[index];
      auto cum_vel = edgesData.at(ind).curr_cum_vel;
      auto num_cars = edgesData.at(ind).curr_iter_num_cars;
      auto avg_edge_vel_index = avg_edge_vel[index];
    } else {
      abm::EdgeProperties anEdgeProperties = x.second->second;
      new_impedance = edgesData.at(ind).length / anEdgeProperties.max_speed_limit_mps; // no one transited - default impedance
    }
    auto vertex_from = std::get<0>(std::get<0>(x));
    auto vertex_to = std::get<1>(std::get<0>(x));
    assert(new_impedance > 0);
    graph_->update_edge(vertex_from, vertex_to, new_impedance);

    index++;
  }

  //save avg_edge_vel vector to file
  Benchmarker allEdgesVelBenchmark("all_edges_vel_" + std::to_string(increment_index), true);
  allEdgesVelBenchmark.startMeasuring();
  std::string name = "./all_edges_vel_" + std::to_string(increment_index) + ".txt";
  std::ofstream output_file(name);
  std::ostream_iterator<float> output_iterator(output_file, "\n");
  std::copy(avg_edge_vel.begin(), avg_edge_vel.end(), output_iterator);
  allEdgesVelBenchmark.stopAndEndBenchmark();
}
std::string personPathToString(const personPath& path) {
    std::stringstream ss;
    ss << path.person_id;
    for (int vertex : path.pathInVertexes) {
        ss << ", " << vertex;
    }
    return ss.str();
}
void savePaths(const std::vector<personPath>& paths, const std::string& filename) {
    std::ofstream file(filename);
    for (const auto& path : paths) {
        file << personPathToString(path) << "\n";
    }
}
//////////////////////////////////////////////////
// GPU
//////////////////////////////////////////////////
void B18TrafficSimulator::simulateInGPU(const int ngpus, const int numOfPasses, const float startTimeH, const float endTimeH,
    const bool useJohnsonRouting, const bool useSP, const std::shared_ptr<abm::Graph>& graph_,
    const parameters & simParameters,
    const int rerouteIncrementMins, const std::vector<std::array<abm::graph::vertex_t, 2>> & all_od_pairs,
    const std::vector<float> & dep_times, const std::string & networkPathSP, const std::vector<int>& vertexIdToPar) {
  
  std::vector<uint> edgeIdToLaneMapNum_n[ngpus];
  std::vector<uchar> laneMap_n[ngpus];
  std::vector<B18EdgeData> edgesData_n[ngpus];
  std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> edgeDescToLaneMapNum_n[ngpus];
  std::map<uint, std::shared_ptr<abm::Graph::Edge>> laneMapNumToEdgeDescSP_n[ngpus];
  std::map<std::shared_ptr<abm::Graph::Edge>, uint> edgeDescToLaneMapNumSP_n[ngpus];
  std::map<uint, uint> laneIdToLaneIdInGpu[ngpus];
  std::vector<uint> indexPathVec_n[ngpus];
  std::vector<uchar> trafficLights_n[ngpus];
  std::vector<B18IntersectionData> intersections_n[ngpus];
  
  
  
  
  std::vector<personPath> allPathsInVertexes;
      
  Benchmarker laneMapCreation("Lane_Map_creation", true);
   
  laneMapCreation.startMeasuring();
  if (useSP) {
	  // createLaneMapSP(graph_);
    // std::vector<int> partitions[ngpus]= {{0,1,3},{2,4,5,6,7,8,9}};
    // vertexIdToPar eg:[0,0,1,1,0,0,1,0]
    
    // Fill the first half with 0s
    // std::fill(vertexIdToPar.begin(), vertexIdToPar.begin() + graph_->vertex_edges_.size() / 2, 0);

    // Fill the second half with 1s
    // std::fill(vertexIdToPar.begin() + graph_->vertex_edges_.size() / 2, vertexIdToPar.end(), 1);
      //check vertexIdToPar legal
  auto maxElement = std::max_element(vertexIdToPar.begin(), vertexIdToPar.end());
  if (maxElement != vertexIdToPar.end()){
    if(*maxElement>=ngpus){
      printf("NUM_GPUS is %d but vertexIdToPar has element %d\n",ngpus,*maxElement);
      exit(1);
    }
  }
  else{
    printf("vertexIdToPar error\n");
    exit(1);
  }

    createLaneMapSP_n(ngpus,vertexIdToPar, graph_,laneMap_n, edgesData_n, intersections_n, trafficLights_n, laneMapNumToEdgeDescSP_n, edgeDescToLaneMapNumSP_n,edgeIdToLaneMapNum_n, laneIdToLaneIdInGpu);
  } else {
	  createLaneMap();
  }
  laneMapCreation.stopAndEndBenchmark();


  QTime pathTimer;
  pathTimer.start();

  int weigthMode;
  float peoplePathSampling[] = {1.0f, 1.0f, 0.5f, 0.25f, 0.12f, 0.67f};

  for (int nP = 0; nP < numOfPasses; nP++) {
    Benchmarker roadGenerationBench("Road generation");
    Benchmarker initCudaBench("Init Cuda step");
    Benchmarker shortestPathBench("Shortest path step");

    roadGenerationBench.startMeasuring();
    weigthMode = 1;
    if (nP == 0) {
      weigthMode = 0; //first time run normal weights
    }

    if (useJohnsonRouting) {
      shortestPathBench.startMeasuring();	    
      printf("***Start generateRoute Johnson\n");
      B18TrafficJohnson::generateRoutes(simRoadGraph->myRoadGraph_BI, trafficVehicleVec,
          indexPathVec, edgeDescToLaneMapNum, weigthMode, peoplePathSampling[nP]);
      shortestPathBench.stopAndEndBenchmark();
    } else if (useSP) {
      //
    } else {
      printf("***Start generateRoutesMulti Disktra\n");
      B18TrafficDijstra::generateRoutesMulti(simRoadGraph->myRoadGraph_BI,
                                             trafficVehicleVec, indexPathVec, edgeDescToLaneMapNum, weigthMode,
                                             peoplePathSampling[nP]);
    }

    roadGenerationBench.stopAndEndBenchmark();

    Benchmarker edgeOutputting("Edge outputting");
    edgeOutputting.startMeasuring();

    int index = 0;
    std::vector<uint> u(graph_->edges_.size());
    std::vector<uint> v(graph_->edges_.size());
    for (auto const& x : graph_->edges_) {
      abm::graph::vertex_t vertex_u = std::get<0>(std::get<0>(x));
      abm::graph::vertex_t vertex_v = std::get<1>(std::get<0>(x));
      u[index] = graph_->nodeIndex_to_osmid_[vertex_u];
      v[index] = graph_->nodeIndex_to_osmid_[vertex_v];
      index++;
    }

    //save avg_edge_vel vector to file
    std::string name_u = "./edges_u.txt";
    std::ofstream output_file_u(name_u);
    std::ostream_iterator<uint> output_iterator_u(output_file_u, "\n");
    std::copy(u.begin(), u.end(), output_iterator_u);

    //save avg_edge_vel vector to file
    std::string name_v = "./edges_v.txt";
    std::ofstream output_file_v(name_v);
    std::ostream_iterator<uint> output_iterator_v(output_file_v, "\n");
    std::copy(v.begin(), v.end(), output_iterator_v);
    std::cout << "Wrote edge vertices files!" << std::endl;
    edgeOutputting.stopAndEndBenchmark();


    /////////////////////////////////////
    // 1. Init Cuda
    initCudaBench.startMeasuring();
    bool firstInitialization = (nP == 0);

    std::cout << "Traffic person vec size = " << trafficVehicleVec.size() << std::endl;
    std::cout << "Index path vec size = " << indexPathVec.size() << std::endl;
    std::cout << "EdgesData size = " << edgesData.size() << std::endl;
    std::cout << "LaneMap size = " << laneMap.size() << std::endl;
    std::cout << "Intersections size = " << intersections.size() << std::endl;
    // Command Line Option Processing


    // const int ngpus = find_int_arg(argc, argv, "-ngpus", 4);
    // const int ngpus = 2;
    // printf("ngpus %i\n", ngpus);
    // int* gpus = new int[ngpus];
    // for(int i = 0; i < ngpus; i++){
    //     gpus[i] = i;
    // }

    // create streams to make event recording easier to handle
    // cudaStream_t streams[ngpus];
    // for(int i = 0; i < ngpus; i++){
    //     // create stream for gpu i
    //     cudaStreamCreate( &streams[i]);
    // }

    // b18InitCUDA(firstInitialization, trafficVehicleVec, indexPathVec, edgesData,
    //     laneMap, trafficLights, intersections, startTimeH, endTimeH,
    //     accSpeedPerLinePerTimeInterval, numVehPerLinePerTimeInterval, deltaTime);
    b18InitCUDA_n(ngpus, firstInitialization, vertexIdToPar, graph_->max_edge_id_,laneIdToLaneIdInGpu, trafficVehicleVec, indexPathVec_n, edgesData_n,
        laneMap_n, trafficLights_n, intersections_n, startTimeH, endTimeH,
        accSpeedPerLinePerTimeInterval, numVehPerLinePerTimeInterval, deltaTime);

    initCudaBench.stopAndEndBenchmark();

    float startTimeSecs = startTimeH * 3600.0f; //7.0f
    float endTimeSecs = endTimeH * 3600.0f; //8.0f//10.0f

    float currentTime = 23.99f * 3600.0f;

    for (int p = 0; p < trafficVehicleVec.size(); p++) {
      if (currentTime > trafficVehicleVec[p].time_departure) {
        currentTime = trafficVehicleVec[p].time_departure;
      }
    }
      
    int numInt = currentTime / deltaTime;//floor
    currentTime = numInt * deltaTime;
    uint steps = 0;
    steps = (currentTime - startTimeSecs) / deltaTime;

    // start as early as starting time
    if (currentTime < startTimeSecs) { //as early as the starting time
      currentTime = startTimeSecs;
    }

    QTime timer;
    //G::global()["cuda_render_displaylist_staticRoadsBuildings"] = 1;//display list
    timer.start();
    // Reset people to inactive.
    b18ResetPeopleLanesCUDA(trafficVehicleVec.size());
    // 2. Execute
    printf("First time_departure %f\n", currentTime);

    int numBlocks = ceil(trafficVehicleVec.size() / 384.0f);
    int threadsPerBlock = 384;
    std::cout << "Running trafficSimulation with the following configuration:"  << std::endl
              << ">  Number of people: " << trafficVehicleVec.size() << std::endl
              << ">  Number of blocks: " << numBlocks << std::endl
              << ">  Number of threads per block: " << threadsPerBlock
              << std::endl;

    int increment = 7200;
    int increment_index = 0;
    std::cerr
      << "Running main loop from " << (startTimeSecs / 3600.0f)
      << " to " << (endTimeSecs / 3600.0f)
      << " with " << trafficVehicleVec.size() << " vehicle..."
      << std::endl;

    for (int i = 0; i < trafficVehicleVec.size(); ++i){
      trafficVehicleVec[i].indexPathInit = INIT_EDGE_INDEX_NOT_SET;
    }
      
    std::cerr << "Starting simulation ..." << std::endl;
    std::vector<uint> allPathsInEdgesCUDAFormat;
    while (currentTime < endTimeSecs) {
      updateEdgeImpedances(graph_, increment_index);
      
      float currentBatchStartTimeSecs = startTimeSecs + increment_index * rerouteIncrementMins * 60;
      float currentBatchEndTimeSecs = startTimeSecs + (increment_index + 1) * rerouteIncrementMins * 60;

      auto currentBatchPathsInVertexes = B18TrafficSP::RoutingWrapper(all_od_pairs, graph_, dep_times,
                                            currentBatchStartTimeSecs, currentBatchEndTimeSecs,
                                            (const int) increment_index, trafficVehicleVec);
    
      allPathsInVertexes.insert(std::end(allPathsInVertexes), std::begin(currentBatchPathsInVertexes), std::end(currentBatchPathsInVertexes));

      allPathsInEdgesCUDAFormat = B18TrafficSP::convertPathsToCUDAFormat(
          allPathsInVertexes, edgeIdToLaneMapNum, graph_, trafficVehicleVec);
//    std::cout << "For person 61507, indexPathInit is " << trafficVehicleVec[61507].indexPathInit<<" allPathsInEdgesCUDAFormat[]="<<allPathsInEdgesCUDAFormat[trafficVehicleVec[61507].indexPathInit]<<std::endl;
//    QFile indexPathInitFile("indexPathInit01.csv");
//    if (indexPathInitFile.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
//    std::cout << "> Saving indexPathInitFile... (size " << trafficVehicleVec.size() << ")" << std::endl;
//    QTextStream streamP(&indexPathInitFile);
//    for (int p = 0; p < trafficVehicleVec.size(); p++) {
//       streamP << trafficVehicleVec[p].id<<","<<trafficVehicleVec[p].indexPathInit<<"\n";
//    }
//    indexPathInitFile.close();
//    }

//   std::ofstream edgeIdToLaneFile("./edgeIdToLaneMapNum00_.txt");
//   std::ostream_iterator<float> output_iterator_edgeIdToLane(edgeIdToLaneFile, "\n");
//   std::copy(edgeIdToLaneMapNum.begin(), edgeIdToLaneMapNum.end(), output_iterator_edgeIdToLane);
//   savePaths(allPathsInVertexes, "pathsInVertexes00_.txt");
      Benchmarker benchmarkb18updateStructuresCUDA("b18updateStructuresCUDA");
      benchmarkb18updateStructuresCUDA.startMeasuring();
      // b18updateStructuresCUDA(trafficVehicleVec, allPathsInEdgesCUDAFormat, edgesData);
      benchmarkb18updateStructuresCUDA.stopAndEndBenchmark();

      Benchmarker benchmarkb18updateStructuresCUDA_n("b18updateStructuresCUDA_n");
      benchmarkb18updateStructuresCUDA_n.startMeasuring();
      b18updateStructuresCUDA_n(vertexIdToPar,trafficVehicleVec, allPathsInEdgesCUDAFormat, edgesData_n,allPathsInVertexes);
      benchmarkb18updateStructuresCUDA_n.stopAndEndBenchmark();

      Benchmarker microsimulationInGPU("Microsimulation_in_GPU_batch_" + to_string(increment_index), true);
      microsimulationInGPU.startMeasuring();

      currentTime = currentBatchStartTimeSecs;
      std::cout << ">> startTimeSecs " << currentBatchStartTimeSecs << std::endl
                << ">> currentTime " << currentTime << std::endl
                << ">> endTimeSecs " << currentBatchEndTimeSecs << std::endl
                << ">> deltaTime " << deltaTime << std::endl
                << "simulating from " << currentTime
                << " secs to " << currentBatchEndTimeSecs
                << " secs." << std::endl;

      float progress = 0;
      uint32_t* intersections_size_n = new uint32_t[ngpus]();
      for(int i=0;i<ngpus;i++){
        intersections_size_n[i]=intersections_n[i].size();
      }
      while (currentTime < currentBatchEndTimeSecs) {
        printProgressBar(progress);
        float nextMilestone = currentBatchStartTimeSecs + (progress + 0.1) * (currentBatchEndTimeSecs - currentBatchStartTimeSecs);
        while(currentTime < nextMilestone) {
          
          b18SimulateTrafficCUDA(currentTime, trafficVehicleVec.size(),
                              intersections_size_n, deltaTime, simParameters, numBlocks, threadsPerBlock);
           
          currentTime += deltaTime;
          // if(currentTime>18030)break;
        }
        progress += 0.1;
        // if(currentTime>18030)break;
      }
      // if(currentTime>18030)break;
      b18GetDataCUDA(trafficVehicleVec, edgesData);
      b18GetSampleTrafficCUDA(accSpeedPerLinePerTimeInterval,
                            numVehPerLinePerTimeInterval);
      /*
      for (int i = 0; i < trafficVehicleVec.size(); i++) {
        if ((trafficVehicleVec[i].time_departure < currentBatchEndTimeSecs && trafficVehicleVec[i].active == 0) ||
            (isgreaterequal(trafficVehicleVec[i].time_departure, currentBatchEndTimeSecs) && trafficVehicleVec[i].active != 0)) {
          std::string errorMessage =
              "Person " + std::to_string(i) + " has active state " +
              std::to_string(trafficVehicleVec[i].active) + " and dep time " +
              std::to_string(trafficVehicleVec[i].time_departure);
          throw std::runtime_error(errorMessage);
        }
      }*/

      printFullProgressBar();

      microsimulationInGPU.stopAndEndBenchmark();
      increment_index++;

      #ifdef B18_RUN_WITH_GUI

      if (clientMain != nullptr &&
          clientMain->ui.b18RenderSimulationCheckBox->isChecked()) {
        steps++;

        if (steps % clientMain->ui.b18RenderStepSpinBox->value() ==
            0) { //each "steps" steps, render

          Benchmarker getDataCudatrafficPersonAndEdgesData("Get data trafficVehicleVec");
          b18GetDataCUDA(trafficVehicleVec); // trafficLights
          getDataCudatrafficPersonAndEdgesData.startMeasuring();
          getDataCudatrafficPersonAndEdgesData.stopAndEndBenchmark();
          QString timeT;

          int timeH = currentTime / 3600.0f;
          int timeM = (currentTime - timeH * 3600.0f) / 60.0f;
          timeT.sprintf("%d:%02d", timeH, timeM);
          clientMain->ui.b18TimeLCD->display(timeT);

          QApplication::processEvents();
          QApplication::processEvents();
          clientMain->glWidget3D->updateGL();
          QApplication::processEvents();
        }
      }
      #endif
    }

    // 3. Finish
    // b18GetDataCUDA(trafficVehicleVec, edgesData);
    // debug
    float totalNumSteps = 0;
    float totalCO = 0;

    for (int p = 0; p < trafficVehicleVec.size(); p++) {
      totalNumSteps += trafficVehicleVec[p].num_steps;
      totalCO += trafficVehicleVec[p].co;
    }

    avgTravelTime = (totalNumSteps * deltaTime) / (trafficVehicleVec.size() * 60.0f); //in min
    printf("Total num steps %.1f Avg %.2f min Avg CO %.2f\nSimulation time = %d ms\n",
            totalNumSteps, avgTravelTime, totalCO / trafficVehicleVec.size(),
            timer.elapsed());

    Benchmarker fileOutput("File_output", true);
    fileOutput.startMeasuring();
    savePeopleAndRoutesSP(allPathsInVertexes, allPathsInEdgesCUDAFormat, edgeIdToLaneMapNum, nP, graph_,
                        (int) startTimeH, (int) endTimeH, edgesData);
    fileOutput.stopAndEndBenchmark();
  }

  b18FinishCUDA();
  G::global()["cuda_render_displaylist_staticRoadsBuildings"] = 3;//kill display list

#ifdef B18_RUN_WITH_GUI

  if (clientMain != nullptr) {
    clientMain->glWidget3D->updateGL();
  }

#endif
}//


//////////////////////////////////////////////////
// CPU
//////////////////////////////////////////////////

// calculate if the car will turn left center or right (or the same)
void calculateLaneCarShouldBe(
  uint curEdgeLane,
  uint nextEdge,
  std::vector<B18IntersectionData> &intersections,
  uint edgeNextInters,
  ushort edgeNumLanes,
  ushort &initOKLanes,
  ushort &endOKLanes) {
  initOKLanes = 0;
  endOKLanes = edgeNumLanes;

  if (DEBUG_TRAFFIC == 1) {
    printf("curEdgeLane %05x nextEdge %05x\n", curEdgeLane, nextEdge);
  }

  if (DEBUG_TRAFFIC == 1) {
    for (int eN = 0; eN < intersections[edgeNextInters].totalInOutEdges; eN++) {
      printf("* procEdge %05x\n", intersections[edgeNextInters].edge[eN]);
    }
  }

  bool currentEdgeFound = false;
  bool exitFound = false;
  ushort numExitToTake = 0;
  ushort numExists = 0;

  for (int eN = intersections[edgeNextInters].totalInOutEdges - 1; eN >= 0;
       eN--) {  // clockwise
    uint procEdge = intersections[edgeNextInters].edge[eN];

    if ((procEdge & kMaskLaneMap) == curEdgeLane) { //current edge 0xFFFFF
      if (DEBUG_TRAFFIC == 1) {
        printf("CE procEdge %05x\n", procEdge);
      }

      currentEdgeFound = true;

      if (exitFound == false) {
        numExitToTake = 0;
      }

      continue;
    }


    if ((procEdge & kMaskInEdge) == 0x0) { //out edge 0x800000
      if (DEBUG_TRAFFIC == 1) {
        printf("   procEdge %05x\n", procEdge);
      }

      numExists++;

      if (currentEdgeFound == true) {
        numExitToTake++;
      }

      if (currentEdgeFound == false && exitFound == false) {
        numExitToTake++;
      }
    }

    if ((procEdge & kMaskInEdge) == nextEdge) {
      exitFound = true;
      currentEdgeFound = false;

      if (DEBUG_TRAFFIC == 1) {
        printf("NE procEdge %05x\n", procEdge);
      }
    }
  }

  if (DEBUG_TRAFFIC == 1) {
    printf("Num extis %u Num exit to take %u%\n", numExists, numExitToTake);
  }

  if (edgeNumLanes == 0) {
    printf("ERRRROR\n");
  }

  switch (edgeNumLanes) {
  /// ONE LANE
  case 1:
    initOKLanes = 0;
    endOKLanes = 1;
    break;

  /// TWO LANE
  case 2:
    switch (numExists) {
    case 1:
    case 2://all okay
      initOKLanes = 0;
      endOKLanes = 2;
      break;

    case 3:
      if (numExitToTake > 2) { //left
        initOKLanes = 0;
        endOKLanes = 1;
        break;
      }

      initOKLanes = 1;
      endOKLanes = 2;
      break;

    default:
      if (DEBUG_TRAFFIC == 1) {
        printf("2 defalt\n");
      }

      if (numExitToTake >= numExists - 1) {
        initOKLanes = 0;
        endOKLanes = 1;
        break;
      }

      initOKLanes = 1;
      endOKLanes = 2;
      break;
    }

    break;

  /// THREE LANE
  case 3:
    switch (numExists) {
    case 1:
    case 2://all okay
      initOKLanes = 0;
      endOKLanes = 3;
      break;

    case 3:
      if (numExitToTake > 2) { //left
        initOKLanes = 0;
        endOKLanes = 1;
        break;
      }

      initOKLanes = 1;
      endOKLanes = 3;
      break;

    default:
      if (numExitToTake >= numExists - 1) {
        initOKLanes = 0;
        endOKLanes = 1;
        break;
      }

      initOKLanes = 1;
      endOKLanes = 2;
      break;
    }

    break;

  case 4:
    switch (numExists) {
    case 1:
    case 2://all okay
      initOKLanes = 0;
      endOKLanes = 4;
      break;

    case 3:
      if (numExitToTake == 1) { //right
        initOKLanes = 3;
        endOKLanes = 4;
      }

      if (numExitToTake > 3) { //left
        initOKLanes = 0;
        endOKLanes = 1;
        break;
      }

      initOKLanes = 1;
      endOKLanes = 4;
      break;

    default:
      if (numExitToTake == 1) { //right
        initOKLanes = edgeNumLanes - 1;
        endOKLanes = edgeNumLanes;
      }

      if (numExitToTake >= numExists - 2) {
        initOKLanes = 0;
        endOKLanes = 2;
        break;
      }

      initOKLanes = 1; //also lane 2
      endOKLanes = edgeNumLanes;
    }

    break;

  default:
    switch (numExists) {
    case 1:
    case 2://all okay
      initOKLanes = 0;
      endOKLanes = edgeNumLanes;
      break;

    case 3:
      if (numExitToTake == 1) { //right
        initOKLanes = edgeNumLanes - 1;
        endOKLanes = edgeNumLanes;
      }

      if (numExitToTake > edgeNumLanes - 2) { //left
        initOKLanes = 0;
        endOKLanes = 2;
        break;
      }

      initOKLanes = 1;
      endOKLanes = edgeNumLanes;
      break;

    default:
      if (numExitToTake < 2) { //right
        initOKLanes = edgeNumLanes - 2;
        endOKLanes = edgeNumLanes;
      }

      if (numExitToTake >= numExists - 2) {
        initOKLanes = 0;
        endOKLanes = 2;
        break;
      }

      initOKLanes = 1; //also lane 2
      endOKLanes = edgeNumLanes - 1;
    }

    break;
  }
}//

void calculateGapsLC(
  uint mapToReadShift,
  std::vector<uchar> &laneMap,
  uchar trafficLightState,
  uint laneToCheck,
  ushort numLinesEdge,
  float posInMToCheck,
  float length,
  uchar &v_a,
  uchar &v_b,
  float &gap_a,
  float &gap_b) {
  ushort numOfCells = ceil(length);
  ushort initShift = ceil(posInMToCheck);
  uchar laneChar;
  bool found = false;

  // CHECK FORWARD
  //printf("initShift %u numOfCells %u\n",initShift,numOfCells);
  for (ushort b = initShift - 1; (b < numOfCells) &&
       (found == false);
       b++) {  //NOTE -1 to make sure there is none in at the same level
    // laneChar = laneMap[mapToReadShift + maxWidth * (laneToCheck) + b];
    const uint posToSample = mapToReadShift + kMaxMapWidthM * (laneToCheck + (((
                               int)(b / kMaxMapWidthM)) * numLinesEdge)) + b % kMaxMapWidthM;
    laneChar = laneMap[posToSample];

    if (laneChar != 0xFF) {
      gap_a = ((float)b - initShift); //m
      v_a = laneChar; //laneChar is in 3*ms (to save space in array)
      found = true;
      break;
    }
  }

  if (found == false) {
    if (trafficLightState == 0x00) { //red
      //found=true;
      gap_a = gap_b = 1000.0f; //force to change to the line without vehicle
      v_a = v_b = 0xFF;
      return;
    }
  }

  if (found == false) {
    gap_a = 1000.0f;
  }

  // CHECK BACKWARDS
  found = false;

  //printf("2initShift %u numOfCells %u\n",initShift,numOfCells);
  for (int b = initShift + 1; (b >= 0) &&
       (found == false);
       b--) {  // NOTE +1 to make sure there is none in at the same level
    //laneChar = laneMap[mapToReadShift + maxWidth * (laneToCheck) + b];
    const uint posToSample = mapToReadShift + kMaxMapWidthM * (laneToCheck + (((
                               int)(b / kMaxMapWidthM)) * numLinesEdge)) + b % kMaxMapWidthM;
    laneChar = laneMap[posToSample];

    if (laneChar != 0xFF) {
      gap_b = ((float)initShift - b); //m
      v_b = laneChar; //laneChar is in 3*ms (to save space in array)
      found = true;
      break;
    }
  }

  //printf("3initShift %u numOfCells %u\n",initShift,numOfCells);
  if (found == false) {
    gap_b = 1000.0f;
  }

}//

void simulateOnePersonCPU(
  uint p,
  float deltaTime,
  float currentTime,
  uint mapToReadShift,
  uint mapToWriteShift,
  std::vector<LC::B18TrafficVehicle> &trafficVehicleVec,
  std::vector<uint> &indexPathVec,
  std::vector<LC::B18EdgeData> &edgesData,
  std::vector<uchar> &laneMap,
  std::vector<B18IntersectionData> &intersections,
  std::vector<uchar> &trafficLights,
  const parameters & simParameters) {
  //if(DEBUG_TRAFFIC==1)printf("currentTime %f   0 Person: %d State %d Time Dep %f\n",currentTime,p,trafficVehicleVec[p].active, trafficVehicleVec[p].time_departure);
  ///////////////////////////////
  //2.0. check if finished
  if (trafficVehicleVec[p].active == 2) {
    return;
  }

  ///////////////////////////////
  //2.1. check if person should still wait or should start
  if (trafficVehicleVec[p].active == 0) {

    //printf("  1. Person: %d active==0\n",p);
    if (trafficVehicleVec[p].time_departure > currentTime) { //wait
      //1.1 just continue waiting
      //printf("   1.1 Person: %d wait\n",p);
      return;
    } else { //start
      //1.2 find first edge
      trafficVehicleVec[p].indexPathCurr =
        trafficVehicleVec[p].indexPathInit; // reset index.
      uint firstEdge = indexPathVec[trafficVehicleVec[p].indexPathCurr];

      if (firstEdge == -1) {
        trafficVehicleVec[p].active = 2;
        //printf("0xFFFF\n");
        return;
      }

      if (DEBUG_TRAFFIC == 1) {
        printf("   1.2 Person: %d TRY put in first edge\n", p, firstEdge);
      }

      //1.3 update person edgeData
      //if(DEBUG_TRAFFIC==1)printf("   1.3 Person: %d put in first edge %u\n",p,firstEdge);
      //printf("edgesData %d\n",edgesData);

      // COPY DATA FROM EDGE TO PERSON
      trafficVehicleVec[p].edgeNumLanes = edgesData[firstEdge].numLines;
      trafficVehicleVec[p].edgeNextInters = edgesData[firstEdge].nextInters;

      trafficVehicleVec[p].length = edgesData[firstEdge].length;
      trafficVehicleVec[p].maxSpeedMperSec = edgesData[firstEdge].maxSpeedMperSec;
      //printf("edgesData %.10f\n",edgesData[firstEdge].maxSpeedCellsPerDeltaTime);
      //1.4 try to place it in middle of edge
      ushort numOfCells = ceil(trafficVehicleVec[p].length);
      ushort initShift = (ushort)(0.5f *
                                  numOfCells); //number of cells it should be placed (half of road)

      uchar laneChar;
      bool placed = false;

      ushort numCellsEmptyToBePlaced = simParameters.s_0;
      ushort countEmptyCells = 0;

      for (ushort b = initShift; (b < numOfCells) && (placed == false); b++) {
        ushort lN = trafficVehicleVec[p].edgeNumLanes - 1; //just right LANE !!!!!!!
        laneChar = laneMap[mapToReadShift + kMaxMapWidthM * (firstEdge + lN) +
                                          b]; //get byte of edge (proper line)

        if (laneChar != 0xFF) {
          countEmptyCells = 0;
          continue;
        }

        countEmptyCells++;// ensure there is enough room to place the car

        if (countEmptyCells < numCellsEmptyToBePlaced) {
          continue;
        }

        trafficVehicleVec[p].numOfLaneInEdge = lN;
        trafficVehicleVec[p].posInLaneM = b; //m
        uchar vInMpS = (uchar)(trafficVehicleVec[p].v *
                               3); //speed in m/s *3 (to keep more precision
        laneMap[mapToWriteShift + kMaxMapWidthM * (firstEdge + lN) + b] = vInMpS;
        placed = true;
        //printf("Placed\n");
        break;
        //}
      }

      if (placed == false) { //not posible to start now
        return;
      }

      trafficVehicleVec[p].v = 0;
      trafficVehicleVec[p].LC_stateofLaneChanging = 0;

      //1.5 active car
      if (DEBUG_TRAFFIC == 1) {
        printf("   1.2 Person: %d PUT in first edge %u Pos %f of %f\n", p, firstEdge,
               trafficVehicleVec[p].posInLaneM, trafficVehicleVec[p].length);
      }

      trafficVehicleVec[p].active = 1;
      trafficVehicleVec[p].isInIntersection = 0;
      trafficVehicleVec[p].num_steps = 1;
      trafficVehicleVec[p].co = 0.0f;
      trafficVehicleVec[p].gas = 0.0f;
      //trafficVehicleVec[p].nextPathEdge++;//incremet so it continues in next edge
      // set up next edge info
      uint nextEdge = indexPathVec[trafficVehicleVec[p].indexPathCurr + 1];

      //trafficVehicleVec[p].nextEdge=nextEdge;
      if (nextEdge != -1) {
        trafficVehicleVec[p].nextEdgemaxSpeedMperSec =
          edgesData[nextEdge].maxSpeedMperSec;
        trafficVehicleVec[p].nextEdgeNumLanes = edgesData[nextEdge].numLines;
        trafficVehicleVec[p].nextEdgeNextInters = edgesData[nextEdge].nextInters;
        trafficVehicleVec[p].nextEdgeLength = edgesData[nextEdge].length;
        //trafficVehicleVec[p].nextPathEdge++;
        trafficVehicleVec[p].LC_initOKLanes = 0xFF;
        trafficVehicleVec[p].LC_endOKLanes = 0xFF;
      }

      return;
    }
  }

  if (DEBUG_TRAFFIC == 1) {
    printf("    2. Person: %d moving\n", p);
  }

  ///////////////////////////////
  //2. it is moving
  if (float(currentTime) == int(
        currentTime)) { // assuming deltatime = 0.5f --> each second
    trafficVehicleVec[p].num_steps++;
  }

  //2.1 try to move
  float numMToMove;
  bool getToNextEdge = false;
  bool nextVehicleIsATrafficLight = false;
  uint currentEdge = indexPathVec[trafficVehicleVec[p].indexPathCurr];
  uint nextEdge = indexPathVec[trafficVehicleVec[p].indexPathCurr + 1];

  if (DEBUG_TRAFFIC == 1) {
    printf("    2. Person: %d Try to move current Edge %u Next %u\n", p,
           currentEdge, nextEdge);
  }

  // www.vwi.tu-dresden.de/~treiber/MicroApplet/IDM.html
  // IDM
  float thirdTerm = 0;
  ///////////////////////////////////////////////////
  // 2.1.1 Find front car
  int numCellsCheck = std::max<float>(30.0f,
                                      trafficVehicleVec[p].v * deltaTime * 2); //30 or double of the speed*time

  // a) SAME LINE (BEFORE SIGNALING)
  bool found = false;
  bool noFirstInLaneBeforeSign =
    false; //use for stop control (just let 1st to pass)
  bool noFirstInLaneAfterSign =
    false; //use for stop control (just let 1st to pass)
  float s;
  float delta_v;
  uchar laneChar;
  ushort byteInLine = (ushort)floor(trafficVehicleVec[p].posInLaneM);
  ushort numOfCells = ceil((trafficVehicleVec[p].length - intersectionClearance));

  for (ushort b = byteInLine + 2; (b < numOfCells) && (found == false) &&
       (numCellsCheck > 0); b++, numCellsCheck--) {
    // laneChar = laneMap[mapToReadShift + maxWidth * (indexPathVec[trafficVehicleVec[p].indexPathCurr] + trafficVehicleVec[p].numOfLaneInEdge) + b];
    // ShiftRead + WIDTH * (width number * # edges + # laneInEdge) + b
    const uint posToSample = mapToReadShift + kMaxMapWidthM *
                             (indexPathVec[trafficVehicleVec[p].indexPathCurr] + (((int)(
                                   byteInLine / kMaxMapWidthM)) * trafficVehicleVec[p].edgeNumLanes) +
                              trafficVehicleVec[p].numOfLaneInEdge) + b % kMaxMapWidthM;
    laneChar = laneMap[posToSample];

    if (laneChar != 0xFF) {
      s = ((float)(b - byteInLine)); //m
      delta_v = trafficVehicleVec[p].v - (laneChar /
                                         3.0f); //laneChar is in 3*ms (to save space in array)
      found = true;
      noFirstInLaneBeforeSign = true;

      if (DEBUG_TRAFFIC == 1) {
        printf("    2. Person: %d BEFORE Intersection: first obstacle -> car\n", p);
      }

      break;
    }
  }

  // b) TRAFFIC LIGHT
  if (byteInLine < numOfCells && found == false &&
      numCellsCheck > 0) { //before traffic signaling (and not cell limited)
    if (trafficLights[currentEdge + trafficVehicleVec[p].numOfLaneInEdge] ==
        0x00) { //red
      s = ((float)(numOfCells - byteInLine)); //m
      delta_v = trafficVehicleVec[p].v - 0; //it should be treated as an obstacle
      nextVehicleIsATrafficLight = true;

      if (DEBUG_TRAFFIC == 1) {
        printf("    2. Person: %d first obstacle -> traffic light\n", p);
      }

      //printf("\nFOUND TL\n",s,delta_v);
      found = true;
    }
  }

  // c) SAME LINE (AFTER SIGNALING)
  for (ushort b = byteInLine + 2; (b < numOfCells) && (found == false) &&
       (numCellsCheck > 0); b++, numCellsCheck--) {
    // laneChar = laneMap[mapToReadShift + maxWidth * t(indexPathVec[rafficPersonVec[p].indexPathCurr] + trafficVehicleVec[p].numOfLaneInEdge) + b];
    const uint posToSample = mapToReadShift + kMaxMapWidthM *
                             (indexPathVec[trafficVehicleVec[p].indexPathCurr] + (((int)(
                                   byteInLine / kMaxMapWidthM)) * trafficVehicleVec[p].edgeNumLanes) +
                              trafficVehicleVec[p].numOfLaneInEdge) + b % kMaxMapWidthM;
    laneChar = laneMap[posToSample];

    if (laneChar != 0xFF) {
      s = ((float)(b - byteInLine)); //m
      delta_v = trafficVehicleVec[p].v - (laneChar /
                                         3.0f); //laneChar is in 3*ms (to save space in array)
      found = true;
      noFirstInLaneAfterSign = true;

      if (DEBUG_TRAFFIC == 1) {
        printf("    2. Person: %d AFTER Intersection: first obstacle -> car\n", p);
      }

      break;
    }
  }

  if (trafficLights[currentEdge + trafficVehicleVec[p].numOfLaneInEdge] == 0x0F &&
      numCellsCheck > 0) { //stop
    //check
    if (noFirstInLaneBeforeSign == false && byteInLine < numOfCells &&
        //first before traffic
        trafficVehicleVec[p].v == 0 && //stopped
        noFirstInLaneAfterSign ==
        false) { // noone after the traffic light (otherwise wait before stop) !! TODO also check the beginning of next edge

      trafficLights[currentEdge + trafficVehicleVec[p].numOfLaneInEdge] =
        0x00; //reset stop
      trafficVehicleVec[p].posInLaneM = ceil(numOfCells) + 1; //move magicly after stop

      if (DEBUG_TRAFFIC == 1) {
        printf("    2. Person: %d move after stop\n", p);
      }

    } else { //stop before STOP
      if (noFirstInLaneBeforeSign ==
          false) { //just update this if it was the first one before sign
        s = ((float)(numOfCells - byteInLine)); //m
        delta_v = trafficVehicleVec[p].v - 0; //it should be treated as an obstacle
        nextVehicleIsATrafficLight = true;
        found = true;

        if (DEBUG_TRAFFIC == 1) {
          printf("    2. Person: %d just update this if it was the first one before sign\n",
                 p);
        }
      }
    }
  }

  // NEXT LINE
  if (found == false && numCellsCheck > 0) { //check if in next line
    if ((nextEdge != -1) &&
        (trafficVehicleVec[p].edgeNextInters !=
         trafficVehicleVec[p].end_intersection)) { // we haven't arrived to destination (check next line)
      if (DEBUG_TRAFFIC == 1) {
        printf("    2. Person: NEXT LINE\n", p);
      }

      ushort nextEdgeLaneToBe = trafficVehicleVec[p].numOfLaneInEdge; //same lane

      //printf("trafficVehicleVec[p].numOfLaneInEdge %u\n",trafficVehicleVec[p].numOfLaneInEdge);
      if (nextEdgeLaneToBe >= trafficVehicleVec[p].nextEdgeNumLanes) {
        nextEdgeLaneToBe = trafficVehicleVec[p].nextEdgeNumLanes -
                           1; //change line if there are less roads
      }

      //printf("2trafficVehicleVec[p].numOfLaneInEdge %u\n",trafficVehicleVec[p].numOfLaneInEdge);
      ushort numOfCells = ceil(trafficVehicleVec[p].nextEdgeLength);

      for (ushort b = 0; (b < numOfCells) && (found == false) &&
           (numCellsCheck > 0); b++, numCellsCheck--) {
        //laneChar = laneMap[mapToReadShift + maxWidth * (nextEdge + nextEdgeLaneToBe) + b];
        const uint posToSample = mapToReadShift + kMaxMapWidthM *
                                 (nextEdge + nextEdgeLaneToBe) + b; // b18 not changed since we check first width
        laneChar = laneMap[posToSample];

        if (laneChar != 0xFF) {
          s = ((float)(b)); //m
          delta_v = trafficVehicleVec[p].v - (laneChar /
                                             3.0f);  // laneChar is in 3*ms (to save space in array)
          found = true;
          break;
        }
      }
    }
  }


  float s_star;

  if (found == true) { //car in front and slower than us
    // 2.1.2 calculate dv_dt
    s_star = simParameters.s_0 + std::max(0.0f,
                            (trafficVehicleVec[p].v * trafficVehicleVec[p].T + (trafficVehicleVec[p].v *
                                delta_v) / (2 * std::sqrt(trafficVehicleVec[p].a * trafficVehicleVec[p].b))));
    thirdTerm = std::pow(((s_star) / (s)), 2);
    //printf(">FOUND s_star %f thirdTerm %f!!!!\n",s_star,thirdTerm);
  }

  float dv_dt = trafficVehicleVec[p].a * (1.0f - std::pow((
      trafficVehicleVec[p].v / trafficVehicleVec[p].maxSpeedMperSec), 4) - thirdTerm);

  // 2.1.3 update values
  numMToMove = std::max(0.0f,
                        trafficVehicleVec[p].v * deltaTime + 0.5f * (dv_dt) * deltaTime * deltaTime);

  if (DEBUG_TRAFFIC == 1) {
    printf("v %f v0 %f a %f dv_dt %f MOVE %f\n", trafficVehicleVec[p].v,
           trafficVehicleVec[p].maxSpeedMperSec, trafficVehicleVec[p].a, dv_dt, numMToMove);
  }

  //printf("v %.10f v d %.10f\n",trafficVehicleVec[p].v,trafficVehicleVec[p].v+((dv_dt/(deltaTime)/deltaTime)));
  trafficVehicleVec[p].v += dv_dt * deltaTime;

  if (trafficVehicleVec[p].v < 0) {
    //printf("p %d v %f v0 %f a %f dv_dt %f s %f s_star %f MOVE %f\n",p,trafficVehicleVec[p].v,trafficVehicleVec[p].maxSpeedMperSec,trafficVehicleVec[p].a,dv_dt,s,s_star,numMToMove);
    trafficVehicleVec[p].v = 0;
    dv_dt = 0.0f;
  }

  if (calculatePollution &&
      ((float(currentTime) == int(
          currentTime)))) { // enabled and each second (assuming deltaTime 0.5f)
    // CO Calculation
    const float speedMph = trafficVehicleVec[p].v * 2.2369362920544; //mps to mph
    const float coStep = -0.064 + 0.0056 * speedMph + 0.00026 *
                         (speedMph - 50.0f) * (speedMph - 50.0f);

    if (coStep > 0) {
      // coStep *= deltaTime; // we just compute it each second
      trafficVehicleVec[p].co += coStep;
    }

    // Gas Consumption
    const float a = dv_dt;
    const float v = trafficVehicleVec[p].v; // in mps
    const float Pea = a > 0.0f ? (0.472f * 1.680f * a * a * v) : 0.0f;
    const float gasStep = 0.666f + 0.072f * (0.269f * v + 0.000672f *
                          (v * v * v) + 0.0171f * (v * v) + 1.680f * a * v + Pea);
    /*if (p == 0) {
      printf("Time %f --> a %.6f v %.6f\n", currentTime, a, v);
      printf("Time %f --> Consumption %.6f %.6f %.6f %.6f\n", currentTime, (0.269f*v + 0.000672f*(v*v*v)), (0.0171f*(v*v)), 1680.0f*a*v, Pea);
      printf("Time %f --> Consumption %f+0.072*%f --> %f\n\n", currentTime, 0.666f, (0.269f*v + 0.000672f*(v*v*v) + 0.0171f*(v*v) + 1680.0f*a*v + Pea), gasStep);
    }*/
    trafficVehicleVec[p].gas +=
      gasStep; // *= deltaTime // we just compute it each second

  }

  //////////////////////////////////////////////

  if (trafficVehicleVec[p].v == 0) { //if not moving not do anything else
    ushort posInLineCells = (ushort)(trafficVehicleVec[p].posInLaneM);
    //laneMap[mapToWriteShift + maxWidth * (currentEdge + trafficVehicleVec[p].numOfLaneInEdge) + posInLineCells] = 0;
    const uint posToSample = mapToWriteShift + kMaxMapWidthM * (currentEdge + (((
                               int)(posInLineCells / kMaxMapWidthM)) * trafficVehicleVec[p].edgeNumLanes) +
                             trafficVehicleVec[p].numOfLaneInEdge) + posInLineCells % kMaxMapWidthM;
    laneMap[posToSample] = 0;

    return;
  }

  //////////

  ///////////////////////////////
  // COLOR
  trafficVehicleVec[p].color = p << 8;
  //if (clientMain->ui.b18RenderSimulationCheckBox->isChecked()) {
  //if(G::global().getInt("cuda_carInfoRendering_type")==0){
  //qsrand(p);

  /*}
  if(G::global().getInt("cuda_carInfoRendering_type")==1){
      uchar c=(uchar)(255*trafficVehicleVec[p].v/15.0f);//84m/s is more than 300km/h
      trafficVehicleVec[p].color=(c<<24)|(c<<16)|(c<<8);
  }
  if(G::global().getInt("cuda_carInfoRendering_type")==2){
      uchar c=255*trafficVehicleVec[p].LC_stateofLaneChanging;
      trafficVehicleVec[p].color=(c<<24)|(c<<16)|(c<<8);

  }*/
  //}

  ////////////////////////////////

  if (DEBUG_TRAFFIC == 1) {
    printf("2 v(t+de) %f\n\n", trafficVehicleVec[p].v);
  }

  // STOP (check if it is a stop if it can go through)

  trafficVehicleVec[p].posInLaneM = trafficVehicleVec[p].posInLaneM + numMToMove;

  if (trafficVehicleVec[p].posInLaneM >
      trafficVehicleVec[p].length) { //reach intersection
    numMToMove = trafficVehicleVec[p].posInLaneM - trafficVehicleVec[p].length;
    getToNextEdge = true;
  } else { //does not research next intersection
    ////////////////////////////////////////////////////////
    // LANE CHANGING (happens when we are not reached the intersection)
    if (trafficVehicleVec[p].v > 3.0f && //at least 10km/h to try to change lane
        trafficVehicleVec[p].num_steps % 5 == 0 //just check every (5 steps) 5 seconds
       ) {
      //next thing is not a traffic light
      // skip if there is one lane (avoid to do this)
      // skip if it is the last edge
      if (nextVehicleIsATrafficLight == false &&
          trafficVehicleVec[p].edgeNumLanes > 1 && nextEdge != -1) {

        ////////////////////////////////////////////////////
        // LC 1 update lane changing status
        if (trafficVehicleVec[p].LC_stateofLaneChanging == 0) {
          // 2.2-exp((x-1)^2)
          float x = trafficVehicleVec[p].posInLaneM / trafficVehicleVec[p].length;

          if (x > 0.4f) { //just after 40% of the road
            float probabiltyMandatoryState = 2.2 - exp((x - 1) * (x - 1));

            if (((float)qrand() / RAND_MAX) < probabiltyMandatoryState) {
              trafficVehicleVec[p].LC_stateofLaneChanging = 1;
            }
          }

        }

        ////////////////////////////////////////////////////
        // LC 2 NOT MANDATORY STATE
        if (trafficVehicleVec[p].LC_stateofLaneChanging == 0) {
          //if(p==40)printf("LC v %f v0 %f a %f\n",trafficVehicleVec[p].v,trafficVehicleVec[p].maxSpeedMperSec*0.5f,dv_dt);
          // discretionary change: v slower than the current road limit and deccelerating and moving
          if ((trafficVehicleVec[p].v < (trafficVehicleVec[p].maxSpeedMperSec * 0.7f)) &&
              (dv_dt < 0) && trafficVehicleVec[p].v > 3.0f) {
            //printf(">>LANE CHANGE\n");

            //printf("LC 0 %u\n",trafficVehicleVec[p].numOfLaneInEdge);
            bool leftLane = trafficVehicleVec[p].numOfLaneInEdge >
                            0; //at least one lane on the left
            bool rightLane = trafficVehicleVec[p].numOfLaneInEdge <
                             trafficVehicleVec[p].edgeNumLanes - 1; //at least one lane

            if (leftLane == true && rightLane == true) {
              if (qrand() % 2 == 0) {
                leftLane = false;
              } else {
                rightLane = false;
              }
            }

            ushort laneToCheck;

            if (leftLane == true) {
              laneToCheck = trafficVehicleVec[p].numOfLaneInEdge - 1;
            } else {
              laneToCheck = trafficVehicleVec[p].numOfLaneInEdge + 1;
            }

            uchar v_a, v_b;
            float gap_a, gap_b;
            //printf("p %u LC 1 %u\n",p,laneToCheck);
            uchar trafficLightState = trafficLights[currentEdge +
                                                                trafficVehicleVec[p].numOfLaneInEdge];
            calculateGapsLC(mapToReadShift, laneMap, trafficLightState,
                            currentEdge + laneToCheck, trafficVehicleVec[p].edgeNumLanes,
                            trafficVehicleVec[p].posInLaneM,
                            trafficVehicleVec[p].length, v_a, v_b, gap_a, gap_b);

            //printf("LC 2 %u %u %f %f\n",v_a,v_b,gap_a,gap_b);
            if (gap_a == 1000.0f && gap_b == 1000.0f) { //lag and lead car very far
              trafficVehicleVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE

            } else { // NOT ALONE
              float b1A = 0.05f, b2A = 0.15f;
              float b1B = 0.15f, b2B = 0.40f;
              // s_0-> critical lead gap
              float g_na_D, g_bn_D;
              bool acceptLC = true;

              if (gap_a != 1000.0f) {
                g_na_D = std::max(simParameters.s_0, simParameters.s_0 + b1A * trafficVehicleVec[p].v + b2A *
                                  (trafficVehicleVec[p].v - v_a * 3.0f));

                if (gap_a < g_na_D) { //gap smaller than critical gap
                  acceptLC = false;
                }
              }

              if (acceptLC == true && gap_b != 1000.0f) {
                g_bn_D = std::max(simParameters.s_0, simParameters.s_0 + b1B * v_b * 3.0f + b2B * (v_b * 3.0f -
                                  trafficVehicleVec[p].v));

                if (gap_b < g_bn_D) { //gap smaller than critical gap
                  acceptLC = false;
                }
              }

              if (acceptLC == true) {
                trafficVehicleVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE
              }
            }

            //printf("<<LANE CHANGE\n");
          }


        }// Discretionary

        ////////////////////////////////////////////////////
        // LC 3 *MANDATORY* STATE
        if (trafficVehicleVec[p].LC_stateofLaneChanging == 1) {
          // LC 3.1 Calculate the correct lanes
          if (trafficVehicleVec[p].LC_endOKLanes == 0xFF) {
            calculateLaneCarShouldBe(currentEdge, nextEdge, intersections,
                                     trafficVehicleVec[p].edgeNextInters, trafficVehicleVec[p].edgeNumLanes,
                                     trafficVehicleVec[p].LC_initOKLanes, trafficVehicleVec[p].LC_endOKLanes);

            //printf("p%u num lanes %u min %u max %u\n",p,trafficVehicleVec[p].edgeNumLanes,trafficVehicleVec[p].LC_initOKLanes,trafficVehicleVec[p].LC_endOKLanes);
            if (trafficVehicleVec[p].LC_initOKLanes == 0 &&
                trafficVehicleVec[p].LC_endOKLanes == 0) {
              exit(0);
            }
          }


          //printf(">>LANE CHANGE\n");
          //printf("LC 0 %u\n",trafficVehicleVec[p].numOfLaneInEdge);
          bool leftLane = false, rightLane = false;

          // LC 3.2 CORRECT LANES--> DICRETIONARY LC WITHIN
          if (trafficVehicleVec[p].numOfLaneInEdge >= trafficVehicleVec[p].LC_initOKLanes &&
              trafficVehicleVec[p].numOfLaneInEdge < trafficVehicleVec[p].LC_endOKLanes) {
            // for discretionary it should be under some circustances
            if ((trafficVehicleVec[p].v < (trafficVehicleVec[p].maxSpeedMperSec * 0.7f)) &&
                (dv_dt < 0) && trafficVehicleVec[p].v > 3.0f) {
              leftLane =
                (trafficVehicleVec[p].numOfLaneInEdge > 0) && //at least one lane on the left
                (trafficVehicleVec[p].numOfLaneInEdge - 1 >= trafficVehicleVec[p].LC_initOKLanes)
                &&
                (trafficVehicleVec[p].numOfLaneInEdge - 1 < trafficVehicleVec[p].LC_endOKLanes);
              rightLane =
                (trafficVehicleVec[p].numOfLaneInEdge < trafficVehicleVec[p].edgeNumLanes - 1) &&
                //at least one lane
                (trafficVehicleVec[p].numOfLaneInEdge + 1 >= trafficVehicleVec[p].LC_initOKLanes)
                &&
                (trafficVehicleVec[p].numOfLaneInEdge + 1 < trafficVehicleVec[p].LC_endOKLanes);
              //printf("D\n");
            }
          }
          // LC 3.3 INCORRECT LANES--> MANDATORY LC
          else {
            //printf("num lanes %u min %u max %u\n",trafficVehicleVec[p].edgeNumLanes,trafficVehicleVec[p].LC_initOKLanes,trafficVehicleVec[p].LC_endOKLanes);
            //printf("p%u num lanes %u min %u max %u\n",p,trafficVehicleVec[p].edgeNumLanes,trafficVehicleVec[p].LC_initOKLanes,trafficVehicleVec[p].LC_endOKLanes);

            if (trafficVehicleVec[p].numOfLaneInEdge < trafficVehicleVec[p].LC_initOKLanes) {
              rightLane = true;
            } else {
              leftLane = true;
            }

            if (rightLane == true &&
                trafficVehicleVec[p].numOfLaneInEdge + 1 >= trafficVehicleVec[p].edgeNumLanes) {
              printf("ERROR: RT laneToCheck>=trafficVehicleVec[p].edgeNumLanes\n");
            }

            if (leftLane == true && trafficVehicleVec[p].numOfLaneInEdge == 0) {
              printf("ERROR %u: LT laneToCheck>=trafficVehicleVec[p].edgeNumLanes OK %u-%u NE %u\n",
                     p, trafficVehicleVec[p].LC_initOKLanes, trafficVehicleVec[p].LC_endOKLanes,
                     nextEdge);
              exit(0);
            }

            //printf("M L %d R %d nL %u\n",leftLane,rightLane,trafficVehicleVec[p].numOfLaneInEdge);
          }

          if (leftLane == true || rightLane == true) {

            // choose lane (if necessary)
            if (leftLane == true && rightLane == true) {
              if (qrand() % 2 == 0) {
                leftLane = false;
              } else {
                rightLane = false;
              }
            }

            ushort laneToCheck;

            if (leftLane == true) {
              laneToCheck = trafficVehicleVec[p].numOfLaneInEdge - 1;
            } else {
              laneToCheck = trafficVehicleVec[p].numOfLaneInEdge + 1;
            }

            if (laneToCheck >= trafficVehicleVec[p].edgeNumLanes) {
              printf("ERROR: laneToCheck>=trafficVehicleVec[p].edgeNumLanes %u %u\n",
                     laneToCheck, trafficVehicleVec[p].edgeNumLanes);
            }

            uchar v_a, v_b;
            float gap_a, gap_b;
            //printf("p %u LC 1 %u\n",p,laneToCheck);
            uchar trafficLightState = trafficLights[currentEdge +
                                                                trafficVehicleVec[p].numOfLaneInEdge];
            calculateGapsLC(mapToReadShift, laneMap, trafficLightState,
                            currentEdge + laneToCheck, trafficVehicleVec[p].edgeNumLanes,
                            trafficVehicleVec[p].posInLaneM,
                            trafficVehicleVec[p].length, v_a, v_b, gap_a, gap_b);

            //printf("LC 2 %u %u %f %f\n",v_a,v_b,gap_a,gap_b);
            if (gap_a == 1000.0f && gap_b == 1000.0f) { //lag and lead car very far
              trafficVehicleVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE

            } else { // NOT ALONE
              float b1A = 0.05f, b2A = 0.15f;
              float b1B = 0.15f, b2B = 0.40f;
              float gamma = 0.000025;
              // s_0-> critical lead gap
              float distEnd = trafficVehicleVec[p].length - trafficVehicleVec[p].posInLaneM;
              float expTerm = (1 - exp(-gamma * distEnd * distEnd));

              float g_na_M, g_bn_M;
              bool acceptLC = true;

              if (gap_a != 1000.0f) {
                g_na_M = std::max(simParameters.s_0, simParameters.s_0 + (b1A * trafficVehicleVec[p].v + b2A *
                                              (trafficVehicleVec[p].v - v_a * 3.0f)));

                if (gap_a < g_na_M) { //gap smaller than critical gap
                  acceptLC = false;
                }
              }

              if (acceptLC == true && gap_b != 1000.0f) {
                g_bn_M = std::max(simParameters.s_0, simParameters.s_0 + (b1B * v_b * 3.0f + b2B * (v_b * 3.0f -
                                              trafficVehicleVec[p].v)));

                if (gap_b < g_bn_M) { //gap smaller than critical gap
                  acceptLC = false;
                }
              }

              if (acceptLC == true) {
                trafficVehicleVec[p].numOfLaneInEdge = laneToCheck; // CHANGE LINE
              }
            }


          }

        }// Mandatory

      }//at least two lanes and not stopped by traffic light

    }

    ///////////////////////////////////////////////////////
    if (DEBUG_TRAFFIC == 1) {
      printf("    2. Person: %d moving in edge %u pos %f of %f\n", p, currentEdge,
             trafficVehicleVec[p].posInLaneM, trafficVehicleVec[p].length);
    }

    uchar vInMpS = (uchar)(trafficVehicleVec[p].v *
                           3); //speed in m/s to fit in uchar
    ushort posInLineCells = (ushort)(trafficVehicleVec[p].posInLaneM);
    //laneMap[mapToWriteShift + maxWidth * (currentEdge + trafficVehicleVec[p].numOfLaneInEdge) + posInLineCells] = vInMpS;
    //printf("numeoflaneinedge %d calculated edge %d\n", trafficVehicleVec[p].numOfLaneInEdge, (currentEdge + (((int) (posInLineCells / kMaxMapWidthM)) * trafficVehicleVec[p].edgeNumLanes) + trafficVehicleVec[p].numOfLaneInEdge));
    const uint posToSample = mapToWriteShift + kMaxMapWidthM * (currentEdge + (((
                               int)(posInLineCells / kMaxMapWidthM)) * trafficVehicleVec[p].edgeNumLanes) +
                             trafficVehicleVec[p].numOfLaneInEdge) + posInLineCells % kMaxMapWidthM;
    laneMap[posToSample] = vInMpS;

    //printf("2<<LANE CHANGE\n");
    if (DEBUG_TRAFFIC == 1) {
      printf("    2. Person: %d moving in edge %u pos %f of %f END\n", p, currentEdge,
             trafficVehicleVec[p].posInLaneM, trafficVehicleVec[p].length);
    }

    return;
  }

  //}
  //2.2 close to intersection

  //2.2 check if change intersection
  //!!!ALWAYS CHANGE
  //2.2.1 find next edge
  /*ushort curr_intersection=trafficVehicleVec[p].edgeNextInters;
  ushort end_intersection=trafficVehicleVec[p].end_intersection;
  //2.1 check if end*/
  if (nextEdge == -1) { //if(curr_intersection==end_intersection){
    if (DEBUG_TRAFFIC == 1) {
      printf("    2.1 Person: %d FINISHED\n", p);
    }

    trafficVehicleVec[p].active = 2; //finished
    return;
  }

  //if(trafficVehicleVec[p].nextPathEdge>=nextEdgeM.size())printf("AAAAAAAAAAAAAAAAA\n");
  /////////////
  // update edge
  /*// stop
  if(noFirstInLane==false&&trafficLights[currentEdge+trafficVehicleVec[p].numOfLaneInEdge]==0x0F){
        // first in lane and stop--> update to avoid to pass another car
        trafficLights[currentEdge+trafficVehicleVec[p].numOfLaneInEdge]=0x00;
  }*/
  //trafficVehicleVec[p].curEdgeLane=trafficVehicleVec[p].nextEdge;
  trafficVehicleVec[p].indexPathCurr++;
  trafficVehicleVec[p].maxSpeedMperSec =
    trafficVehicleVec[p].nextEdgemaxSpeedMperSec;
  trafficVehicleVec[p].edgeNumLanes = trafficVehicleVec[p].nextEdgeNumLanes;
  trafficVehicleVec[p].edgeNextInters = trafficVehicleVec[p].nextEdgeNextInters;
  trafficVehicleVec[p].length = trafficVehicleVec[p].nextEdgeLength;
  trafficVehicleVec[p].posInLaneM = numMToMove;

  if (trafficVehicleVec[p].numOfLaneInEdge >= trafficVehicleVec[p].edgeNumLanes) {
    trafficVehicleVec[p].numOfLaneInEdge = trafficVehicleVec[p].edgeNumLanes -
                                          1; //change line if there are less roads
  }

  ////////////
  // update next edge
  uint nextNEdge = indexPathVec[trafficVehicleVec[p].indexPathCurr + 1];

  //trafficVehicleVec[p].nextEdge=nextEdge;
  if (nextNEdge != -1) {
    //trafficVehicleVec[p].nextPathEdge++;
    trafficVehicleVec[p].LC_initOKLanes = 0xFF;
    trafficVehicleVec[p].LC_endOKLanes = 0xFF;

    if (DEBUG_TRAFFIC == 1) {
      printf("    2.2.1 Person: %d curr %u end %u nextEdge %u\n", p,
             trafficVehicleVec[p].edgeNextInters, trafficVehicleVec[p].end_intersection,
             nextNEdge);
    }

    //2.2.3 update person edgeData
    //trafficVehicleVec[p].nextEdge=nextEdge;
    trafficVehicleVec[p].nextEdgemaxSpeedMperSec =
      edgesData[nextNEdge].maxSpeedMperSec;
    trafficVehicleVec[p].nextEdgeNumLanes = edgesData[nextNEdge].numLines;
    trafficVehicleVec[p].nextEdgeNextInters = edgesData[nextNEdge].nextInters;
    trafficVehicleVec[p].nextEdgeLength = edgesData[nextNEdge].length;
  }

  trafficVehicleVec[p].LC_stateofLaneChanging = 0;
  uchar vInMpS = (uchar)(trafficVehicleVec[p].v *
                         3); //speed in m/s to fit in uchar
  ushort posInLineCells = (ushort)(trafficVehicleVec[p].posInLaneM);

  // laneMap[mapToWriteShift + maxWidth * (nextEdge + trafficVehicleVec[p].numOfLaneInEdge) + posInLineCells] = vInMpS;
  const uint posToSample = mapToWriteShift + kMaxMapWidthM * (nextEdge + (((int)(
                             posInLineCells / kMaxMapWidthM)) * trafficVehicleVec[p].edgeNumLanes) +
                           trafficVehicleVec[p].numOfLaneInEdge) + posInLineCells %
                           kMaxMapWidthM;  // note the last % should not happen
  laneMap[posToSample] = vInMpS;


  if (DEBUG_TRAFFIC == 1) {
    printf("    2.2.4 Person: %d New Lane %u Pos %f\n", p, nextEdge, numMToMove);
  }

}//

void simulateOneSTOPIntersectionCPU(
  uint i,
  float deltaTime,
  float currentTime, std::vector<B18IntersectionData> &intersections,
  std::vector<uchar> &trafficLights,
  std::vector<LC::B18EdgeData> &edgesData,//for the length
  std::vector<uchar> &laneMap,//to check if there are cars
  uint mapToReadShift) {
  const float deltaEvent = 0; //10.0f;!!!! CHANGE

  //if(i==0)printf("i %d\n",i);
  if (currentTime > intersections[i].nextEvent &&
      intersections[i].totalInOutEdges > 0) {
    uint edgeOT = intersections[i].edge[intersections[i].state];
    uchar numLinesO = edgeOT >> 24;
    uint edgeONum = edgeOT & kMaskLaneMap; // 0xFFFFF

    // red old traffic lights
    for (int nL = 0; nL < numLinesO; nL++) {
      trafficLights[edgeONum + nL] = 0x00; //red old traffic light
    }

    for (int iN = 0; iN <= intersections[i].totalInOutEdges + 1;
         iN++) { //to give a round
      intersections[i].state = (intersections[i].state + 1) %
                               intersections[i].totalInOutEdges;//next light

      if ((intersections[i].edge[intersections[i].state] & kMaskInEdge) ==
          kMaskInEdge) {  // 0x800000
        uint edgeIT = intersections[i].edge[intersections[i].state];
        uint edgeINum = edgeIT & kMaskLaneMap; //get edgeI 0xFFFFF
        uchar numLinesI = edgeIT >> 24;
        /// check if someone in this edge
        int rangeToCheck = 5.0f; //5m
        ushort firstPosToCheck = edgesData[edgeINum].length -
                                 intersectionClearance; //last po
        bool atLeastOneStopped = false;

        for (int posCheck = firstPosToCheck; rangeToCheck >= 0 &&
             posCheck >= 0;
             posCheck--, rangeToCheck--) { //as many cells as the rangeToCheck says
          for (int nL = 0; nL < numLinesI; nL++) {
            //int cellNum = mapToReadShift + maxWidth * (edgeINum + nL) + posCheck;
            const uint posToSample = mapToReadShift + kMaxMapWidthM * (edgeINum + (((int)(
                                       posCheck / kMaxMapWidthM)) * numLinesI) + nL) + posCheck % kMaxMapWidthM;


            if (laneMap[posToSample] == 0) { //car stopped
              trafficLights[edgeINum + nL] = 0x0F; // STOP SIGN 0x0F--> Let pass
              atLeastOneStopped = true;
            }
          }
        }

        if (atLeastOneStopped == true) {
          intersections[i].nextEvent = currentTime +
                                       deltaEvent; //just move forward time if changed (otherwise check in next iteration)
          break;
        }
      }
    }
  }
}//

void simulateOneIntersectionCPU(uint i, float currentTime,
                                std::vector<B18IntersectionData> &intersections,
                                std::vector<uchar> &trafficLights) {
  const float deltaEvent = 20.0f; /// !!!!

  //if (DEBUG_T_LIGHT == 1) printf("Inter %d CurrTime %.1f Next Event %.1f InOut %d\n", i, currentTime, intersections[i].nextEvent, intersections[i].totalInOutEdges);
  if (currentTime > intersections[i].nextEvent &&
      intersections[i].totalInOutEdges > 0) {

    uint edgeOT = intersections[i].edge[intersections[i].state];
    uchar numLinesO = edgeOT >> 24;
    uint edgeONum = edgeOT & kMaskLaneMap; // 0xFFFFF;

    if (DEBUG_T_LIGHT == 1) {
      printf("Inter %d: Event happen: numLines %u laneMap %u\n", i, numLinesO,
             edgeONum);
    }

    // red old traffic lights
    if ((edgeOT & kMaskInEdge) == kMaskInEdge) { // Just do it if we were in in
      for (int nL = 0; nL < numLinesO; nL++) {
        trafficLights[edgeONum + nL] = 0x00; //red old traffic light

        if (DEBUG_T_LIGHT == 1) {
          printf("Inter %d: Event happen: numLines %u laneMap %u --> Set red traffic light %u\n",
                 i, numLinesO, edgeONum, edgeONum + nL);
        }
      }
    }

    for (int iN = 0; iN <= intersections[i].totalInOutEdges + 1;
         iN++) { //to give a round
      intersections[i].state = (intersections[i].state + 1) %
                               intersections[i].totalInOutEdges;//next light

      if ((intersections[i].edge[intersections[i].state] & kMaskInEdge) ==
          kMaskInEdge) {  // 0x800000
        // green new traffic lights
        uint edgeIT = intersections[i].edge[intersections[i].state];
        uint edgeINum = edgeIT & kMaskLaneMap; //  0xFFFFF; //get edgeI
        uchar numLinesI = edgeIT >> 24;

        for (int nL = 0; nL < numLinesI; nL++) {
          trafficLights[edgeINum + nL] = 0xFF;

          if (DEBUG_T_LIGHT == 1) {
            printf("Inter %d: Event happen: numLines %u laneMap %u --> Set green traffic light %u\n",
                   i, numLinesO, edgeONum, edgeINum + nL);
          }
        }

        //trafficLights[edgeINum]=0xFF;
        break;
      }
    }//green new traffic light

    //printf("i %d CHANGE state %u of %d (Old edge %u New Edge %u)\n",i,intersections[i].state,intersections[i].totalInOutEdges,edgeO,edgeI);
    ////
    intersections[i].nextEvent = currentTime + deltaEvent;
  }
}//

void sampleTraffic(std::vector<B18TrafficVehicle> &trafficVehicleVec,
                   std::vector<uint> &indexPathVec,
                   std::vector<float> &accSpeedPerLinePerTimeInterval,
                   std::vector<float> &numVehPerLinePerTimeInterval, uint offset) {
  int numPeople = trafficVehicleVec.size();

  //printf("offset %d\n",offset);
  for (int p = 0; p < numPeople; p++) {
    if (trafficVehicleVec[p].active == 1) {
      int edgeNum = indexPathVec[trafficVehicleVec[p].indexPathCurr];
      accSpeedPerLinePerTimeInterval[edgeNum + offset] += trafficVehicleVec[p].v /
          3.0f;
      numVehPerLinePerTimeInterval[edgeNum + offset]++;
    }
  }//for people
}//

// numOfPasses-> define if just disjktra or iterative
// reGeneratePeopleLanes-> recompute lanes (it is used in MCMC that calls those func before)
void B18TrafficSimulator::simulateInCPU_MultiPass(int numOfPasses,
    float startTimeH, float endTimeH, bool useJohnsonRouting) {

  // create lane map
  if (DEBUG_SIMULATOR) {
    printf(">>simulateInCPU_MultiPass\n");
  }

  createLaneMap();

  if (DEBUG_SIMULATOR) {
    printf("MP Start multi pass simulation\n");
  }

  int weigthMode;
  float peoplePathSampling[] = {1.0f, 1.0f, 0.5f, 0.25f, 0.12f, 0.67f};

  for (int nP = 0; nP < numOfPasses; nP++) {
    printf("\nNumber of pass %d of %d\n", nP, numOfPasses);
    resetPeopleJobANDintersections();//reset people
    weigthMode = 1; //first time run normal weights

    if (nP == 0) {
      weigthMode = 0;
    }

    if (DEBUG_SIMULATOR) {
      printf("Num edges %d\n", boost::num_edges(simRoadGraph->myRoadGraph_BI));
    }

    QTime pathTimer;
    pathTimer.start();

    if (useJohnsonRouting) {
      printf("***Start generateRoute Johnson\n");
      B18TrafficJohnson::generateRoutes(simRoadGraph->myRoadGraph_BI,
                                        trafficVehicleVec, indexPathVec, edgeDescToLaneMapNum, weigthMode,
                                        peoplePathSampling[nP]);
    } else {
      printf("***Start generateRoutesMulti Disktra\n");
      B18TrafficDijstra::generateRoutesMulti(simRoadGraph->myRoadGraph_BI,
                                             trafficVehicleVec, indexPathVec, edgeDescToLaneMapNum, weigthMode,
                                             peoplePathSampling[nP]);
    }

    printf("***Routing computation time %d\n", pathTimer.elapsed());
    // run simulation
    printf("***Start simulateInCPU \n");
    simulateInCPU(startTimeH, endTimeH);
    printf("***End simulateInCPU \n");
    //estimate traffic density
    calculateAndDisplayTrafficDensity(nP);
    savePeopleAndRoutes(nP);
  }

  if (DEBUG_SIMULATOR) {
    printf("<<simulateInCPU_MultiPass\n");
  }
}//

void B18TrafficSimulator::simulateInCPU_Onepass(float startTimeH,
    float endTimeH, bool useJohnsonRouting) {
  resetPeopleJobANDintersections();
  // create lane map
  createLaneMap();
  // find path for each vehicle
  generateCarPaths(useJohnsonRouting);
  // run simulation
  simulateInCPU(startTimeH, endTimeH);
  //estimate traffic density
  calculateAndDisplayTrafficDensity(0);
  savePeopleAndRoutes(0);
}


void B18TrafficSimulator::simulateInCPU(float startTimeH, float endTimeH) {
  float startTime = startTimeH * 3600.0f; //7.0f
  float endTime = endTimeH * 3600.0f; //8.0f//10.0f

  if (DEBUG_SIMULATOR) {
    printf(">>simulateInCPU\n");
  }

  float currentTime = 23.99f * 3600.0f;

  for (int p = 0; p < trafficVehicleVec.size() ; p++) {
    //for(int p=40;p<41;p++){
    if (currentTime > trafficVehicleVec[p].time_departure) {
      currentTime = trafficVehicleVec[p].time_departure;
    }
  }

  //round time to be delta divisible
  int numInt = currentTime / deltaTime; //floor

  //if (DEBUG_SIMULATOR) {
  printf("currentTime %.2fs (%.2fh)--> %f\n", currentTime, currentTime / 3600.0f,
         numInt * deltaTime);
  //}

  currentTime = numInt * deltaTime;
  uint steps = 0;
  steps = (currentTime - startTime) / deltaTime;

  // start as early as starting time
  if (currentTime < startTime) { //as early as the starting time
    currentTime = startTime;
  }

  // 0. put data as references to simplified code

  int numPeople = trafficVehicleVec.size();
  int numIntersec = intersections.size();

  //if (DEBUG_SIMULATOR) {
  printf(">>Start Simulation %.2fs (%.2fh)\n", currentTime,
         currentTime / 3600.0f);
  //}

  QTime timer;
  timer.start();
  bool readFirstMap = true;
  uint mapToReadShift;
  uint mapToWriteShift;
  uint halfLaneMap = laneMap.size() / 2; //it is multiple of 2

  // sampling
  uint numStepsPerSample = 30.0f / deltaTime; //each min
  const uint numStepsTogether = 12; //change also in density (10 per hour)
  uint numSamples = ceil(((endTime - startTime) / (deltaTime * numStepsPerSample *
                          numStepsTogether))) + 1; //!!!

  if (DEBUG_SIMULATOR) {
    printf("numSamples %d\n", numSamples);
  }

  uint numLines = trafficLights.size();
  accSpeedPerLinePerTimeInterval.resize(numSamples * numLines); //!!
  numVehPerLinePerTimeInterval.resize(numSamples * numLines);
  memset(accSpeedPerLinePerTimeInterval.data(), 0,
         accSpeedPerLinePerTimeInterval.size()*sizeof(float));
  memset(numVehPerLinePerTimeInterval.data(), 0,
         numVehPerLinePerTimeInterval.size()*sizeof(float));
  int samplingNumber = 0;

  if (DEBUG_SIMULATOR) {
    printf("Map -->hlafMap /2 %u delta %f\n", halfLaneMap, deltaTime);
  }

  printf("\nStart Simulation\n");

  int count = 0;

  while (currentTime < endTime) {
    if (count++ % 360 == 0) {
      printf("Time %.2fh (%.2f --> %.2f): %.0f%%\n", (currentTime / 3600.0f),
             (startTime / 3600.0f), (endTime / 3600.0f),
             100.0f - (100.0f * (endTime - currentTime) / (endTime - startTime)));
    }

    //bool anyActive=false;
    ////////////////////////////////////////////////////////////
    // 1. CHANGE MAP: set map to use and clean the other
    if (DEBUG_SIMULATOR) {
      printf("Clean Map\n");
    }

    if (readFirstMap == true) {
      mapToReadShift = 0;
      mapToWriteShift = halfLaneMap;
      memset(&laneMap.data()[halfLaneMap], -1,
             halfLaneMap * sizeof(unsigned char)); //clean second half
    } else {
      mapToReadShift = halfLaneMap;
      mapToWriteShift = 0;
      memset(&laneMap.data()[0], -1,
             halfLaneMap * sizeof(unsigned char)); //clean first half
    }

    readFirstMap = !readFirstMap; //next iteration invert use


    ////////////////////////////////////////////////////////////
    // 2. UPDATE INTERSECTIONS
    if (DEBUG_SIMULATOR) {
      printf("Update Intersections\n");
    }

    for (int i = 0; i < intersections.size(); i++) {
      simulateOneIntersectionCPU(i, currentTime, intersections, trafficLights);
    }

    //printf("Sim people\n");
    ////////////////////////////////////////////////////////////
    // 3. UPDATE PEOPLE
    if (DEBUG_SIMULATOR) {
      printf("Update People\n");
    }

    for (int p = 0; p < numPeople; p++) {
      simulateOnePersonCPU(p, deltaTime, currentTime, mapToReadShift, mapToWriteShift,
                           trafficVehicleVec, indexPathVec, edgesData, laneMap, intersections,
                           trafficLights, simParameters);
    }//for people

    ////////////////////////////////////////////////////////////
    // 4. UPDATE SAMPLIG
    //if(steps%numStepsPerSample==0){
    if (DEBUG_SIMULATOR) {
      printf("Update Sampling\n");
    }

    if ((((float)((int)currentTime)) == (currentTime)) &&
        ((int)currentTime % ((int)30)) == 0) { //3min //(sample double each 3min)
      samplingNumber = (currentTime - startTime) / (30 * numStepsTogether);
      uint offset = numLines * samplingNumber;
      sampleTraffic(trafficVehicleVec, indexPathVec, accSpeedPerLinePerTimeInterval,
                    numVehPerLinePerTimeInterval, offset);
      //if((((float)((int)currentTime))==(currentTime))&&((int)currentTime%((int)30*60))==0)//each hour
      int cTH = currentTime / 3600;
      int cTM = (currentTime - cTH * 3600) / 60;

      //int cTS=(currentTime-cTH*3600-cTM*60);
      if (cTM % 20 == 0) {
        printf("T[%d] Time %f (%d:%02d) samplingNumber %d\n", threadNumber, currentTime,
               cTH, cTM, samplingNumber);
      }
    }

    // UPDATE POLLUTION

    /*if (calculatePollution == true &&
        (((float)((int)currentTime)) == (currentTime)) &&
        ((int)currentTime % ((int)60 * 6)) == 0) { //each 6 min
      if (DEBUG_SIMULATOR) {
        printf("Update Pollution\n");
      }
      gridPollution.addValueToGrid(currentTime, trafficVehicleVec, indexPathVec, simRoadGraph, clientMain, laneMapNumToEdgeDesc);
    }*/

    currentTime += deltaTime;
    steps++;

    // RENDER IF NECESSARY

#ifdef B18_RUN_WITH_GUI

    if (clientMain != nullptr &&
        clientMain->ui.b18RenderSimulationCheckBox->isChecked()) {
      if (DEBUG_SIMULATOR) {
        printf("Render\n");
      }

      if (steps % clientMain->ui.b18RenderStepSpinBox->value() ==
          0) { //each "steps" steps, render
        QString timeT;
        int timeH = currentTime / 3600.0f;
        int timeM = (currentTime - timeH * 3600.0f) / 60.0f;
        timeT.sprintf("%d:%02d", timeH, timeM);
        //clientMain->ui.timeLCD->display(timeT);
        //printf("About to render\n");
        clientMain->glWidget3D->updateGL();
        QApplication::processEvents();
      }
    }

#endif


    if (DEBUG_SIMULATOR) {
      printf("Finish one step\n");
    }
  }

  if (DEBUG_SIMULATOR) {
    printf("<<End Simulation (count %d) TIME: %d ms.\n", count, timer.elapsed());
  }

  {
    // Total number of car steps
    uint totalNumSteps = 0;
    float totalCO = 0.0f;

    for (int p = 0; p < numPeople; p++) {
      totalNumSteps += trafficVehicleVec[p].num_steps;
      totalCO += trafficVehicleVec[p].co;
    }

    avgTravelTime = (totalNumSteps) / (trafficVehicleVec.size() * 60.0f); //in min
    printf("(Count %d) Total num steps %u Avg %f min Avg CO %f. Calculated in %d ms\n",
           count, totalNumSteps, avgTravelTime, totalCO / trafficVehicleVec.size(),
           timer.elapsed());
  }

  //clientMain->glWidget3D->updateGL();

  if (DEBUG_SIMULATOR) {
    printf("<<simulate\n");
  }
}//


#ifdef B18_RUN_WITH_GUI
void B18TrafficSimulator::render(VBORenderManager &rendManager) {

  ///////////////////////////////
  // RENDER POLUTION
  rendManager.removeStaticGeometry("Sim_Points", false);
  rendManager.removeStaticGeometry("Sim_Box", false);
  /////////////////////////////
  // RENDER CARS
  const bool renderMultiEdge = false;
  const bool renderBoxes = false;
  const bool renderModels = false;

  // init render people
  if (b18TrafficSimulatorRender.size() != trafficVehicleVec.size() &&
      trafficVehicleVec.size() > 0) {
    printf("SIm init render people\n");
    b18TrafficSimulatorRender.clear();
    b18TrafficSimulatorRender.resize(trafficVehicleVec.size());
  }

  if (b18TrafficLightRender.size() != trafficLights.size() &&
      trafficLights.size() > 0) {
    printf("SIm init traffic sim\n");
    b18TrafficLightRender.clear();
    b18TrafficLightRender.resize(trafficLights.size());
  }

  //////////////////////////
  // RENDER CAR AS MODELS
  /*if(renderModels==true&&trafficVehicleVec.size()>0){
        int numPeople=trafficVehicleVec.size();
        const float heightPoint=2.0f;
        //glDisable(GL_DEPTH_TEST);
        //glBegin(GL_QUADS);

        // SAVE TO FILE
        QTextStream out;
        QFile* file;
        int numPeopleActive=0;
        if(clientMain->ui.animationSaveCheckBox->isChecked()){
                QString fileName="data/animation.txt";
                file=new QFile(fileName);
                if(!file->open(QIODevice::WriteOnly|QIODevice::Append)) {
                        printf("ERROR: Not possible to read %s\n",fileName.toLatin1().constData());
                        exit(0);
                }
                out.setDevice(file);
                for(int p=0;p<numPeople;p++){
                        if(trafficVehicleVec[p].active!=1){
                                continue;
                        }
                        numPeopleActive++;
                }
                out<<"NumCars:"<<numPeopleActive<<"\n";
        }


        for(int p=0;p<numPeople;p++){

                //0. check if finished
                if(trafficVehicleVec[p].active!=1){
                        continue;
                }
                glColor3ub((trafficVehicleVec[p].color>>24)&0xFF,(trafficVehicleVec[p].color>>16)&0xFF,(trafficVehicleVec[p].color>>8)&0xFF);
                //position
                if(laneMapNumToEdgeDesc.find(indexPathVec[trafficVehicleVec[p].indexPathCurr])==laneMapNumToEdgeDesc.end()){
                        printf("ERROR\n");//edge not found in map
                        continue;
                }
                QVector3D p0,p1;
                float posInLaneM= trafficVehicleVec[p].posInLaneM;
                float lenghtLane= trafficVehicleVec[p].length;//meters?
                RoadGraph::roadGraphEdgeDesc_BI ei=laneMapNumToEdgeDesc[indexPathVec[trafficVehicleVec[p].indexPathCurr]];
                // MULTI EDGE
                if(renderMultiEdge==true&& simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry.size()>0){

                        for(int gN=0;gN<simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry.size()-1;gN++){//note -1
                                float currLe=(simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN]-simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN+1]).length();
                                if(posInLaneM>currLe){//next edge
                                        posInLaneM-=currLe;
                                        continue;
                                }
                                 p0 = simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN];
                                 p1 = simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN+1];
                                break;
                        }
                }else{
                        // ONE EDGE
                         p0 = simRoadGraph->myRoadGraph_BI[boost::source(ei, simRoadGraph->myRoadGraph_BI)].pt;
                         p1 = simRoadGraph->myRoadGraph_BI[boost::target(ei, simRoadGraph->myRoadGraph_BI)].pt;
                }
                QVector3D dir=(p1-p0).normalized();
                QVector3D per=(QVector3D::crossProduct(QVector3D(0,0,1.0f),dir).normalized());
                bool goodPoint=true;
                float range=-1.0f;
                if(
                        //posInLaneM<intersectionClearance+range||
                        posInLaneM<intersectionClearance+range*2.0f||
                        posInLaneM>(lenghtLane-intersectionClearance-range)){
                                goodPoint=false;
                }
                //printf("trafficVehicleVec[p].numOfLaneInEdge %u\n",trafficVehicleVec[p].numOfLaneInEdge);
                float perShift=-0.5f*LC::misctools::Global::global()->roadLaneWidth*(1+2*trafficVehicleVec[p].numOfLaneInEdge);
                QVector3D v=p0+dir*posInLaneM+perShift*per;// center of the back of the car
                ///////////////////////////////
                // INT
                //printf("Pos %f %f %f ",v.x(),v.y(),v.z());
                if(B18TrafficSimulatorRender[p].getInterpolated(goodPoint,v,dir,v,dir)==false){//copy and ref
                        // not render, just write a negative z
                        if(clientMain->ui.animationSaveCheckBox->isChecked()){//save to file
                                out<<"Car:"<<QString::number(p)<<":pos:"<<10000.0f<<":"<<10000.0f<<":-100.0f:dir:"<<dir.x()<<":"<<dir.y()<<":"<<dir.z()<<":\n";
                        }
                        continue;//no ready to render
                }
                //printf("Inter %f %f %f\n",v.x(),v.y(),v.z());
                ///////////////////////////////
                ModelSpec specR;
                specR.type=0;//car
                specR.transMatrix.setToIdentity();
                //specR.transMatrix.rotate(180.0f,0,0,1.0f);

                float angle;
                //angle=acos(QVector3D::dotProduct(QVector3D(1.0,0,0),dir))*57.2957795f+90.0f;
                {
                        QVector3D referenceForward =QVector3D(1.0,0,0);
                        QVector3D referenceRight= QVector3D::crossProduct(QVector3D(0,0,1.0f), referenceForward);
                        QVector3D newDirection = dir;
                        //float angle = Vector3.Angle(newDirection, referenceForward);
                        {
                                float s = QVector3D::crossProduct(referenceForward,newDirection).length();
                                float c = QVector3D::dotProduct(referenceForward,newDirection);
                                angle = atan2(s, c);
                        }
                        float sign = (QVector3D::dotProduct(newDirection, referenceRight) > 0.0f) ? 1.0f: -1.0f;
                        angle = sign * angle;
                        angle*=57.2957795f;//to defrees
                        angle+=90.0f;//model is original turned 90degrees
                }
                specR.transMatrix.translate(v.x(),v.y(),v.z());//-1.5f is the street level
                specR.transMatrix.rotate(angle,0,0,1.0f);

                clientMain->mGLWidget_3D->modelsVBO.renderModel(specR);
                if(clientMain->ui.animationSaveCheckBox->isChecked()){//save to file
                        out<<"Car:"<<QString::number(p)<<":pos:"<<v.x()<<":"<<v.y()<<":"<<v.z()<<":dir:"<<dir.x()<<":"<<dir.y()<<":"<<dir.z()<<":\n";
                }

        }//for people

        if(clientMain->ui.animationSaveCheckBox->isChecked()){
                out.flush();
                file->close();
                //file.close();
        }
        //glEnd();
        //glEnable(GL_DEPTH_TEST);
  }//model*/

  //////////////////////////
  // RENDER CAR AS BOXES
  if (renderBoxes == true && trafficVehicleVec.size() > 0) {
    std::vector<Vertex> carQuads;
    QVector3D carColor;
    int numPeople = trafficVehicleVec.size();
    //carQuads.resize(4*numPeople);//quad per car many not active

    const float heightPoint = 5.0f;

    for (int p = 0; p < numPeople; p++) {

      //0. check if finished
      if (trafficVehicleVec[p].active != 1) {
        continue;
      }

      carColor = QVector3D(1, 0, 0);

      //position
      if (laneMapNumToEdgeDesc.find(
            indexPathVec[trafficVehicleVec[p].indexPathCurr]) ==
          laneMapNumToEdgeDesc.end()) {
        printf("ERROR\n");//edge not found in map
        continue;
      }

      QVector3D p0, p1;
      float posInLaneM = trafficVehicleVec[p].posInLaneM;
      RoadGraph::roadGraphEdgeDesc_BI ei =
        laneMapNumToEdgeDesc[indexPathVec[trafficVehicleVec[p].indexPathCurr]];

      // MULTI EDGE
      if (renderMultiEdge == true &&
          simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry.size() > 0) {

        for (int gN = 0;
             gN < simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry.size() - 1;
             gN++) { //note -1
          float currLe = (simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN] -
                          simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN + 1]).length();

          if (posInLaneM > currLe) { //next edge
            posInLaneM -= currLe;
            continue;
          }

          p0 = simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN];
          p1 = simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN + 1];
          break;
        }
      } else {
        // ONE EDGE
        p0 = simRoadGraph->myRoadGraph_BI[boost::source(ei,
                                          simRoadGraph->myRoadGraph_BI)].pt;
        p1 = simRoadGraph->myRoadGraph_BI[boost::target(ei,
                                          simRoadGraph->myRoadGraph_BI)].pt;
      }

      QVector3D dir = (p1 - p0).normalized();
      QVector3D per = (QVector3D::crossProduct(QVector3D(0, 0, 1.0f),
                       dir).normalized());
      //printf("trafficVehicleVec[p].numOfLaneInEdge %u\n",trafficVehicleVec[p].numOfLaneInEdge);
      float perShift = -0.5f * G::global().getFloat("roadLaneWidth") *
                       (1 + 2 * trafficVehicleVec[p].numOfLaneInEdge);
      QVector3D v = p0 + dir * posInLaneM + perShift *
                    per; // center of the back of the car
      float widthHalf = 1.82f / 2.0f;
      float length = 4.12f;
      QVector3D v0 = v - widthHalf * per;
      QVector3D v1 = v - widthHalf * per + length * dir;
      QVector3D v2 = v + widthHalf * per + length * dir;
      QVector3D v3 = v + widthHalf * per;



      carQuads.push_back(Vertex(QVector3D(v0.x(), v0.y(), heightPoint), carColor,
                                QVector3D(), QVector3D()));
      carQuads.push_back(Vertex(QVector3D(v1.x(), v1.y(), heightPoint), carColor,
                                QVector3D(), QVector3D()));
      carQuads.push_back(Vertex(QVector3D(v2.x(), v2.y(), heightPoint), carColor,
                                QVector3D(), QVector3D()));
      carQuads.push_back(Vertex(QVector3D(v3.x(), v3.y(), heightPoint), carColor,
                                QVector3D(), QVector3D()));

    }//for people

    //printf("carQuads %d\n",carQuads.size());

    if (carQuads.size() > 0) {
      rendManager.addStaticGeometry("Sim_Box", carQuads, "", GL_QUADS,
                                    1 | mode_AdaptTerrain);
      glDisable(GL_DEPTH_TEST);
      rendManager.renderStaticGeometry("Sim_Box");
      glEnable(GL_DEPTH_TEST);
    }
  }//boxes


  //////////////////////////
  // RENDER CAR AS POINTS
  if (renderBoxes == false && renderModels == false &&
      trafficVehicleVec.size() > 0) {
    // printf("SIm render as points");
    //rendManager.removeStaticGeometry("Sim_Points",false);
    const float heightPoint = 5.0f;
    std::vector<Vertex> carPoints;
    QVector3D pointColor;
    int numPeople = trafficVehicleVec.size();
    //carPoints.resize(numPeople);//many not active
    int activeCars = 0;

    for (int p = 0; p < numPeople; p++) {
      //printf(" 0. Person: %d\n",p);
      //0. check if finished
      if (trafficVehicleVec[p].active == 1) {
        //printf("%x\n",trafficVehicleVec[p].color);
        pointColor = QVector3D((trafficVehicleVec[p].color >> 24) & 0xFF,
                               (trafficVehicleVec[p].color >> 16) & 0xFF,
                               (trafficVehicleVec[p].color >> 8) & 0xFF) / 255.0f;

        //position
        if (laneMapNumToEdgeDesc.find(indexPathVec[trafficVehicleVec[p].indexPathCurr])
            == laneMapNumToEdgeDesc.end()) {
          printf("ERROR\n");//edge not found in map
          continue;
        }

        RoadGraph::roadGraphEdgeDesc_BI ei =
          laneMapNumToEdgeDesc[indexPathVec[trafficVehicleVec[p].indexPathCurr]];

        // MULTI EDGE
        if (renderMultiEdge == true &&
            simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry.size() > 0) {
          float posInLaneM = trafficVehicleVec[p].posInLaneM;

          for (int gN = 0;
               gN < simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry.size() - 1;
               gN++) { //note -1
            float currLe = (simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN] -
                            simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN + 1]).length();

            if (posInLaneM > currLe) { //next edge
              posInLaneM -= currLe;
              continue;
            }

            QVector3D p0 = simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN];
            QVector3D p1 = simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN + 1];
            QVector3D dir = (p1 - p0).normalized();
            QVector3D per = (QVector3D::crossProduct(QVector3D(0, 0, 1.0f),
                             dir).normalized());
            //printf("trafficVehicleVec[p].numOfLaneInEdge %u\n",trafficVehicleVec[p].numOfLaneInEdge);
            float perShift = -0.5f * G::global().getFloat("roadLaneWidth") *
                             (1 + 2 * trafficVehicleVec[p].numOfLaneInEdge);
            QVector3D v = p0 + dir * posInLaneM + perShift * per;
            //glVertex3f(v.x(),v.y(),heightPoint);
            carPoints[p] = Vertex(QVector3D(v.x(), v.y(), heightPoint), pointColor,
                                  QVector3D(), QVector3D());
            activeCars++;
            break;
          }
        } else {
          // ONE EDGE
          //printf("One edge\n");
          QVector3D p0 = simRoadGraph->myRoadGraph_BI[boost::source(ei,
                         simRoadGraph->myRoadGraph_BI)].pt;
          QVector3D p1 = simRoadGraph->myRoadGraph_BI[boost::target(ei,
                         simRoadGraph->myRoadGraph_BI)].pt;
          QVector3D dir = (p1 - p0).normalized();
          QVector3D per = (QVector3D::crossProduct(QVector3D(0, 0, 1.0f),
                           dir).normalized());

          float perShift = -0.5f * G::global().getFloat("roadLaneWidth") *
                           (1 + 2 * trafficVehicleVec[p].numOfLaneInEdge);
          //QVector3D v = p0 + dir * trafficVehicleVec[p].posInLaneM + perShift * per;

          float roadLength = (p1 - p0).length();
          float dirShift = (trafficVehicleVec[p].posInLaneM / trafficVehicleVec[p].length) *
                           roadLength;
          QVector3D v = p0 + dir * dirShift + perShift * per;
          //printf("Edge %u GUI length %.2f Data %.2f PosInLaneM %.2f DirShift %.2f--> Perc %.2f\n", trafficVehicleVec[p].currPathEdge, roadLength, trafficVehicleVec[p].length, trafficVehicleVec[p].posInLaneM, dirShift, (trafficVehicleVec[p].posInLaneM / trafficVehicleVec[p].length));

          carPoints.push_back(Vertex(QVector3D(v.x(), v.y(), heightPoint), pointColor,
                                     QVector3D(), QVector3D()));
          activeCars++;
        }
      }
    }

    //glEnd();
    if (carPoints.size() > 0) {
      rendManager.addStaticGeometry("Sim_Points", carPoints, "", GL_POINTS,
                                    1 | mode_AdaptTerrain);
      glDisable(GL_DEPTH_TEST);
      glPointSize(15.0f);
      rendManager.renderStaticGeometry("Sim_Points");
      glEnable(GL_DEPTH_TEST);
    }
  }

  /////////////////////////////
  // RENDER INTERSECTIONS
  RoadGraph::roadGraphEdgeIter_BI ei, eiEnd;
  QVector3D p0;
  QVector3D p1;

  //Render edges ===============
  if (false && edgeDescToLaneMapNum.size() > 0) {
    int numEdges = 0;

    for (boost::tie(ei, eiEnd) = boost::edges(simRoadGraph->myRoadGraph_BI);
         ei != eiEnd; ++ei) {


      p0 = simRoadGraph->myRoadGraph_BI[boost::source(*ei,
                                        simRoadGraph->myRoadGraph_BI)].pt;
      p1 = simRoadGraph->myRoadGraph_BI[boost::target(*ei,
                                        simRoadGraph->myRoadGraph_BI)].pt;
      /// N
      QVector3D dir = (p1 - p0).normalized();
      QVector3D per = (QVector3D::crossProduct(QVector3D(0, 0, 1.0f),
                       dir).normalized());

      ///
      int numL = simRoadGraph->myRoadGraph_BI[*ei].numberOfLanes;
      uint laneMapNum = edgeDescToLaneMapNum[*ei];

      for (int lN = 0; lN < numL; lN++) {
        uchar trafficLight = 0x01; //default

        //uint lN=0;
        if (trafficLights.size() > 0) {
          trafficLight = trafficLights[laneMapNum + lN];
        }

        b18TrafficLightRender[laneMapNum + lN].getInterpolated(trafficLight,
            trafficLight);

        switch (trafficLight) {
        case 0x00:
          glColor3ub(255, 0, 0);
          break;

        case 0xFF:
          glColor3ub(0, 255, 0);
          break;

        default:
          glColor3ub(0, 0, 0);
          //printf("Unknown traffic light state\n");
        }

        float perShift = -0.5f * G::global().getFloat("roadLaneWidth") * (1 + 2 * lN);
        QVector3D p1S = QVector3D(p1.x() + per.x() * perShift,
                                  p1.y() + per.y() * perShift, p1.z());
        p1S -= intersectionClearance * dir; //move to be brefore clearance
        /// CONE
        //if(LC::misctools::Global::global()->view_arterials_arrows_render==true){
        LC::misctools::drawCone(dir, p1S - dir * 0.5f, 2.0f, 1.0f, 16);
        //}
      }

      //
    }
  }
}//
#endif

void writeIndexPathInitFile(
  int numOfPass,
  int start_time, int end_time,
  const std::vector<B18TrafficVehicle> &trafficVehicleVec){
  QFile indexPathInitFile(QString::number(numOfPass) + "_indexPathInit" + QString::number(start_time) + "to" + QString::number(end_time) + ".csv");
  if (indexPathInitFile.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
    std::cout << "> Saving indexPathInit file... (size " << trafficVehicleVec.size() << ")" << std::endl;
    QTextStream streamP(&indexPathInitFile);
    streamP << "p,indexPathInit\n";
    for (int p = 0; p < trafficVehicleVec.size(); p++) {
      streamP << p;
      streamP << "," << trafficVehicleVec[p].indexPathInit;
      streamP << "\n";
    }
    indexPathInitFile.close();
    std::cout << "> Finished saving indexPathInit file." << std::endl;
  }
}

void writePeopleFile(
  int numOfPass,
  const std::shared_ptr<abm::Graph> & graph_,
  int start_time, int end_time,
  const std::vector<B18TrafficVehicle> &trafficVehicleVec,
  float deltaTime){
  QFile peopleFile(QString::number(numOfPass) + "_people" + QString::number(start_time) + "to" + QString::number(end_time) + ".csv");
  if (peopleFile.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
    std::cout << "> Saving People file... (size " << trafficVehicleVec.size() << ")" << std::endl;
    QTextStream streamP(&peopleFile);
    streamP << "p,init_intersection,end_intersection,time_departure,num_steps,co,gas,distance,a,b,T,avg_v(mph),active,last_time_simulated,path_length_cpu,path_length_gpu\n";

    for (int p = 0; p < trafficVehicleVec.size(); p++) {
      streamP << trafficVehicleVec[p].id;
      streamP << "," << graph_->nodeIndex_to_osmid_[trafficVehicleVec[p].init_intersection];
      streamP << "," << graph_->nodeIndex_to_osmid_[trafficVehicleVec[p].end_intersection];
      streamP << "," << trafficVehicleVec[p].time_departure;
      streamP << "," << trafficVehicleVec[p].num_steps * deltaTime;
      streamP << "," << trafficVehicleVec[p].co;
      streamP << "," << trafficVehicleVec[p].gas;
      streamP << "," << trafficVehicleVec[p].dist_traveled;
      streamP << "," << trafficVehicleVec[p].a;
      streamP << "," << trafficVehicleVec[p].b;
      streamP << "," << trafficVehicleVec[p].T;
      streamP << "," << (trafficVehicleVec[p].cum_v / trafficVehicleVec[p].num_steps) * 3600 / 1609.34;
      streamP << "," << trafficVehicleVec[p].active;
      streamP << "," << trafficVehicleVec[p].last_time_simulated;
      streamP << "," << trafficVehicleVec[p].path_length_cpu;
      streamP << "," << trafficVehicleVec[p].path_length_gpu;
      streamP << "\n";
    }

    peopleFile.close();
    std::cout << "> Finished saving People file." << std::endl;
  }
}

bool isLastEdgeOfPath(abm::graph::edge_id_t edgeInPath){
  return edgeInPath == -1;
}

void writeRouteFile(int numOfPass,
  const std::vector<personPath> allPathsInVertexes,
  int start_time, int end_time,
  const std::vector<B18TrafficVehicle> &trafficVehicleVec,
  const std::shared_ptr<abm::Graph> & graph_,
  const std::vector<uint>& allPathsInEdgesCUDAFormat,
  const std::vector<uint>& edgeIdToLaneMapNum,
  const std::vector<LC::B18EdgeData>& edgesData) { 
  QFile routeFile(QString::number(numOfPass) + "_route" + QString::number(start_time) + "to" + QString::number(end_time) + ".csv");
  if (routeFile.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
    std::cout << "> Saving Route file..." << std::endl;
    QTextStream streamR(&routeFile);
    // streamR << "p:route:distance\n";

    for (const personPath& aPersonPath: allPathsInVertexes){
      // streamR << aPersonPath.person_id << ":[";
      float distance = 0;
      for (int j = 0; j < aPersonPath.pathInVertexes.size()-1; j++){
        auto vertexFrom = aPersonPath.pathInVertexes[j];
        auto vertexTo = aPersonPath.pathInVertexes[j+1];
        auto oneEdgeInCPUFormat = graph_->edge_ids_[vertexFrom][vertexTo];
        auto oneEdgeInGPUFormat =  edgeIdToLaneMapNum[oneEdgeInCPUFormat];
        streamR << oneEdgeInCPUFormat;
        if (j < aPersonPath.pathInVertexes.size() - 2){
          streamR << ",";
        }

        // Check that indexPathInit matches the first edge for that person
        assert(oneEdgeInCPUFormat < edgeIdToLaneMapNum.size());
        if (allPathsInEdgesCUDAFormat[trafficVehicleVec[aPersonPath.person_id].indexPathInit + j] != edgeIdToLaneMapNum[oneEdgeInCPUFormat]){
          std::cout << "For person " << aPersonPath.person_id
                    << ", indexPathInit is " << trafficVehicleVec[aPersonPath.person_id].indexPathInit
                    << ", which means the first edge in CUDA format is " << allPathsInEdgesCUDAFormat[trafficVehicleVec[aPersonPath.person_id].indexPathInit + j]
                    << ". However, edgeIdToLaneMapNum[oneEdgeInCPUFormat] is " << oneEdgeInGPUFormat
                    << std::endl;
          throw runtime_error("Initial edges do not match.");
        }

        distance += edgesData[oneEdgeInGPUFormat].length;

      }
      // streamR << "]:" << distance << "\n";
      streamR <<"\n";
    }
    routeFile.close();
  }
  std::cout << "> Finished saving Route file." << std::endl;
}

void writeAllPathsInEdgesCUDAFormatFile(int numOfPass,
  int start_time, int end_time,
  const std::vector<uint>& allPathsInEdgesCUDAFormat) {
  QFile allPathsInEdgesCUDAFormatFile(QString::number(numOfPass) + "_allPathsInEdgesCUDAFormat" + QString::number(start_time) + "to" + QString::number(end_time) + ".csv");
  if (allPathsInEdgesCUDAFormatFile.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
    std::cout << "> Saving allPathsInEdgesCUDAFormat (size " << allPathsInEdgesCUDAFormat.size() << ")..." << std::endl;
    QTextStream allPathsInEdgesCUDAFormatStream(&allPathsInEdgesCUDAFormatFile);
    allPathsInEdgesCUDAFormatStream << "i,allPathsInEdgesCUDAFormat\n";

    for (int i = 0; i < allPathsInEdgesCUDAFormat.size(); ++i){
      allPathsInEdgesCUDAFormatStream << i << "," << allPathsInEdgesCUDAFormat[i] << "\n";
    }

    allPathsInEdgesCUDAFormatFile.close();
  }
  std::cout << "> Finished saving AllPathsInEdgesCUDAFormat." << std::endl;
}

void B18TrafficSimulator::savePeopleAndRoutesSP(
  const std::vector<personPath>& allPathsInVertexes,
  const std::vector<uint>& allPathsInEdgesCUDAFormat,
  const std::vector<uint>& edgeIdToLaneMapNum,
  const int numOfPass,
  const std::shared_ptr<abm::Graph>& graph_,
  const int start_time, const int end_time,
  const std::vector<LC::B18EdgeData>& edgesData) {

  std::cout << "Saving output files..." << std::endl; 
  std::thread threadWritePeopleFile(writePeopleFile, numOfPass, graph_, start_time, end_time, trafficVehicleVec, deltaTime);
  std::thread threadWriteRouteFile(writeRouteFile, numOfPass, allPathsInVertexes, start_time, end_time, trafficVehicleVec, graph_, allPathsInEdgesCUDAFormat, edgeIdToLaneMapNum, edgesData);
  std::thread threadWriteIndexPathInitFile(writeIndexPathInitFile, numOfPass, start_time, end_time, trafficVehicleVec);
  std::thread threadWriteAllPathsInEdgesCUDAFormatFile(writeAllPathsInEdgesCUDAFormatFile, numOfPass, start_time, end_time, allPathsInEdgesCUDAFormat);

  threadWritePeopleFile.join();
  threadWriteRouteFile.join();
  threadWriteAllPathsInEdgesCUDAFormatFile.join();
  threadWriteIndexPathInitFile.join();
  std::cout << "Finished saving output files." << std::endl;
}

void B18TrafficSimulator::savePeopleAndRoutes(int numOfPass) {
  const bool saveToFile = true;

  if (saveToFile) {
    /////////////////////////////////
    // SAVE TO FILE
    QFile peopleFile(QString::number(numOfPass) + "_people.csv");
    QFile routeFile(QString::number(numOfPass) + "_route.csv");
    QFile routeCount(QString::number(numOfPass) + "_edge_route_count.csv");

    if (peopleFile.open(QIODevice::ReadWrite) &&
        routeFile.open(QIODevice::ReadWrite) && routeCount.open(QIODevice::ReadWrite)) {

      /////////////
      // People Route
      printf("Save route %d\n", trafficVehicleVec.size());
      QHash<uint, uint> laneMapNumCount;
      QTextStream streamR(&routeFile);
      std::vector<float> personDistance(trafficVehicleVec.size(), 0.0f);
      streamR << "p,route\n";

      for (int p = 0; p < trafficVehicleVec.size(); p++) {
        streamR << p;
        // Save route
        uint index = 0;

        while (indexPathVec[trafficVehicleVec[p].indexPathInit + index] != -1) {
          uint laneMapNum = indexPathVec[trafficVehicleVec[p].indexPathInit + index];

          if (laneMapNumToEdgeDesc.count(laneMapNum) > 0) { // laneMapNum in map
            streamR << "," <<
                    simRoadGraph->myRoadGraph_BI[laneMapNumToEdgeDesc[laneMapNum]].faci; // get id of the edge from the roadgraph
            laneMapNumCount.insert(laneMapNum, laneMapNumCount.value(laneMapNum,
                                   0) + 1);//is it initialized?
            personDistance[p] +=
              simRoadGraph->myRoadGraph_BI[laneMapNumToEdgeDesc[laneMapNum]].edgeLength;
            index++;
          } else {
            //printf("Save route: This should not happen\n");
            break;
          }

        }

        streamR << "\n";
      } // people

      routeFile.close();

      ///////////////
      // People
      printf("Save people %d\n", trafficVehicleVec.size());
      QTextStream streamP(&peopleFile);
      streamP <<
              "p,init_intersection,end_intersection,time_departure,num_steps,co,gas,distance,a,b,T\n";

      for (int p = 0; p < trafficVehicleVec.size(); p++) {
        streamP << p;
        streamP << "," << trafficVehicleVec[p].init_intersection;
        streamP << "," << trafficVehicleVec[p].end_intersection;
        streamP << "," << trafficVehicleVec[p].time_departure;
        streamP << "," << trafficVehicleVec[p].num_steps;
        streamP << "," << trafficVehicleVec[p].co;
        streamP << "," << trafficVehicleVec[p].gas;
        streamP << "," << personDistance[p];

        streamP << "," << trafficVehicleVec[p].a;
        streamP << "," << trafficVehicleVec[p].b;
        streamP << "," << trafficVehicleVec[p].T;
        streamP << "\n";
      } // people

      peopleFile.close();

      ////////////
      // Per edge route count
      printf("Save edge route count %d\n", laneMapNumCount.size());
      QTextStream streamC(&routeCount);
      QHash<uint, uint>::iterator i;

      for (i = laneMapNumCount.begin(); i != laneMapNumCount.end(); ++i) {
        uint laneMapNum = i.key();
        streamC <<
                simRoadGraph->myRoadGraph_BI[laneMapNumToEdgeDesc[laneMapNum]].faci; // get id of the edge from the roadgraph
        streamC << "," << i.value();
        streamC << "\n";
      }

      streamC << "\n";
      routeCount.close();
    }
  }

  printf("\n<<calculateAndDisplayTrafficDensity\n");
}//

void B18TrafficSimulator::calculateAndDisplayTrafficDensity(int numOfPass) {
  int tNumLanes = trafficLights.size();
  const float numStepsTogether = 12;
  int numSampling = accSpeedPerLinePerTimeInterval.size() / tNumLanes;
  printf(">>calculateAndDisplayTrafficDensity numSampling %d\n", numSampling);


  RoadGraph::roadGraphEdgeIter_BI ei, eiEnd;
  const bool saveToFile = true;
  const bool updateSpeeds = true;

  if (saveToFile) {
    /////////////////////////////////
    // SAVE TO FILE
    QFile speedFile(QString::number(numOfPass) + "_average_speed.csv");
    QFile utilizationFile(QString::number(numOfPass) + "_utilization.csv");

    if (speedFile.open(QIODevice::ReadWrite) &&
        utilizationFile.open(QIODevice::ReadWrite)) {
      QTextStream streamS(&speedFile);
      QTextStream streamU(&utilizationFile);

      for (boost::tie(ei, eiEnd) = boost::edges(simRoadGraph->myRoadGraph_BI);
           ei != eiEnd; ++ei) {
        int numLanes = simRoadGraph->myRoadGraph_BI[*ei].numberOfLanes;

        if (numLanes == 0) {
          continue;  //edges with zero lines just skip
        }

        int numLane = edgeDescToLaneMapNum[*ei];
        //  0.8f to make easier to become red
        float maxVehicles = 0.5f * simRoadGraph->myRoadGraph_BI[*ei].edgeLength *
                            simRoadGraph->myRoadGraph_BI[*ei].numberOfLanes / (simParameters.s_0);

        if (maxVehicles < 1.0f) {
          maxVehicles = 1.0f;
        }

        streamS << simRoadGraph->myRoadGraph_BI[*ei].faci;
        streamU << simRoadGraph->myRoadGraph_BI[*ei].faci;

        for (int sa = 0; sa < numSampling - 1; sa++) {
          uint offset = sa * tNumLanes;

          // avarage speed
          float averageSpeed;

          if (numVehPerLinePerTimeInterval[numLane + offset] * numStepsTogether > 0) {
            averageSpeed = (accSpeedPerLinePerTimeInterval[numLane + offset]) / ((
                             float) numVehPerLinePerTimeInterval[numLane + offset]); //!!!!!
          } else {
            averageSpeed = -1.0f;
          }

          streamS << "," << averageSpeed;
          // average utilization
          float averageUtilization;
          averageUtilization = numVehPerLinePerTimeInterval[numLane + offset] /
                               (maxVehicles * numStepsTogether);
          averageUtilization = std::min(1.0f, averageUtilization);
          streamU << "," << averageUtilization;
        }

        streamS << "\n";
        streamU << "\n";
      }
    }

    speedFile.close();
    utilizationFile.close();
  }

  if (updateSpeeds) {

    ///////////////////////////////
    // COMPUTE AVG SPEED TO UPDATE NETWORK FOR MULTI STEP (and display)
    printPercentageMemoryUsed();

    if (DEBUG_SIMULATOR) {
      printf(">>calculateAndDisplayTrafficDensity Allocate memory numSampling %d\n",
             numSampling);
    }

    int count = 0;
    printPercentageMemoryUsed();
    printf(">>calculateAndDisplayTrafficDensity Process\n");

    for (boost::tie(ei, eiEnd) = boost::edges(simRoadGraph->myRoadGraph_BI);
         ei != eiEnd; ++ei) {
      int numLanes = simRoadGraph->myRoadGraph_BI[*ei].numberOfLanes;

      if (numLanes == 0) {
        continue;  //edges with zero lines just skip
      }

      int numLane = edgeDescToLaneMapNum[*ei];
      //0.8f to make easier to become red
      float maxVehicles = 0.5f * simRoadGraph->myRoadGraph_BI[*ei].edgeLength *
                          simRoadGraph->myRoadGraph_BI[*ei].numberOfLanes / (simParameters.s_0);

      if (maxVehicles < 1.0f) {
        maxVehicles = 1.0f;
      }

      float averageSpeed = 0.0f;
      float averageUtilization = 0.0f;

      for (int sa = 0; sa < numSampling - 1; sa++) {
        uint offset = sa * tNumLanes;

        ////////////////////////////////////////////////
        // average speed

        if (numVehPerLinePerTimeInterval[numLane + offset] * numStepsTogether > 0) {
          averageSpeed += (accSpeedPerLinePerTimeInterval[numLane + offset]) / ((
                            float) numVehPerLinePerTimeInterval[numLane + offset]); //!!!!!
        } else {
          averageSpeed += simRoadGraph->myRoadGraph_BI[*ei].maxSpeedMperSec;
        }

        ///////////////////////////////
        // average utilization
        averageUtilization += std::min(1.0f,
                                       numVehPerLinePerTimeInterval[numLane + offset] / (maxVehicles *
                                           numStepsTogether));
      }

      simRoadGraph->myRoadGraph_BI[*ei].averageSpeed.resize(1);
      simRoadGraph->myRoadGraph_BI[*ei].averageUtilization.resize(1);
      simRoadGraph->myRoadGraph_BI[*ei].averageSpeed[0] = averageSpeed / numSampling;
      simRoadGraph->myRoadGraph_BI[*ei].averageUtilization[0] = averageUtilization /
          numSampling;
    }
  }

  printf("\n<<calculateAndDisplayTrafficDensity\n");
}//

//////////////////////////////
// control the number of samples
static const uint numElements = 200;

bool B18TrafficSimulatorRender::getInterpolated(bool goodPoint,
    QVector3D newPos, QVector3D newDir, QVector3D &interPos, QVector3D &interDir) {
  // initialization
  if (positions.size() < numElements) {
    positions.push_back(newPos);
    directions.push_back(newDir);
    goodPoints.push_back(true);
    interPos = positions[0];
    interDir = directions[0];
    indexToRead = 0;
    return false;
  }

  interPos = positions[indexToRead];
  interDir = directions[indexToRead];

  if (goodPoints[indexToRead] == false) {
    // reconstruct!!!
    // 1. find the following good point and length
    QVector3D p_0 = positions[indexToRead];
    QVector3D d_0 = directions[indexToRead];
    QVector3D p_1, d_1;
    float intersectionLength = 0;
    bool foundCorrect = false;
    int index;
    std::vector<float> lengthToStart;

    for (int i = 1; i < numElements - 1; i++) { //note i=1
      index = (indexToRead + i) % numElements;
      int formerIndex = index - 1;

      if (formerIndex < 0) {
        formerIndex = numElements - 1;
      }

      intersectionLength += (positions[index] - positions[formerIndex]).length();
      lengthToStart.push_back(intersectionLength);

      if (goodPoints[index] == true) {
        foundCorrect = true;
        break;
      }

    }

    /*if(foundCorrect==false){
        printf("No enough elements to reconstruct!!!\n");
        exit(0);
    }*/
    p_1 = positions[index];
    d_1 = directions[index];

    // 2. update values with curve
    for (int i = (indexToRead + 1) % numElements, j = 1; i != index;
         i = (i + 1) % numElements, j++) { //skip first because it stays
      float currenLength = lengthToStart[j - 1];
      float t = currenLength / intersectionLength; //0-1
      // en.wikipedia.org/wiki/Hermite_curve
      // {p}(t) = (2t^3-3t^2+1){p}_0 + (t^3-2t^2+t){m}_0 + (-2t^3+3t^2){p}_1 +(t^3-t^2){m}_1
      float t2 = t * t;
      float t3 = t2 * t;
      float d_0_factor = 2.0f;
      positions[i] = (2 * (t3) - 3 * (t2) + 1) * p_0 + ((t3) - 2 *
                     (t2) + t) * d_0 * d_0_factor + (-2 * (t3) + 3 * (t2)) * p_1 + ((t3) -
                         (t2)) * d_1;

      // dir
      //int beforeIndex=i-1;
      //if(beforeIndex<0)beforeIndex=numElements-1;
      //directions[i]=(positions[i]-positions[beforeIndex]).normalized();
      t = t - 0.1f;

      if (t < 0) {
        t = 0;
      }

      t2 = t * t;
      t3 = t2 * t;
      QVector3D beforePos = (2 * (t3) - 3 * (t2) + 1) * p_0 + ((t3) - 2 *
                            (t2) + t) * d_0 * d_0_factor + (-2 * (t3) + 3 * (t2)) * p_1 + ((t3) -
                                (t2)) * d_1;
      directions[i] = (positions[i] - beforePos).normalized();
      // !! direction
      goodPoints[i] = true;
    }
  }

  positions[indexToRead] = newPos;
  directions[indexToRead] = newDir;
  goodPoints[indexToRead] = goodPoint;
  indexToRead = (indexToRead + 1) % numElements;
  return true;
  //printf("Pos %f %f %f Inter %f %f %f\n",newPos.x(),newPos.y(),newPos.z(),interPos.x(),interPos.y(),interPos.z());
}//


void B18TrafficLightRender::getInterpolated(uchar newTrafficLight,
    uchar &interTrafficLight) {
  if (trafficLight.size() < numElements) {
    trafficLight.push_back(newTrafficLight);
    interTrafficLight = trafficLight[0];
    indexToRead = 0;
    return;
  }

  interTrafficLight = trafficLight[indexToRead];
  trafficLight[indexToRead] = newTrafficLight;
  indexToRead = (indexToRead + 1) % numElements;
}//

}

