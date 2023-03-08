#pragma once
#include <QString>
#include <string>

#include "b18CommandLineVersion.h"

#include "src/benchmarker.h"

#include "roadGraphB2018Loader.h"
#include "qcoreapplication.h"

#include "sp/graph.h"
#include "traffic/b18TrafficSP.h"
#include "../roadGraphB2018Loader.h"
#include "accessibility.h"
#include <stdexcept>

#ifdef B18_RUN_WITH_GUI
#include "b18TestSimpleRoadAndOD.h"
#endif

namespace LC {

using namespace std::chrono;

void B18CommandLineVersion::runB18Simulation() {
  QSettings settings(QCoreApplication::applicationDirPath() + "/command_line_options.ini",
      QSettings::IniFormat);
  bool useCPU = settings.value("USE_CPU", false).toBool();
  bool useJohnsonRouting = settings.value("USE_JOHNSON_ROUTING", false).toBool();
  bool useSP = settings.value("USE_SP_ROUTING", false).toBool();
  bool loadPrevPaths = settings.value("LOAD_PREV_PATHS", false).toBool();
  bool savePrevPaths = settings.value("SAVE_PREV_PATHS", false).toBool();

  QString networkPath = settings.value("NETWORK_PATH").toString();
  const std::string networkPathSP = networkPath.toStdString();

  bool addRandomPeople = settings.value("ADD_RANDOM_PEOPLE", true).toBool();
  int limitNumPeople = settings.value("LIMIT_NUM_PEOPLE", -1).toInt(); // -1
  int numOfPasses = settings.value("NUM_PASSES", 1).toInt();
  const float deltaTime = settings.value("TIME_STEP", .5).toFloat();
  const float startSimulationH = settings.value("START_HR", 5).toFloat();
  const float endSimulationH = settings.value("END_HR", 12).toFloat();
  const bool showBenchmarks = settings.value("SHOW_BENCHMARKS", false).toBool();
  int rerouteIncrementMins = settings.value("REROUTE_INCREMENT", 30).toInt(); //in minutes
  std::string odDemandPath = settings.value("OD_DEMAND_FILENAME", "od_demand_5to12.csv").toString().toStdString();
  const bool runUnitTests = settings.value("RUN_UNIT_TESTS", false).toBool();

  ClientGeometry cg;
  std::vector<std::string> allParameters = {"GUI", "USE_CPU", "USE_JOHNSON_ROUTING",
                                            "USE_SP_ROUTING", "USE_PREV_PATHS",
                                            "NETWORK_PATH", "ADD_RANDOM_PEOPLE",
                                            "LIMIT_NUM_PEOPLE", "NUM_PASSES",
                                            "TIME_STEP", "START_HR", "END_HR",
                                            "SHOW_BENCHMARKS", "REROUTE_INCREMENT",
                                            "OD_DEMAND_FILENAME", "RUN_UNIT_TESTS"};

  for (const auto inputedParameter: settings.childKeys()) {
    if (inputedParameter.at(0) != QChar('#') // it's a comment
      && std::find(allParameters.begin(), allParameters.end(), inputedParameter.toStdString()) == allParameters.end()) {
      throw std::invalid_argument("Argument " + inputedParameter.toStdString() + " is invalid.");
    }
  }

  for (const auto parameter: allParameters) {
    if (!settings.childKeys().contains(QString::fromUtf8(parameter.c_str()))) {
      std::cout << "Argument " << parameter << " is missing from command_line_options. Setting it to its default value." << std::endl;
    }
  }

  if (rerouteIncrementMins < 0){
    throw std::invalid_argument("Invalid reroute increment value.");
  } else if (rerouteIncrementMins == 0) {
    // rerouteIncrementMins of 0 means static routing.
    // We set the rerouteIncrement as the maximum possible, so it only routes once
    float totalMinsSimulation = (endSimulationH - startSimulationH) * 60;
    rerouteIncrementMins = int(totalMinsSimulation);
    std::cout << "Since the reroute increment is 0, static routing will be used." << std::endl;
  } else {
    std::cout << "Rerouting every " << rerouteIncrementMins << " minutes" << std::endl;
  }

  if ((loadPrevPaths || savePrevPaths) && rerouteIncrementMins != 0) {
    throw std::invalid_argument("USE_PREV_PATHS is only allowed with static routing. Please set REROUTE_INCREMENT to 0 or REROUTE_INCREMENT to False.");
  }
    

  const parameters simParameters {
      settings.value("a",0.557040909258405).toDouble(),
      settings.value("b",2.9020578588167).toDouble(),
      settings.value("T",0.5433027817144876).toDouble(),
      settings.value("s_0",1.3807498735425845).toDouble()};


  std::cout << "Simulation parameters: "
            << "[a: " << simParameters.a 
            << ", b: " << simParameters.b
            << ", T: " << simParameters.T
            << ", s_0: " << simParameters.s_0
            << "]" << std::endl;

  if (showBenchmarks){
    Benchmarker::enableShowBenchmarks();
  }

  Benchmarker loadNetwork("Load_network", true);
  Benchmarker loadODDemandData("Load_OD_demand_data", true);
  
  B18TrafficSimulator b18TrafficSimulator(deltaTime, &cg.roadGraph, simParameters);
  
  const bool directed = true;
  const std::shared_ptr<abm::Graph>& street_graph = std::make_shared<abm::Graph>(directed, networkPathSP);
  std::vector<abm::graph::edge_id_t> all_paths;
  std::vector<std::vector<int>> all_paths_ch;
  loadNetwork.startMeasuring();

  const std::string& odFileName = networkPathSP + odDemandPath;
  std::cout << odFileName << " as OD file\n";
  RoadGraphB2018::loadABMGraph(networkPathSP, street_graph, (int) startSimulationH, (int) endSimulationH);
  loadNetwork.stopAndEndBenchmark();

  loadODDemandData.startMeasuring();
  const std::vector<std::array<abm::graph::vertex_t, 2>> all_od_pairs_ = B18TrafficSP::read_od_pairs_from_file(odFileName, startSimulationH, endSimulationH);
  const std::vector<float> dep_times = B18TrafficSP::read_dep_times(odFileName, startSimulationH, endSimulationH);
  loadODDemandData.stopAndEndBenchmark();
  
  if (useSP) {
	  //make the graph from edges file and load the OD demand from od file
	  printf("# of OD pairs = %d\n", all_od_pairs_.size());
    const float startTimeMins = startSimulationH * 60;
    const float endTimeMins = startTimeMins + rerouteIncrementMins;
    std::cout << "startTime: " << startTimeMins << ", endTime: " << endTimeMins << std::endl;
    b18TrafficSimulator.createB2018PeopleSP(startSimulationH, endSimulationH, limitNumPeople, addRandomPeople, street_graph, dep_times);
  } else {
    RoadGraphB2018::loadB2018RoadGraph(cg.roadGraph, networkPath);
    b18TrafficSimulator.createB2018People(startSimulationH, endSimulationH, limitNumPeople, addRandomPeople, useSP);
  }

  
  if (useCPU) {
    b18TrafficSimulator.simulateInCPU_MultiPass(numOfPasses, startSimulationH, endSimulationH,
        useJohnsonRouting);
  } else {
	  //if useSP, convert all_paths to indexPathVec format and run simulation
    b18TrafficSimulator.simulateInGPU(numOfPasses, startSimulationH, endSimulationH,
        useJohnsonRouting, useSP, street_graph, simParameters,
        rerouteIncrementMins, all_od_pairs_, dep_times,
        networkPathSP);
  }

}
}  // LC
