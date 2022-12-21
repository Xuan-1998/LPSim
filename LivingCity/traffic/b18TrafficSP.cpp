#include "b18TrafficSP.h"

#include <boost/graph/exterior_property.hpp>
#include "src/linux_host_memory_logger.h"
#include "roadGraphB2018Loader.h"
#include "accessibility.h"
#include <math.h>

#define ROUTE_DEBUG 0
//#define DEBUG_JOHNSON 0

namespace LC {


////////////////
/////////////////////////////
using namespace boost;
using namespace std::chrono;

inline bool fileExists(const std::string& fileName) {
  std::ifstream f(fileName.c_str());
  return f.good();
}

typedef exterior_vertex_property<RoadGraph::roadBGLGraph_BI, float>
DistanceProperty;
typedef DistanceProperty::matrix_type DistanceMatrix;
typedef DistanceProperty::matrix_map_type DistanceMatrixMap;

// Convert OD pairs to SP graph format
std::vector<std::array<abm::graph::vertex_t, 2>> B18TrafficSP::make_od_pairs(std::vector<B18TrafficPerson> trafficPersonVec, 
									    const int nagents) {
  bool status = true;
  std::vector<std::array<abm::graph::vertex_t, 2>> od_pairs;
  try {
    abm::graph::vertex_t v1, v2;
    abm::graph::weight_t weight;
    printf("trafficPersonSize = %d\n", trafficPersonVec.size());
    for (int person = 0; person < trafficPersonVec.size(); person++) {
    //for (int person = 0; person < 1; person++) {
      v1 = trafficPersonVec[person].init_intersection;
      v2 = trafficPersonVec[person].end_intersection;
      std::array<abm::graph::vertex_t, 2> od = {v1, v2};
      od_pairs.emplace_back(od);
    }
    if (nagents != std::numeric_limits<int>::max())
      od_pairs.resize(nagents);
    
    sort( od_pairs.begin(), od_pairs.end() );
    //od_pairs.erase( unique( od_pairs.begin(), od_pairs.end() ), od_pairs.end() );

    /*
    //make sure all OD pairs are unique
    std::sort(od_pairs.begin(), od_pairs.end());
    auto last = std::unique(od_pairs.begin(), od_pairs.end());
      // using default comparison:
  std::array<abm::graph::vertex_t, 2>::iterator it;
  it = std::unique (od_pairs.begin(), od_pairs.end());   // 10 20 30 20 10 ?  ?  ?  ?
                                                         //                ^

  od_pairs.resize( std::distance(od_pairs.begin(),it) ); // 10 20 30 20 10
  */
  // using predicate comparison:
  //std::unique (od_pairs.begin(), od_pairs.end(), myfunction);   // (no changes)
    //for (int i = 0; i < od_pairs.size(); i++) {
	    //printf("vertex %d = %llu %llu\n", i, od_pairs[i].first, od_pairs[i].second);
	    //printf("vertex %d = %llu %llu\n", i, std::get<0>(od_pairs[i]), std::get<1>(od_pairs[i]));
    //}
    //od_pairs.erase(last, od_pairs.end());
    /*
    for (int i = 0; i < od_pairs.size(); i++) {
	    printf("vertex %d = %llu %llu\n", i, std::get<0>(od_pairs[i]), std::get<1>(od_pairs[i]));
    }
    */

  } catch (std::exception& exception) {
    std::cout << "Looping through trafficPersonVec doesn't work " << exception.what() << "\n";
    status = false;
  }
  return od_pairs;
}

void B18TrafficSP::read_od_pairs_from_structure(
  const std::vector<std::array<abm::graph::vertex_t, 2>>& od_pairs,
  const float startSimulationH,
  const float endSimulationH) {

  for(const auto & od_pair: od_pairs) {
    auto v1 = od_pair[0];
    auto v2 = od_pair[1];
    RoadGraphB2018::demandB2018.push_back(DemandB2018(1, v1, v2));
  }
}

// Read OD pairs file format
std::vector<std::array<abm::graph::vertex_t, 2>> B18TrafficSP::read_od_pairs_from_file(
  const std::string& filename,
  const float startSimulationH,
  const float endSimulationH,
  const int nagents) {
  std::vector<std::array<abm::graph::vertex_t, 2>> od_pairs;
  csvio::CSVReader<3> in(filename);
  in.read_header(csvio::ignore_extra_column, "dep_time", "origin", "destination");
  abm::graph::vertex_t v1, v2;
  abm::graph::weight_t weight;
  float dep_time;
  int count_outside_filter = 0;
  while (in.read_row(dep_time, v1, v2)) {
    if (dep_time >= startSimulationH * 3600 && dep_time < endSimulationH * 3600){
      std::array<abm::graph::vertex_t, 2> od = {v1, v2};
      od_pairs.emplace_back(od);
      RoadGraphB2018::demandB2018.push_back(DemandB2018(1, v1, v2)); //there is only one person for each OD pair
    } else {
      count_outside_filter++;
    }
  }
  if (count_outside_filter > 0){
    std::cout << "WARNING: Filtering " << count_outside_filter << " trips outside the input time range." << std::endl;
  }
  RoadGraphB2018::totalNumPeople = RoadGraphB2018::demandB2018.size();
  if (nagents != std::numeric_limits<int>::max())
    od_pairs.resize(nagents);
  return od_pairs;
}

// Read OD pairs file format
std::vector<float> B18TrafficSP::read_dep_times(
  const std::string& filename,
  const float startSimulationH,
  const float endSimulationH) {
  std::vector<float> dep_time_vec;
  csvio::CSVReader<1> in(filename);
  in.read_header(csvio::ignore_extra_column, "dep_time");
  float dep_time;
  int count_outside_filter = 0;
  while (in.read_row(dep_time)) {
    if (dep_time >= startSimulationH * 3600 && dep_time < endSimulationH * 3600){
      dep_time_vec.emplace_back(dep_time);
    } else {
      count_outside_filter++;
    }
  }
  if (count_outside_filter > 0) {
    std::cout << "WARNING: Filtering " << count_outside_filter << " trips outside the input time range." << std::endl;
  }
  return dep_time_vec;
}

void const B18TrafficSP::edgePreprocessingForRouting(
  std::vector<std::vector<long>>& edges_routing,
  std::vector<std::vector<double>> & edge_weights_routing,
  const std::shared_ptr<abm::Graph>& street_graph) {
  edges_routing.clear();
  edge_weights_routing.clear();
  edges_routing.reserve(street_graph->edges_.size());
  edge_weights_routing.reserve(street_graph->edges_.size());

  std::vector<double> edge_weights_routing_inside_vec;
  for (auto const& one_edge : street_graph->edges_) {
    // build node routes vector
    auto nodes = one_edge.first;
    auto nodeFrom = std::get<0>(nodes);
    auto nodeTo = std::get<1>(nodes);
    std::vector<long> edge_nodes = {nodeFrom, nodeTo};
    edges_routing.emplace_back(edge_nodes);

    // build weight vector
    std::shared_ptr<abm::Graph::Edge> edge = one_edge.second;
    abm::EdgeProperties anEdgeProperties = edge->second;
    double edge_weight = anEdgeProperties.weight;
    assert(edge_weight > 0);
    edge_weights_routing_inside_vec.emplace_back(edge_weight);
  }
  edge_weights_routing.emplace_back(edge_weights_routing_inside_vec);
  std::cout << "# nodes = " << street_graph->vertices_data_.size() << std::endl;
}

void B18TrafficSP::filterODByTimeRange(
    const std::vector<std::array<abm::graph::vertex_t, 2>> & od_pairs,
    const std::vector<float> & dep_times_in_seconds,
    const float currentBatchStartTimeSecs,
    const float currentBatchEndTimeSecs,
    std::vector<abm::graph::vertex_t>& filtered_od_pairs_sources_,
    std::vector<abm::graph::vertex_t>& filtered_od_pairs_targets_,
    std::vector<float>& filtered_dep_times_,
    std::vector<uint>& pathsOrder) {
  
  if (od_pairs.size() != dep_times_in_seconds.size()){
    throw std::runtime_error("Input from trips should match in size.");
  }

  filtered_od_pairs_sources_.clear();
  filtered_od_pairs_targets_.clear();
  pathsOrder.clear();
  
  std::cout << "Filtering in the range ["
            << currentBatchStartTimeSecs << ", "
            << currentBatchEndTimeSecs << ")." << std::endl;

  filtered_dep_times_.clear();
  for (uint person_id = 0; person_id < od_pairs.size(); person_id++) {
    if (isgreaterequal(dep_times_in_seconds.at(person_id), currentBatchStartTimeSecs)
        && isless(dep_times_in_seconds.at(person_id), currentBatchEndTimeSecs)) {
      filtered_od_pairs_sources_.push_back(od_pairs.at(person_id).at(0));
      filtered_od_pairs_targets_.push_back(od_pairs.at(person_id).at(1));
      filtered_dep_times_.push_back(dep_times_in_seconds.at(person_id));
      pathsOrder.push_back(person_id);
    }
  }
}

std::string convertSecondsToTime(const float seconds) {
  std::string strHour = std::to_string(int(seconds) / 3600);
  std::string strMin = std::to_string(int(seconds) / 60 % 60);

  if (strHour.size() == 1)
    strHour = "0" + strHour;
  
  if (strMin.size() == 1)
    strMin = "0" + strMin;

  return strHour + ":" + strMin;
}

const std::vector<abm::graph::edge_id_t> B18TrafficSP::loadPrevPathsFromFile(
  const std::string & networkPathSP){

  std::vector<abm::graph::edge_id_t> paths_SP;
  // open file    
  const std::string& pathsFileName = networkPathSP + "all_paths_ch.txt";
  std::cout << "Loading " << pathsFileName << " as paths file" << std::endl;
  std::ifstream inputFile(pathsFileName);
  // test file open   
  if (inputFile) {        
    abm::graph::vertex_t value;
    // read the elements in the file into a vector  
    while (inputFile >> value) {
      paths_SP.push_back(value);
    }
  } else {
    throw std::runtime_error("Could not load previous paths file.");
  }

  const std::vector<abm::graph::edge_id_t> const_paths_SP (paths_SP);
  
  return const_paths_SP;
}

std::vector<personPath> B18TrafficSP::RoutingWrapper (
  const std::vector<std::array<abm::graph::vertex_t, 2>> & all_od_pairs_,
  const std::shared_ptr<abm::Graph>& street_graph,
  const std::vector<float>& dep_times,
  const float currentBatchStartTimeSecs,
  const float currentBatchEndTimeSecs,
  const int reroute_batch_number,
  std::vector<LC::B18TrafficPerson>& trafficPersonVec) {

  if (all_od_pairs_.size() != dep_times.size())
    throw std::runtime_error("RoutingWrapper received od_pairs and dep_times with different sizes.");
  
  std::vector<abm::graph::vertex_t> filtered_od_pairs_sources_;
  std::vector<abm::graph::vertex_t> filtered_od_pairs_targets_;
  std::vector<float> filtered_dep_times_;

  //filter the next set of od pair/departures in the next increment
  std::vector<uint> pathsOrder;
  B18TrafficSP::filterODByTimeRange(all_od_pairs_,
                                    dep_times,
                                    currentBatchStartTimeSecs,
                                    currentBatchEndTimeSecs,
                                    filtered_od_pairs_sources_,
                                    filtered_od_pairs_targets_,
                                    filtered_dep_times_,
                                    pathsOrder);
  
  std::cout << "Simulating trips with dep_time between "
    << convertSecondsToTime(currentBatchStartTimeSecs)
    << "(" << currentBatchStartTimeSecs / 60 << " in minutes)"
    << " and " << convertSecondsToTime(currentBatchEndTimeSecs)
    << "(" << currentBatchEndTimeSecs / 60 << " in minutes)" << std::flush;
  std::cout << ". Trips in this time range: " << filtered_od_pairs_sources_.size() << "/" << dep_times.size() << std::endl;

  std::vector<std::vector<long>> edges_routing;
  std::vector<std::vector<double>> edge_weights_routing;
  B18TrafficSP::edgePreprocessingForRouting(edges_routing, edge_weights_routing, street_graph);

  Benchmarker routingCH("Routing_CH_batch_" + std::to_string(reroute_batch_number), true);
  routingCH.startMeasuring();
  //MTC::accessibility::Accessibility *graph_ch = new MTC::accessibility::Accessibility((int) street_graph->vertices_data_.size(), edges_routing, edge_weights_routing, false);
   std::unique_ptr<MTC::accessibility::Accessibility> graph_ch(
    new MTC::accessibility::Accessibility((int) street_graph->vertices_data_.size(),
    edges_routing, edge_weights_routing, false));
  std::vector<std::vector<abm::graph::edge_id_t> > paths_ch = graph_ch->Routes(filtered_od_pairs_sources_, filtered_od_pairs_targets_, 0);
  routingCH.stopAndEndBenchmark();

  std::cout << "# of paths = " << paths_ch.size() << std::endl;

  std::vector<personPath> currentBatchPaths;
  currentBatchPaths.reserve(paths_ch.size());
  for (int i = 0; i < paths_ch.size(); i++){
    personPath aPersonPath;
    aPersonPath.person_id = pathsOrder.at(i);
    aPersonPath.pathInVertexes = paths_ch.at(i);
    currentBatchPaths.push_back(aPersonPath);
  }

  return currentBatchPaths;
}


std::vector<uint> B18TrafficSP::convertPathsToCUDAFormat (
  const std::vector<personPath>& pathsInVertexes,
  const std::vector<uint> &edgeIdToLaneMapNum,
  const std::shared_ptr<abm::Graph>& graph_,
  std::vector<B18TrafficPerson>& trafficPersonVec) {
  std::vector<uint> allPathsInEdgesCUDAFormat;

  for (const personPath & aPersonPath: pathsInVertexes) {
    assert(aPersonPath.person_id < trafficPersonVec.size());
    int personPathLength = 0;

    // assign current indexPathInit and assert there are no reassignments
    if (trafficPersonVec[aPersonPath.person_id].indexPathInit != INIT_EDGE_INDEX_NOT_SET &&
          trafficPersonVec[aPersonPath.person_id].indexPathInit != allPathsInEdgesCUDAFormat.size()) {
      std::string errorMessage = "Error! person_id " + std::to_string(aPersonPath.person_id)
      + " has indexPathInit " + std::to_string(trafficPersonVec[aPersonPath.person_id].indexPathInit)
      + " while we're trying to set it as " + std::to_string(allPathsInEdgesCUDAFormat.size());
      throw std::runtime_error(errorMessage);
    }
    trafficPersonVec[aPersonPath.person_id].indexPathInit = allPathsInEdgesCUDAFormat.size();

    // convert the path from vertexes to edges in CUDA format (laneMapNum)
    for (int j=0; j < aPersonPath.pathInVertexes.size()-1; j++) {
      auto vertexFrom = aPersonPath.pathInVertexes[j];
      auto vertexTo = aPersonPath.pathInVertexes[j+1];
      auto oneEdgeInCPUFormat = graph_->edge_ids_[vertexFrom][vertexTo];
      assert(oneEdgeInCPUFormat < edgeIdToLaneMapNum.size());
      allPathsInEdgesCUDAFormat.emplace_back(edgeIdToLaneMapNum[oneEdgeInCPUFormat]);
      personPathLength++;
    }
    allPathsInEdgesCUDAFormat.emplace_back(END_OF_PATH);
    trafficPersonVec[aPersonPath.person_id].path_length_cpu = aPersonPath.pathInVertexes.size() - 1; // not including END_OF_PATH

    assert(aPersonPath.pathInVertexes.size() > 1 ||
      allPathsInEdgesCUDAFormat[trafficPersonVec[aPersonPath.person_id].indexPathInit] == END_OF_PATH);
    assert(aPersonPath.pathInVertexes.size() > 1 ||
      trafficPersonVec[aPersonPath.person_id].path_length_cpu == 0);
    assert(trafficPersonVec[aPersonPath.person_id].indexPathInit != INIT_EDGE_INDEX_NOT_SET);
  }

  std::cout << "Converted to CUDA format" << std::endl;

  return allPathsInEdgesCUDAFormat;
}


std::vector<abm::graph::vertex_t> B18TrafficSP::compute_routes(int mpi_rank,
                                                          int mpi_size,
                                                          const std::shared_ptr<abm::Graph>& graph_,
                                                          const std::vector<std::array<abm::graph::vertex_t, 2>>& od_pairs) {
  //! All paths
  std::vector<abm::graph::vertex_t> all_paths_;
  //! All paths indices
  std::vector<std::array<abm::graph::vertex_t, 3>> all_paths_idx_;

  //std::vector<std::array<abm::graph::vertex_t, 2>> od_pairs;

/*
#ifdef USE_MPI
  // Create MPI pair type
  MPI_Datatype pair_t;
  MPI_Type_vector(2, 1, 1, MPI_LONG_LONG_INT, &pair_t);
  MPI_Type_commit(&pair_t);

  // Calculate chunk size to split router
  int chunk_size = od_pairs.size() / mpi_size;
  MPI_Bcast(&chunk_size, 1, MPI_LONG_LONG_INT, 0, MPI_COMM_WORLD);
  od_pairs.resize(chunk_size);
  // Send route chunks to different compute nodes
  MPI_Scatter(od_pairs.data(), chunk_size, pair_t, od_pairs.data(),
              od_pairs.size(), pair_t, 0, MPI_COMM_WORLD);

  // Calculate the remaining chunk of od_pairs and add to rank 0
  int chunk_remainder = od_pairs.size() % mpi_size;
  if (mpi_rank == 0) {
    od_pairs.insert(od_pairs.begin(), od_pairs.end() - chunk_remainder,
                    od_pairs.end());
  }
#else
*/
  //od_pairs = od_pairs;
//#endif
  // Paths (vector of edges)
  std::vector<abm::graph::vertex_t> paths;
  paths.reserve(graph_->nedges());
  // Indices of start of path and length for each agent
  std::vector<std::array<abm::graph::vertex_t, 3>> paths_idx;
  paths_idx.reserve(od_pairs.size());

#pragma omp parallel for schedule(dynamic)
  for (abm::graph::vertex_t i = 0; i < od_pairs.size(); ++i) {
    //std::cout << "index " << i << "\n";
    //const auto sp = graph_->dijkstra_edges(od_pairs[i][0], od_pairs[i][1]);
    const auto sp = graph_->dijkstra_edges(od_pairs[i][0], od_pairs[i][1]);
    //printf("od pair 1 o = %d, od pair 1 d = %d\n", od_pairs[i][0], od_pairs[i][1]);
#pragma omp critical
    {
      paths_idx.emplace_back(std::array<abm::graph::vertex_t, 3>(
          {i, static_cast<abm::graph::vertex_t>(paths.size()),
           static_cast<abm::graph::vertex_t>(sp.size())}));
            paths.insert(std::end(paths), std::begin(sp), std::end(sp));
    }
  }
  // Get all paths and indices
  std::cout << "paths size " << paths.size() << "\n";
  all_paths_ = abm::gather_vectors_ids(paths);
  // for (int i = 0; i < all_paths_.size(); i++) {
  //   printf("all_paths = %d\n", all_paths_[i]);
  //}
  all_paths_idx_ = abm::gather_vector_arrays(paths_idx);
/*
#ifdef USE_MPI
  MPI_Type_free(&pair_t);
#endif
*/
  return all_paths_;
}

}  // Closing namespace LC

