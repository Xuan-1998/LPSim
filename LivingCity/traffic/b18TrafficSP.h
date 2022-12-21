/************************************************************************************************
*		@desc Class that finds the path for each person using Johnsons
*		@author igaciad
************************************************************************************************/
#ifndef LC_B18_TRAFFIC_SP_H
#define LC_B18_TRAFFIC_SP_H

#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "src/benchmarker.h"
#include "src/linux_host_memory_logger.h"
#include "b18TrafficPerson.h"
#include "../RoadGraph/roadGraph.h"
#include "sp/graph.h"
//#include "sp/external/csv.h"
#include "omp.h"

#include "sp/config.h"
#include "sp/mpi_wrapper.h"

namespace LC {
class B18TrafficSP {
 public:	 
  static std::vector<abm::graph::vertex_t> compute_routes(int mpi_rank,
                                                          int mpi_size,
                                                          const std::shared_ptr<abm::Graph>& graph_,
                                                          const std::vector<std::array<abm::graph::vertex_t, 2>>& od_pairs);

  static std::vector<std::array<abm::graph::vertex_t, 2>> make_od_pairs(std::vector<B18TrafficPerson> trafficPersonVec,
                                                                        const int nagents);

  static std::vector<std::array<abm::graph::vertex_t, 2>> read_od_pairs_from_file(
    const std::string& filename,
    const float startSimulationH, const float endSimulationH,
    const int nagents = std::numeric_limits<int>::max());

  static void read_od_pairs_from_structure(
    const std::vector<std::array<abm::graph::vertex_t, 2>>& od_pairs,
    const float startSimulationH, const float endSimulationH);
  
  static std::vector<float> read_dep_times(const std::string& filename,
                                          const float startSimulationH,
                                          const float endSimulationH);

  static const std::vector<abm::graph::edge_id_t> loadPrevPathsFromFile(const std::string & networkPathSP);

  static std::vector<personPath> RoutingWrapper (
    const std::vector<std::array<abm::graph::vertex_t, 2>> & all_od_pairs_,
    const std::shared_ptr<abm::Graph>& street_graph,
    const std::vector<float>& dep_times,
    const float currentBatchStartTimeSecs,
    const float currentBatchEndTimeSecs,
    const int reroute_batch_number,
    std::vector<LC::B18TrafficPerson>& trafficPersonVec);

  static void initialize_person_to_init_edge(
    std::vector<abm::graph::edge_id_t>& all_paths,
    const std::shared_ptr<abm::Graph>& street_graph);

  static const void edgePreprocessingForRouting(
  std::vector<std::vector<long>>& edges_routing,
  std::vector<std::vector<double>> & edge_weights_routing,
  const std::shared_ptr<abm::Graph>& street_graph);

  static void filterODByTimeRange (
    const std::vector<std::array<abm::graph::vertex_t, 2>> & od_pairs,
    const std::vector<float> & dep_times_in_seconds,
    const float currentBatchStartTimeSecs,
    const float currentBatchEndTimeSecs,
    std::vector<abm::graph::vertex_t>& filtered_od_pairs_sources_,
    std::vector<abm::graph::vertex_t>& filtered_od_pairs_targets_,
    std::vector<float>& filtered_dep_times_,
    std::vector<uint>& pathsOrder);

  static std::vector<uint> convertPathsToCUDAFormat (
    const std::vector<personPath>& pathsInVertexes,
    const std::vector<uint> &edgeIdToLaneMapNum,
    const std::shared_ptr<abm::Graph>& graph_,
    std::vector<B18TrafficPerson>& trafficPersonVec);

  explicit B18TrafficSP(const std::shared_ptr<abm::Graph>& graph) : graph_{graph} {};
 private:
  //all od pairs
  std::shared_ptr<std::vector<std::array<abm::graph::vertex_t, 2>>> all_od_pairs_;
  
  //filtered od pairs
  std::shared_ptr<std::vector<std::array<abm::graph::vertex_t, 2>>> filtered_od_pairs_;

  //graph (street network)
  std::shared_ptr<abm::Graph> graph_;

  //all paths
  std::vector<abm::graph::vertex_t> all_paths_;

};
} //namespace LC

#endif  // LC_B18_TRAFFIC_SP_H
