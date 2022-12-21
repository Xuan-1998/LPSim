#ifndef ABM_GRAPH_H_
#define ABM_GRAPH_H_

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
#include <QVector2D>
#include <QVector3D>
#include <stdint.h>
#include <cfloat>
#include <QString>
#include <string>


#include "external/csv.h"
#include "tsl/robin_map.h"

#include "config.h"

namespace abm {
  struct EdgeProperties {
    float length;
    float lanes;
    float max_speed_limit_mps; // meters per second
    abm::graph::weight_t weight;
  };

//! \brief Graph class to store vertices and edge and compute shortest path
//! \details Graph class has Priority Queue Dijkstra algorithm for SSSP
class Graph {
  public:
  //! Edge {{v1, v2}, weight}
  using Edge =
      std::pair<std::pair<graph::vertex_t, graph::vertex_t>, abm::EdgeProperties>;

  //! Construct directed / undirected graph
  //! \param[in] directed Defines if the graph is directed or not
  explicit Graph(bool directed, std::string networkPath) {
    this->directed_ = directed;

    // --- we get the max vertex id
    const std::string& nodeFileName = networkPath + "nodes.csv";
    csvio::CSVReader<1> inMaxVertexIndex(nodeFileName);
    inMaxVertexIndex.read_header(csvio::ignore_extra_column, "index");
    abm::graph::vertex_t osmid, nodeIndex;
    std::string ref;
    std::string highway;
    float lat, lon;
    abm::graph::vertex_t maxVertexIndexFound = 0;
    while (inMaxVertexIndex.read_row(nodeIndex)) {
      maxVertexIndexFound = std::max(nodeIndex, maxVertexIndexFound);
    }
    this->edge_ids_ = std::vector<tsl::robin_map<graph::vertex_t, graph::edge_id_t>>(maxVertexIndexFound+1);
    this->nodeIndex_to_osmid_ = std::vector<graph::vertex_t>(maxVertexIndexFound+1);
  }

  //! Return number of vertices
  unsigned nvertices() const { return nvertices_; }

  //! Number of edges
  graph::vertex_t nedges() const { return edges_.size(); }

  //! Add edge to graph
  //! \param[in] vertex1 ID of vertex1
  //! \param[in] vertex2 ID of vertex2
  //! \param[in] weight Weight of edge connecting vertex 1 and 2
  //! \param[in] edge_id ID of edge
  void add_edge(const graph::vertex_t vertex1, const graph::vertex_t vertex2, const float length, const float lanes,
    const float max_speed_limit_mps, const graph::vertex_t edgeid);
  //! Update edge of a graph
  //! \param[in] vertex1 ID of vertex1
  //! \param[in] vertex2 ID of vertex2
  //! \param[in] weight Weight of edge connecting vertex 1 and 2
  void update_edge(graph::vertex_t vertex1, graph::vertex_t vertex2, graph::weight_t weight);

  //! Remove edge from graph
  //! \param[in] vertex1 ID of vertex1
  //! \param[in] vertex2 ID of vertex2
  void remove_edge(graph::vertex_t vertex1, graph::vertex_t vertex2);

  //! Generate a simple graph
  void generate_simple_graph();

  //! Read MatrixMarket graph file format
  //! \param[in] filename Name of input MatrixMarket file
  //! \retval status File read status
  bool read_graph_matrix_market(const std::string& filename);

  //! Read OSM graph file format
  //! \param[in] filename Name of input MatrixMarket file
  //! \retval status File read status
  bool read_graph_osm(const std::string& filename);

  //! Read OSM graph file format
  //! \param[in] filename Name of input MatrixMarket file
  //! \retval status File read status
  bool read_vertices(const std::string& filename);

  //! Compute the shortest path using priority queue
  //! \param[in] source ID of source vertex1
  //! \param[in] destination ID of destination vertex
  //! \retval path Vertices of the path from source to destination
  std::vector<graph::vertex_t> dijkstra(graph::vertex_t source,
                                        graph::vertex_t destination);

  //! Compute the Dijkstra shortest path and return vertices
  //! \param[in] source ID of source vertex1
  //! \param[in] destination ID of destination vertex
  //! \retval route Vertices pair of the route from source to destination
  std::vector<std::array<graph::vertex_t, 2>> dijkstra_vertices(
      graph::vertex_t source, graph::vertex_t destination);

  //! Compute the Dijkstra shortest path and return vertices suitable for UAL
  //! \param[in] source ID of source vertex1
  //! \param[in] destination ID of destination vertex
  //! \retval route Vertices pair of the route from source to destination
  std::vector<graph::vertex_t> dijkstra_vertices_ual(
      graph::vertex_t source, graph::vertex_t destination);

  //! Compute the Dijkstra shortest path and return edges
  //! \param[in] source ID of source vertex1
  //! \param[in] destination ID of destination vertex
  //! \retval route_edges Edges of the route from source to destination
  std::vector<graph::vertex_t> dijkstra_edges(graph::vertex_t source,
                                              graph::vertex_t destination);

  //! Path cost from edge ids
  //! \param[in] path Vertices of the path from source to destination
  //! \retval cost Cost of traversed path
  abm::graph::weight_t path_cost(
      const std::vector<std::array<graph::vertex_t, 2>>& path);

  //! Path cost from vertices ids
  //! \param[in] path Edges of the path from source to destination
  //! \retval cost Cost of traversed path
  abm::graph::weight_t path_cost(const std::vector<graph::vertex_t>& path);

// private:
  //! Assign number of vertices
  //! \param[in] nvertices Number of vertices in graph
  void assign_nvertices(unsigned nvertices) { this->nvertices_ = nvertices; }

  // Directed / undirected
  bool directed_{false};
  // Number of graph vertices
  unsigned nvertices_{std::numeric_limits<unsigned>::max()};
  // Edge id
  graph::vertex_t edgeid_{0};
  // Max id of vertex
  graph::vertex_t max_vertex_id_{std::numeric_limits<graph::vertex_t>::min()};
  //Vertex data
  std::map<graph::vertex_t, QVector3D> vertices_data_;
  // nodeIndex to osmid
  std::vector<graph::vertex_t> nodeIndex_to_osmid_;
  // Edges - vertices to edge pointer
  std::map<std::tuple<graph::vertex_t, graph::vertex_t>, std::shared_ptr<Edge>>
      edges_;
  // Edges - edge pointer to vertices
  std::map<std::shared_ptr<Edge>, std::tuple<graph::vertex_t, graph::vertex_t>>
      edge_pointer_to_vertices_;
  //Edges to an index
  std::map <graph::edge_id_t, graph::vertex_t> edge_vertex_map_;
  // adjacency list with iteration over each edge
  tsl::robin_map<graph::vertex_t, std::vector<std::shared_ptr<Edge>>>
      vertex_edges_;

  tsl::robin_map<graph::vertex_t, std::vector<std::shared_ptr<Edge>>>
      vertex_in_edges_;

  tsl::robin_map<graph::vertex_t, std::vector<std::shared_ptr<Edge>>>
      vertex_out_edges_;
  // Vertices and counts
  tsl::robin_map<graph::vertex_t, graph::vertex_t> vertices_;

  // Global edges
  std::vector<tsl::robin_map<graph::vertex_t, graph::edge_id_t>> edge_ids_;

  //person to their initial edge
  std::vector<graph::edge_id_t> person_to_init_edge_;

  std::map<graph::edge_id_t, std::tuple<graph::vertex_t, graph::vertex_t>>
      edge_ids_to_vertices;
  // Vertices and counts
  tsl::robin_map<graph::edge_id_t, graph::weight_t> edge_costs_;
};

}  // namespace abm
#endif  // ABM_GRAPH_H_
