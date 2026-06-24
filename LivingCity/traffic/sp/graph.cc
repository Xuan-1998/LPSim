#include "graph.h"
#include <string>
#include <fstream>
#include <cassert>

inline void abm::Graph::add_edge(const graph::vertex_t vertex_from, const graph::vertex_t vertex_to,
  const float length, const float lanes, const float max_speed_limit_mps,
  const graph::vertex_t edgeid = std::numeric_limits<abm::graph::vertex_t>::max()) {
  
  EdgeProperties newEdgeProperties;
  newEdgeProperties.length = length;
  newEdgeProperties.max_speed_limit_mps = max_speed_limit_mps;
  newEdgeProperties.lanes = lanes;
  newEdgeProperties.weight = length / max_speed_limit_mps;

  graph::vertex_t vertex1, vertex2;
  if (this->directed_) {
    vertex1 = vertex_from;
    vertex2 = vertex_to;
  } else {
    // if the graph is not directed we index first by the smallest
    vertex1 = std::min(vertex_from, vertex_to);
    vertex2 = std::max(vertex_from, vertex_to);
  }

  // Create a map of vertices
  if (vertices_.find(vertex1) == vertices_.end())
    vertices_[vertex1] = vertices_.size();
  if (vertices_.find(vertex2) == vertices_.end())
    vertices_[vertex2] = vertices_.size();


  // Create an edge
  auto edge = std::make_shared<Graph::Edge>(std::make_pair(std::make_pair(vertex1, vertex2), newEdgeProperties));
  edges_[std::make_tuple(vertex1, vertex2)] = edge;
  edge_pointer_to_vertices_[edge] = std::make_tuple(vertex1, vertex2);

  // Add edge id
  if (edgeid == std::numeric_limits<abm::graph::vertex_t>::max()) {
    edge_ids_[vertex1][vertex2] = this->edgeid_;
    edge_costs_[this->edgeid_] = newEdgeProperties.weight;
    this->edgeid_ += 1;
  } else {
    edge_ids_[vertex1][vertex2] = edgeid;
    edge_ids_to_vertices[edgeid] = std::make_tuple(vertex1, vertex2);
    edge_costs_[edgeid] = newEdgeProperties.weight;
  }

  // Vertex 1
  auto vertex1_edges = vertex_edges_[vertex1];
  vertex1_edges.emplace_back(edge);
  vertex_edges_[vertex1] = std::vector<std::shared_ptr<Graph::Edge>>(vertex1_edges);

  //out edges vertex 1
  auto vertex1_out_edges = vertex_out_edges_[vertex1];
  vertex1_out_edges.emplace_back(edge);
  vertex_out_edges_[vertex1] = std::vector<std::shared_ptr<Graph::Edge>>(vertex1_out_edges);

  //in edges vertex 2
  auto vertex2_in_edges = vertex_in_edges_[vertex2];
  vertex2_in_edges.emplace_back(edge);
  vertex_in_edges_[vertex2] = std::vector<std::shared_ptr<Graph::Edge>>(vertex2_in_edges);

  if (!this->directed_) {
    // Vertex 2
    auto vertex2_edges = vertex_edges_[vertex2];
    vertex2_edges.emplace_back(edge);
    vertex_edges_[vertex2] = std::vector<std::shared_ptr<Graph::Edge>>(vertex2_edges);

    //out edges vertex 2
    auto vertex2_out_edges = vertex_out_edges_[vertex2];
    vertex2_out_edges.emplace_back(edge);
    vertex_out_edges_[vertex2] = std::vector<std::shared_ptr<Graph::Edge>>(vertex2_out_edges);

    //in edges vertex 1
    auto vertex1_in_edges = vertex_in_edges_[vertex1];
    vertex1_in_edges.emplace_back(edge);
    vertex_in_edges_[vertex1] = std::vector<std::shared_ptr<Graph::Edge>>(vertex1_in_edges);
  }
}

// Update edge
void abm::Graph::update_edge(abm::graph::vertex_t vertex1,
                             abm::graph::vertex_t vertex2,
                             abm::graph::weight_t weight) {
  // Get pointer to specified edge connecting vertex 1 and 2
  auto edge = edges_.at(std::make_tuple(vertex1, vertex2));
  edge->second.weight = weight;
}

// Remove edge
void abm::Graph::remove_edge(abm::graph::vertex_t vertex1,
                             abm::graph::vertex_t vertex2) {
  auto edge = edges_[std::make_tuple(vertex1, vertex2)];
  edges_.erase(edges_.find(std::make_tuple(vertex1, vertex2)));

  auto v1edge = vertex_edges_.at(vertex1);
  auto v2edge = vertex_edges_.at(vertex2);

  v1edge.erase(std::remove(v1edge.begin(), v1edge.end(), edge));
  v2edge.erase(std::remove(v2edge.begin(), v2edge.end(), edge));

  vertex_edges_[vertex1] = v1edge;
  vertex_edges_[vertex2] = v2edge;
}
/*
// Read MatrixMarket graph file format
bool abm::Graph::read_graph_matrix_market(const std::string& filename) {
  bool status = true;
  try {
    std::fstream file;
    file.open(filename.c_str(), std::ios::in);
    if (file.is_open() && file.good()) {
      // Line
      std::string line;
      bool header = true;
      double ignore;
      while (std::getline(file, line)) {
        std::istringstream istream(line);
        int v1, v2;
        double weight;
        unsigned nvertices;
        // ignore comment lines (# or !) or blank lines
        if ((line.find('#') == std::string::npos) &&
            (line.find('%') == std::string::npos) && (line != "")) {
          if (header) {
            // Ignore header
            istream >> nvertices;
            while (istream.good()) istream >> ignore;
            header = false;
            this->assign_nvertices(nvertices + 1);
          }
          while (istream.good()) {
            // Read vertices edges and weights
            istream >> v1 >> v2 >> weight;
            this->add_edge(v1, v2, weight);
          }
        }
      }
      std::cout << "Graph summary #edges: " << this->edges_.size()
                << " #vertices: " << this->nvertices_ << "\n";
    } else {
      throw std::runtime_error("Input file not found");
    }
  } catch (std::exception& exception) {
    std::cout << "Read matrix market file: " << exception.what() << "\n";
    status = false;
  }
  return status;
}
*/
// Read graph edges from CSV — supports both the 8-column format
// (uniqueid, osmid_u, osmid_v, length, lanes, speed_mph, u, v)
// and the legacy 6-column format (uniqueid, u, v, length, lanes, speed_mph).
bool abm::Graph::read_graph_osm(const std::string& filename) {
  bool status = true;
  std::cout << "reading graph osm" << std::endl;

  // Detect format from header
  std::ifstream probe(filename);
  std::string headerLine;
  std::getline(probe, headerLine);
  probe.close();
  bool hasOsmidColumns = (headerLine.find("osmid_u") != std::string::npos);

  try {
    abm::graph::vertex_t nvertices = 0;
    float length, lanes, speed_mph;
    abm::graph::vertex_t index = 0;
    abm::graph::edge_id_t edgeid;
    abm::graph::vertex_t v1, v2;

    if (hasOsmidColumns) {
      csvio::CSVReader<8> in(filename);
      in.read_header(csvio::ignore_extra_column, "uniqueid", "osmid_u", "osmid_v", "length", "lanes", "speed_mph", "u", "v");
      abm::graph::vertex_t osmid_v1, osmid_v2;
      while (in.read_row(edgeid, osmid_v1, osmid_v2, length, lanes, speed_mph, v1, v2)) {
        float max_speed_limit_mps = (speed_mph / 3600.0f) * 1609.34f;
        if (edges_.find(std::make_pair(v1, v2)) == edges_.end()) {
          this->add_edge(v1, v2, length, lanes, max_speed_limit_mps, edgeid);
        }
        ++nvertices;
        edge_vertex_map_[v1] = index;
        ++index;
      }
    } else {
      // Legacy 6-column format: u/v are osmids that need mapping to sequential indices.
      // Build osmid→index map from nodeIndex_to_osmid_ (populated by read_vertices)
      std::unordered_map<abm::graph::vertex_t, abm::graph::vertex_t> osmid_to_idx;
      for (size_t i = 0; i < nodeIndex_to_osmid_.size(); i++) {
        abm::graph::vertex_t osmid = nodeIndex_to_osmid_[i];
        if (osmid != 0) {
          osmid_to_idx[osmid] = i;
        }
      }
      std::cout << "Built osmid->index map with " << osmid_to_idx.size() << " entries" << std::endl;

      csvio::CSVReader<6> in(filename);
      in.read_header(csvio::ignore_extra_column, "uniqueid", "u", "v", "length", "lanes", "speed_mph");
      abm::graph::vertex_t raw_v1, raw_v2;
      while (in.read_row(edgeid, raw_v1, raw_v2, length, lanes, speed_mph)) {
        // Map osmid to sequential index if needed
        v1 = raw_v1;
        v2 = raw_v2;
        if (v1 >= this->edge_ids_.size() && osmid_to_idx.count(raw_v1)) {
          v1 = osmid_to_idx[raw_v1];
        }
        if (v2 >= this->edge_ids_.size() && osmid_to_idx.count(raw_v2)) {
          v2 = osmid_to_idx[raw_v2];
        }
        if (v1 >= this->edge_ids_.size() || v2 >= this->edge_ids_.size()) {
          continue; // skip edges with unmappable vertices
        }
        float max_speed_limit_mps = (speed_mph / 3600.0f) * 1609.34f;
        if (edges_.find(std::make_pair(v1, v2)) == edges_.end()) {
          this->add_edge(v1, v2, length, lanes, max_speed_limit_mps, edgeid);
        }
        ++nvertices;
        edge_vertex_map_[v1] = index;
        ++index;
      }
    }

    std::cout << "total edges = " << index << "\n";
    this->max_edge_id_ = index;
    this->assign_nvertices(nvertices);
    std::cout << "# of edges: " << this->edges_.size() << "\n";

  } catch (std::exception& exception) {
    std::cout << "Read OSM file: " << exception.what() << "\n";
    status = false;
  }

  return status;
}

bool abm::Graph::read_vertices(const std::string& filename) {
  QVector2D minBox(FLT_MAX, FLT_MAX);
  QVector2D maxBox(-FLT_MAX, -FLT_MAX);
  float scale = 1.0f;
  float sqSideSz = std::max<float>(maxBox.x() - minBox.x(),
                                   maxBox.y() - minBox.y()) * scale * 0.5f;
  QVector3D centerV(-minBox.x(), -minBox.y(), 0);
  QVector3D centerAfterSc(-sqSideSz, -sqSideSz, 0);
  bool status = true;

  // Detect whether the CSV has an "index" column by reading the header line
  std::ifstream probe(filename);
  std::string headerLine;
  std::getline(probe, headerLine);
  probe.close();
  bool hasIndex = (headerLine.find("index") != std::string::npos);

  float lat, lon;
  abm::graph::vertex_t nodeIndex, osmid;
  std::string ref, highway;

  if (hasIndex) {
    csvio::CSVReader<6> in(filename);
    in.read_header(csvio::ignore_extra_column, "osmid", "x", "y", "ref", "highway", "index");
    while (in.read_row(osmid, lat, lon, ref, highway, nodeIndex)) {
      this->nodeIndex_to_osmid_[nodeIndex] = osmid;
      QVector3D pos(lat, lon, 0);
      pos += centerV;
      pos *= scale;
      pos += centerAfterSc;
      pos.setX(pos.x() * -1.0f);
      vertices_data_[nodeIndex] = pos;
    }
  } else {
    // No index column — assign sequential indices based on row order
    csvio::CSVReader<5> in(filename);
    in.read_header(csvio::ignore_extra_column, "osmid", "x", "y", "ref", "highway");
    abm::graph::vertex_t autoIndex = 0;
    while (in.read_row(osmid, lat, lon, ref, highway)) {
      nodeIndex = autoIndex++;
      this->nodeIndex_to_osmid_[nodeIndex] = osmid;
      QVector3D pos(lat, lon, 0);
      pos += centerV;
      pos *= scale;
      pos += centerAfterSc;
      pos.setX(pos.x() * -1.0f);
      vertices_data_[nodeIndex] = pos;
    }
  }

  std::cout << "# of vertices: " << vertices_data_.size() << "\n";
  return status;
}

// Dijktra shortest paths from src to a vertex
std::vector<abm::graph::vertex_t> abm::Graph::dijkstra(
    abm::graph::vertex_t source, abm::graph::vertex_t destination) {

  // Using lambda to compare elements.
  auto compare =
      [](std::pair<abm::graph::weight_t, abm::graph::vertex_t> left,
         std::pair<abm::graph::weight_t, abm::graph::vertex_t> right) {
        return left.first > right.first;
      };


  // Create a priority queue to store weights and vertices
  std::priority_queue<
      std::pair<abm::graph::weight_t, abm::graph::vertex_t>,
      std::vector<std::pair<abm::graph::weight_t, abm::graph::vertex_t>>,
      decltype(compare)>
      priority_queue(compare);

  // Create a vector for distances and initialize all to max
  std::vector<graph::weight_t> distances;
  distances.resize(this->vertices_.size(),
                   std::numeric_limits<abm::graph::weight_t>::max());
  // Parent array to store shortest path tree
  std::vector<graph::vertex_t> parent;
  parent.resize(this->vertices_.size(), -1);

  std::vector<abm::graph::vertex_t> path;
  if (vertices_.find(source) == vertices_.end() ||
      vertices_.find(destination) == vertices_.end())
    return path;

  // Insert source itself in priority queue & initialize its distance as 0.
  priority_queue.push(std::make_pair(0., source));
  distances[vertices_.at(source)] = 0.;

  // Looping till priority queue becomes empty (or all
  // distances are not finalized)
  while (!priority_queue.empty()) {
    // {min_weight, vertex} sorted based on weights (distance)
    abm::graph::vertex_t u = priority_queue.top().second;
    priority_queue.pop();

    // Break if destination is reached
    if (u == destination) break;

    // Get all adjacent vertices of a vertex
    for (const auto& edge : vertex_edges_[u]) {
      // Get vertex label and weight of neighbours of u.
      const abm::graph::vertex_t neighbour = edge->first.second;
      const abm::graph::weight_t weight = edge->second.length;

      // Distance from source to neighbour
      // distance_u = distance to current node + weight of edge u to
      // neighbour
      const abm::graph::vertex_t uidx = vertices_.at(u);
      const abm::graph::vertex_t nidx = vertices_.at(neighbour);

      const abm::graph::weight_t distance_u = distances.at(uidx) + weight;
      // If there is shorted path to neighbour vertex through u.
      if (distances.at(nidx) > distance_u) {
        parent[nidx] = u;
        // Update distance of the vertex
        distances.at(nidx) = distance_u;
        priority_queue.push(std::make_pair(distance_u, neighbour));
      }
    }
  }

  path.emplace_back(destination);
  // Iterate until source has been reached
  while (destination != source && destination != -1) {
    destination = parent.at(vertices_.at(destination));
    if (destination != source && destination != -1)
      path.emplace_back(destination);
  }
  path.emplace_back(source);
  // Reverse to arrange path from source to destination
  std::reverse(std::begin(path), std::end(path));

  return path;
}

// Dijktra shortest paths from src to a vertex return vertices
std::vector<std::array<abm::graph::vertex_t, 2>> abm::Graph::dijkstra_vertices(
    abm::graph::vertex_t source, abm::graph::vertex_t destination) {

  const auto path = this->dijkstra(source, destination);

  std::vector<std::array<abm::graph::vertex_t, 2>> route_vertices;
  if (path.size() > 0) {
    for (auto itr = path.begin(); itr != path.end() - 1; ++itr) {
      auto nitr = itr + 1;
      if (itr != path.end()) {
        std::array<abm::graph::vertex_t, 2> edges = {
            static_cast<abm::graph::vertex_t>(*itr),
            static_cast<abm::graph::vertex_t>(*nitr)};
        route_vertices.emplace_back(edges);
      }
    }
  }
  return route_vertices;
}

// Dijktra shortest paths from src to a vertex return vertices
std::vector<abm::graph::vertex_t> abm::Graph::dijkstra_vertices_ual(
    abm::graph::vertex_t source, abm::graph::vertex_t destination) {

  const auto path = this->dijkstra(source, destination);

  std::vector<abm::graph::vertex_t> route_vertices;
  if (path.size() > 0) {
    for (auto itr = path.begin(); itr != path.end(); ++itr) {
      if (itr != path.end())
        route_vertices.emplace_back(static_cast<abm::graph::vertex_t>(*itr));
    }
    route_vertices.emplace_back(-1);
  }
  return route_vertices;
}

// Dijktra shortest paths from src to a vertex return edges
std::vector<abm::graph::vertex_t> abm::Graph::dijkstra_edges(
    abm::graph::vertex_t source, abm::graph::vertex_t destination) {

  const auto path = this->dijkstra(source, destination);
  //printf("path size = %d\n", path.size());

  std::vector<abm::graph::vertex_t> route_edges;
  if (path.size() > 0) {
    // Reverse to arrange from source to destination
    for (auto itr = path.begin(); itr != path.end() - 1; ++itr) {
      auto nitr = itr + 1;
      if (itr != path.end()) {
        auto map_itr = edge_ids_[*itr].find(*nitr);
        if (map_itr != edge_ids_[*itr].end())
          route_edges.emplace_back((*map_itr).second);
      }
    }
  }
  route_edges.emplace_back(-1);
  return route_edges;
}

// Determine cost of path
abm::graph::weight_t abm::Graph::path_cost(
    const std::vector<std::array<abm::graph::vertex_t, 2>>& path) {
  abm::graph::weight_t cost = 0.;
  for (const auto& vertices : path)
    cost += (edges_.at(std::make_tuple(vertices[0], vertices[1])))->second.length;
  return cost;
}

// Determine cost of path
abm::graph::weight_t abm::Graph::path_cost(
    const std::vector<abm::graph::vertex_t>& path) {
  abm::graph::weight_t cost = 0.;
  for (const auto& edge : path) cost += edge_costs_.at(edge);
  return cost;
}
