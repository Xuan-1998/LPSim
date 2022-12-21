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

#include "cudaTrafficPersonPath.h"

#define WIDTH_WATER 4000.0f
#define ROUTE_DEBUG 0

namespace LC {

CUDATrafficPersonPath::CUDATrafficPersonPath() {
}//
CUDATrafficPersonPath::~CUDATrafficPersonPath() {
}//


/////////////////////////////
template <typename UniquePairAssociativeContainer>
class default_assignment_associative_property_map
  : public boost::put_get_helper <
    typename UniquePairAssociativeContainer::value_type::second_type &,
    default_assignment_associative_property_map<UniquePairAssociativeContainer> > {
  typedef UniquePairAssociativeContainer C;
 public:
  typedef typename C::key_type key_type;
  typedef typename C::value_type::second_type value_type;
  typedef value_type &reference;
  typedef boost::lvalue_property_map_tag category;
  default_assignment_associative_property_map() : m_c(0) { }
  default_assignment_associative_property_map(C &c) : m_c(&c) { }

  reference operator[](const key_type &k) const {
    if (m_c->find(k) == m_c->end()) {
      (*m_c)[k] = FLT_MAX;
    }

    return (*m_c)[k];
  }
 private:
  C *m_c;
};

class dijkstra_finish : public std::exception {
};

class TargetedVisitor : public boost::default_dijkstra_visitor {
 public:
  TargetedVisitor(int v_) : v(v_) {}
  template <typename Vertex, typename Graph>
  void finish_vertex(Vertex u, Graph &g) {
    if (u == v) {
      if (ROUTE_DEBUG == true) {
        std::cout << "Found it" << u << "\n";
      }

      throw dijkstra_finish();
    }
  }
 private:
  int v; // target vertex
};

typedef std::map<int, float> DistanceMap;
typedef default_assignment_associative_property_map<DistanceMap>
DistanceMapInternal;

typedef std::map<LC::RoadGraph::roadGraphVertexDesc_BI, LC::RoadGraph::roadGraphVertexDesc_BI>
PredecessorMap;
typedef default_assignment_associative_property_map<PredecessorMap>
PredecessorMapInternal;

void CUDATrafficPersonPath::calculateOneRoute(
  LC::RoadGraph::roadBGLGraph_BI &roadGraph,
  CUDATrafficPerson &person,
  std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum,
  std::vector<ushort> &nextEdgeM
) {

  LC::RoadGraph::roadGraphVertexDesc_BI srcvertex = person.init_intersection;
  LC::RoadGraph::roadGraphVertexDesc_BI tgtvertex = person.end_intersection;

  if (tgtvertex == srcvertex) { //source same than target (we have arrived)
    //person.nextPathEdge=-1;
    return;
  }

  int n = boost::num_vertices(roadGraph);

  if (ROUTE_DEBUG == true) {
    printf("1. Source %d Target %d Total %d\n", srcvertex, tgtvertex, n);
  }

  if (srcvertex < 0 || srcvertex >= n) {
    return;
  }

  if (tgtvertex < 0 || tgtvertex >= n) {
    return;
  }

  if (srcvertex == tgtvertex) {
    return;
  }

  PredecessorMap mapPredecessor;
  boost::associative_property_map<PredecessorMap> pmPredecessor(mapPredecessor);

  TargetedVisitor vis(tgtvertex);

  {
    try {
      DistanceMap mapDistance;
      DistanceMapInternal pmDistance(mapDistance);
      mapDistance[srcvertex] = 0.0;

      typedef LC::RoadGraph::roadGraphVertexDesc_BI VertexDescriptor;
      typedef std::map<VertexDescriptor, size_t>            VertexIndexMap;
      typedef std::map<VertexDescriptor, boost::default_color_type> ColorMap;
      VertexIndexMap mapVertexIndex;
      ColorMap	   mapColor;
      boost::associative_property_map<VertexIndexMap> pmVertexIndex(mapVertexIndex);
      boost::associative_property_map<ColorMap> pmColor(mapColor);

      boost::dijkstra_shortest_paths_no_init(roadGraph,
                                             srcvertex,
                                             pmPredecessor,
                                             pmDistance,
                                             boost::get(&LC::RoadGraphEdge::edge_weight, roadGraph),
                                             pmVertexIndex,
                                             std::less<float>(),
                                             boost::closed_plus<float>(),
                                             float(),
                                             vis,
                                             pmColor);

    } catch (const dijkstra_finish &) {}

  }

  //std::cout << mapPredecessor.size() << "\n";
  if (mapPredecessor.size() <= 0) {
    //person.nextPathEdge=-1;
    return;
  }


  // create path
  ushort vertex = tgtvertex;
  std::vector<ushort> path;

  while (vertex != srcvertex) {
    if (ROUTE_DEBUG == true) {
      printf("%d<- ", vertex);
    }

    path.push_back(vertex);
    vertex = mapPredecessor[vertex];
  }

  path.push_back(srcvertex);

  if (ROUTE_DEBUG == true) {
    printf("%d\n", srcvertex);
  }

  // put path lanes in nextEdgeM
  //person.nextPathEdge=nextEdgeM.size();//the first path edge will be in that possition in nextEdge
  for (int pa = path.size() - 1; pa > 0; pa--) {
    std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> edge_pair = boost::edge(
          path[pa], path[pa - 1], roadGraph);

    if (edge_pair.second == false) {
      if ((path[pa] != 0) && (path[pa - 1] != 0)) {
        printf("N Error: Edge %u %u should exists\n", path[pa], path[pa - 1]);
      }

      //person.nextPathEdge=0xFFFF;
      break;
    } else {
      //std::map<RoadGraph::roadGraphEdgeDesc_BI,uint> edgeDescToLaneMapNum
      if (edgeDescToLaneMapNum.find(edge_pair.first) == edgeDescToLaneMapNum.end()) {
        printf("****Unknown edge\n");
        break;
      }

      ushort lane = edgeDescToLaneMapNum[edge_pair.first];
      nextEdgeM.push_back(lane);
    }

  }

  nextEdgeM.push_back(0xFFFF);//ensure the perso
}

////////////////

void CUDATrafficPersonPath::generateRoutes(
  LC::RoadGraph::roadBGLGraph_BI &roadGraph,
  std::vector<CUDATrafficPerson> &trafficPersonVec,
  std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum,
  std::vector<ushort> &nextEdgeM) {
  if (trafficPersonVec.size() <= 0) {
    printf("ERROR generateRoutes: trafficPersonVec.size()<0");
    return;
  }

  printf(">> generatePathRoutes\n");
  QTime timer;
  timer.start();
  RoadGraph::roadGraphEdgeIter_BI ei, eiEnd;
  //QVector3D p0;
  //QVector3D p1;
  //float r, g, b;

  // 1. Update weight edges

  int numEdges = 0;

  for (boost::tie(ei, eiEnd) = boost::edges(roadGraph);
       ei != eiEnd; ++ei) {
    numEdges++;
    //p0 = roadGraph[boost::source(*ei, roadGraph)].pt;
    //p1 = roadGraph[boost::target(*ei, roadGraph)].pt;
    roadGraph[*ei].edge_weight = roadGraph[*ei].edgeLength /
                                 roadGraph[*ei].maxSpeedMperSec;//(p0-p1).length();
  }

  //printf("Updated length of edges %d\n",numEdges);

  //2. Generate route for each person
  RoadGraph::roadGraphVertexIter_BI vi, viEnd;

  for (int p = 0; p < trafficPersonVec.size(); p++) {
    calculateOneRoute(roadGraph, trafficPersonVec[p], edgeDescToLaneMapNum,
                      nextEdgeM);
  }

  printf("<< generateRoutePaths in %d ms\n", timer.elapsed());
}//
}


