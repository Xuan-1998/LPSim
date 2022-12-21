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

#include "cudaTrafficRoutes.h"

#define WIDTH_WATER 4000.0f
#define ROUTE_DEBUG 0

namespace LC {

CUDATrafficRoutes::CUDATrafficRoutes() {
}//
CUDATrafficRoutes::~CUDATrafficRoutes() {
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


typedef std::map<int, float> DistanceMap;
typedef default_assignment_associative_property_map<DistanceMap>
DistanceMapInternal;

typedef std::map<LC::RoadGraph::roadGraphVertexDesc_BI, LC::RoadGraph::roadGraphVertexDesc_BI>
PredecessorMap;
typedef default_assignment_associative_property_map<PredecessorMap>
PredecessorMapInternal;

void CUDATrafficRoutes::calculateOneRoute(LC::RoadGraph &roadGraph,
    std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum,
    std::vector<ushort> &shortesPath,
    LC::RoadGraph::roadGraphVertexDesc_BI srcvertex) {

  int n = boost::num_vertices(roadGraph.myRoadGraph_BI);
  int shift = srcvertex * n; //shortesPath has n elements per vertex

  if (ROUTE_DEBUG == true) {
    printf("1. Source %d Total %d\n", srcvertex, n);  //if(ROUTE_DEBUG==true)
  }

  if (srcvertex < 0 || srcvertex >= n) {
    return;
  }

  // 1. calculate shortest distance
  PredecessorMap mapPredecessor;
  boost::associative_property_map<PredecessorMap> pmPredecessor(mapPredecessor);
  {
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

    boost::dijkstra_shortest_paths_no_init(roadGraph.myRoadGraph_BI,
                                           srcvertex,
                                           pmPredecessor,
                                           pmDistance,
                                           boost::get(&LC::RoadGraphEdge::edge_weight, roadGraph.myRoadGraph_BI),
                                           pmVertexIndex,
                                           std::less<float>(),
                                           boost::closed_plus<float>(),
                                           float(),
                                           boost::default_dijkstra_visitor(),
                                           pmColor);
  }

  if (ROUTE_DEBUG == true) {
    std::cout << mapPredecessor.size() << "\n";
  }

  if (mapPredecessor.size() <= 0) {
    return;
  }

  // 2. update shortest path in array
  RoadGraph::roadGraphVertexIter_BI vi, viEnd;

  for (boost::tie(vi, viEnd) = boost::vertices(roadGraph.myRoadGraph_BI);
       vi != viEnd; ++vi) {
    ushort vertex = *vi;

    if (vertex == srcvertex) {
      continue;//we are in destination
    }

    if (shortesPath[vertex + shift] != 0xFFFF) {
      //printf("shortesPath[vertex+shift] %d\n",shortesPath[vertex+shift]);
      continue;//already proceesed
    }

    // 2.1 find the path that it is shortest
    std::vector<ushort> path;

    while (vertex != srcvertex) {
      if (ROUTE_DEBUG == true) {
        printf("%d<- ", vertex);
      }

      path.push_back(vertex);
      vertex = mapPredecessor[vertex];
    }

    if (ROUTE_DEBUG == true) {
      printf("%d\n", srcvertex);
    }

    // 2.2 all those in the path will use the same initial edge
    //int firstVertex=;// from source to first vertex
    std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> edge_pair = boost::edge(
          srcvertex, path.back(), roadGraph.myRoadGraph_BI);

    if (edge_pair.second == false) {
      printf("N Error: Edge %d %d should exists\n", srcvertex, path.back());
      RoadGraph::out_roadGraphEdgeIter_BI ei2, ei_end2;

      for (boost::tie(ei2, ei_end2) = boost::out_edges(srcvertex,
                                      roadGraph.myRoadGraph_BI); ei2 != ei_end2; ++ei2) {
        edge_pair.first = *ei2; //exit by one of the edges
        break;
      }

      /*
      printf("\n  **N Error: Edge %d %d should exists\n",srcvertex,path.back());
      printf("  ***Degree out %d \n",boost::out_degree(srcvertex,roadGraph.myRoadGraph_BI));
      RoadGraph::out_roadGraphEdgeIter_BI ei2, ei_end2;
      for(tie(ei2, ei_end2) = boost::out_edges(srcvertex,roadGraph.myRoadGraph_BI); ei2 != ei_end2; ++ei2){
        LC::RoadGraph::roadGraphVertexDesc_BI t=boost::target(*ei2,roadGraph.myRoadGraph_BI);
        printf("%d-> %d\n",srcvertex,t);
      }
      for(int p=0;p<path.size();p++){
        printf("%d<- ",path[p]);
      }
      printf("%d\n",srcvertex);
      printf("PATH size: %d\n",path.size());*/
    }

    uint firstEdge = edgeDescToLaneMapNum[edge_pair.first];

    for (int nP = 0; nP < path.size(); nP++) {
      shortesPath[path[nP] + shift] =
        firstEdge; //all those in the path will use that first edge
    }
  }
}//

////////////////

void CUDATrafficRoutes::generateRoutes(LC::RoadGraph &roadGraph,
                                       std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum,
                                       std::vector<ushort> &shortesPath) {
  printf(">> generateRoutes\n");

  QTime timer;
  timer.start();
  RoadGraph::roadGraphEdgeIter_BI ei, eiEnd;
  QVector3D p0;
  QVector3D p1;
  float r, g, b;

  // 1. Update weight edges

  int numEdges = 0;

  for (boost::tie(ei, eiEnd) = boost::edges(roadGraph.myRoadGraph_BI);
       ei != eiEnd; ++ei) {
    numEdges++;
    p0 = roadGraph.myRoadGraph_BI[boost::source(*ei,
                                  roadGraph.myRoadGraph_BI)].pt;// !!! CALCULATE ONCE WHEN CREATED
    p1 = roadGraph.myRoadGraph_BI[boost::target(*ei, roadGraph.myRoadGraph_BI)].pt;
    roadGraph.myRoadGraph_BI[*ei].edge_weight = (p0 - p1).length() *
        roadGraph.myRoadGraph_BI[*ei].maxSpeedMperSec;
  }

  //printf("Updated length of edges %d\n",numEdges);

  //2. Generate route for each vertex
  LC::RoadGraph::roadGraphVertexDesc_BI srcvertex, tgtvertex;
  RoadGraph::roadGraphVertexIter_BI vi, viEnd;
  numVertex = boost::num_vertices(roadGraph.myRoadGraph_BI);

  if (numVertex >= 15000) {
    printf("ERROR: Too complex network--> Vertex>=15000 (%d).\n It will need too much memory to be hold\n Think use per vehichle route\n",
           numVertex);
    return;
  }

  //printf("Num Vertex: %d\n",numVertex);
  // RESIZE VECTOR TO PROPER SIZE
  /*if(shortesPath!=0){
        delete []*shortesPath;
  }
  *shortesPath=new(std::nothrow) ushort[numVertex*numVertex];*/
  shortesPath.resize(numVertex * numVertex);
  memset(shortesPath.data(), -1,
         numVertex * numVertex * sizeof(unsigned short)); //init data to FF

  /*for(int i=0;i<numVertex;i++){
                for(int j=0;j<numVertex;j++){
                        printf(" %5d ",shortesPath[j+i*numVertex]);
                }
                printf("\n");
                break;
        }
  printf("----\n");*/
  int numV = 0;

  for (boost::tie(vi, viEnd) = boost::vertices(roadGraph.myRoadGraph_BI);
       vi != viEnd; ++vi) {
    if ((numV) % (numVertex / 10) == 0) {
      printf("Routes: %d of %d\n", numV, numVertex);
    }

    if (ROUTE_DEBUG == true) {
      printf("-->deg %d\n", boost::out_degree(*vi, roadGraph.myRoadGraph_BI));
    }

    calculateOneRoute(roadGraph, edgeDescToLaneMapNum, shortesPath, *vi);

    if (ROUTE_DEBUG == true) {
      for (int j = 0; j < numVertex; j++) {
        printf(" %5d ", (shortesPath)[j + *vi * numVertex]);
      }

      printf("\n");
    }

    numV++;
  }

  //



  printf("<< generateRoutes num vertex %d in %d ms\n", numVertex,
         timer.elapsed());
}//
}


