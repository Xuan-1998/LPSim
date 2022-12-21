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

#include "cudaTrafficPersonShortestPath.h"

#include <boost/graph/dijkstra_shortest_paths.hpp>

#define ROUTE_DEBUG 1
#define PATH_DEBUG 1

namespace LC {

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

typedef std::map<int, float> DistanceMap;
typedef default_assignment_associative_property_map<DistanceMap>
DistanceMapInternal;

typedef std::map<LC::RoadGraph::roadGraphVertexDesc_BI, LC::RoadGraph::roadGraphVertexDesc_BI>
PredecessorMap;
typedef default_assignment_associative_property_map<PredecessorMap>
PredecessorMapInternal;

void CUDATrafficPersonShortestPath::calculateSeveralPeopleRoute(
  LC::RoadGraph::roadBGLGraph_BI &roadGraph,
  std::vector<CUDATrafficPerson> &trafficPersonVec,
  std::vector<uint> &peopleStartInInter,
  std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum) {

  LC::RoadGraph::roadGraphVertexDesc_BI srcvertex =
    trafficPersonVec[peopleStartInInter[0]].init_intersection;
  //check not already at the destination
  int n = boost::num_vertices(roadGraph);

  for (int p = 0; p < peopleStartInInter.size(); p++) {
    LC::RoadGraph::roadGraphVertexDesc_BI tgtvertex =
      trafficPersonVec[peopleStartInInter[p]].end_intersection;

    if (tgtvertex == srcvertex || srcvertex < 0 || srcvertex >= n ||
        tgtvertex < 0 || tgtvertex >= n) { //source same than target (we have arrived)
      trafficPersonVec[peopleStartInInter[p]].personPath[0] = -1;
      peopleStartInInter.erase(peopleStartInInter.begin() + p);
      p--;
    }
  }

  if (peopleStartInInter.size() <= 0) { //all processed
    return;
  }

  //printf("run dijktra\n");
  // run dijkstra
  PredecessorMap mapPredecessor;
  boost::associative_property_map<PredecessorMap> pmPredecessor(mapPredecessor);

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
                                         boost::default_dijkstra_visitor(),
                                         pmColor);


  // if not map predecessor all failed
  if (mapPredecessor.size() <= 0) {
    for (int p = 0; p < peopleStartInInter.size(); p++) {
      trafficPersonVec[peopleStartInInter[p]].personPath[0] = -1;
    }

    return;
  }


  // for each person find the path using the map predecersor
  for (int p = 0; p < peopleStartInInter.size(); p++) {
    int currIndex = 0;
    // create path
    uint vertex = trafficPersonVec[peopleStartInInter[p]].end_intersection;
    //printf("s %d d %d\n",srcvertex,vertex);
    std::vector<uint> path;

    while (vertex != srcvertex) {
      if (ROUTE_DEBUG == true) {
        printf("%d<- ", vertex);
      }

      path.push_back(vertex);
      vertex = mapPredecessor[vertex];

      if (path[path.size() - 1] == vertex) {
        break;  //twice the same vertex
      }
    }

    path.push_back(srcvertex);

    if (ROUTE_DEBUG == true) {
      printf("%d\n", srcvertex);
    }

    //printf("path %d\n",path.size());
    // put path lanes in nextEdgeM
    //trafficPersonVec[peopleStartInInter[p]].nextPathEdge=nextEdgeM.size();//the first path edge will be in that possition in nextEdge
    for (int pa = path.size() - 1; pa > 0; pa--) {
      std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> edge_pair = boost::edge(
            path[pa], path[pa - 1], roadGraph);

      if (edge_pair.second == false) {
        if (path[pa] != 0 && path[pa - 1] != 0) {
          printf("N Error: Edge %d %d should exists\n", path[pa], path[pa - 1]);
        }

        //trafficPersonVec[peopleStartInInter[p]].nextPathEdge=0xFFFF;
        //trafficPersonVec[peopleStartInInter[p]].personPath[currIndex]=-1;
        trafficPersonVec[peopleStartInInter[p]].personPath[0] = -1;
        break;
      } else {
        //std::map<RoadGraph::roadGraphEdgeDesc_BI,uint> edgeDescToLaneMapNum
        if (edgeDescToLaneMapNum.find(edge_pair.first) == edgeDescToLaneMapNum.end()) {
          if (roadGraph[edge_pair.first].numberOfLanes > 0) { //not found and >0 --> error
            printf("****Unknown edge\n");
          }

          break;
        }

        uint lane = edgeDescToLaneMapNum[edge_pair.first];
        trafficPersonVec[peopleStartInInter[p]].personPath[currIndex] = lane;
        currIndex++;

        if (currIndex >= 140) { //change CUDATrafficPerson::personPath
          printf("Error: More than 140 edges %d\n", path.size());
          currIndex--;
          break;
        }

        //nextEdgeM.push_back(lane);
      }

    }
    trafficPersonVec[peopleStartInInter[p]].personPath[currIndex] = -1;
  }

  ////////////////////
  // Debug
  const bool kDebugFirstPerson = true;
  if (trafficPersonVec.size()>0 && kDebugFirstPerson) {
    int currIndex = 0;
    while (true) {
      uint laneMap = trafficPersonVec[0].personPath[currIndex++
      ];
      if (laneMap != -1) {
        printf("-> %u ", laneMap);
      } else {
        break;
      }
    }
    printf("\n");
  }

}//

////////////////

void CUDATrafficPersonShortestPath::generateRoutesMulti(
  LC::RoadGraph::roadBGLGraph_BI &roadGraph,
  std::vector<CUDATrafficPerson> &trafficPersonVec,
  std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum,
  int weigthMode,
  float sample) {
  // Just using the lane length and number of edges
  if (weigthMode == 0 || weigthMode == 1) {
    if (PATH_DEBUG) {
      printf(">> generateRoutesMulti\n");
    }

    QTime timer;
    timer.start();
    RoadGraph::roadGraphEdgeIter_BI ei, eiEnd;

    // 1. Update weight edges

    int numEdges = 0;
    float minTravelTime = FLT_MAX;
    float maxTravelTime = -FLT_MAX;
    float minLength = FLT_MAX;
    float maxLength = -FLT_MAX;
    float minSpeed = FLT_MAX;
    float maxSpeed = -FLT_MAX;

    for (boost::tie(ei, eiEnd) = boost::edges(roadGraph);
      ei != eiEnd; ++ei) {
      numEdges++;

      if (roadGraph[*ei].numberOfLanes > 0) {
        // Use the original info, length and max speed
        float speed;
        if (weigthMode == 0 || roadGraph[*ei].averageSpeed.size() <= 0) {
          speed = roadGraph[*ei].maxSpeedMperSec;// *sqrt(roadGraph[*ei].numberOfLanes));
        } else {
          // Use the avarage speed sampled in a former step
          float avOfAv = 0;
          for (int avOfAvN = 0; avOfAvN < roadGraph[*ei].averageSpeed.size(); avOfAvN++) {
            avOfAv += roadGraph[*ei].averageSpeed[avOfAvN];
          }
          speed = avOfAv / roadGraph[*ei].averageSpeed.size();
        }
        float travelTime = roadGraph[*ei].edgeLength / speed;
        roadGraph[*ei].edge_weight = travelTime;

        minTravelTime = minTravelTime>travelTime ? travelTime : minTravelTime;
        maxTravelTime = maxTravelTime < travelTime ? travelTime : maxTravelTime;

        minLength = minLength > roadGraph[*ei].edgeLength ? roadGraph[*ei].edgeLength : minLength;
        maxLength = maxLength < roadGraph[*ei].edgeLength ? roadGraph[*ei].edgeLength : maxLength;

        minSpeed = minSpeed > speed ? speed : minSpeed;
        maxSpeed = maxSpeed < speed ? speed : maxSpeed;
      } else {
        roadGraph[*ei].edge_weight =
          100000000.0; //FLT_MAX;// if it has not lines, then the weight is inf
      }
    }
    printf("Travel time Min: %f Max: %f\n", minTravelTime, maxTravelTime);
    printf("Length Min: %f Max: %f\n", minLength, maxLength);
    printf("Speed Min: %f Max: %f\n", minSpeed, maxSpeed);

    //2. Generate hash with init intersectios nad people
    QHash<uint, std::vector<uint>> intersectionToPeople;

    for (int p = 0; p < trafficPersonVec.size(); p++) {
      if (sample != 1.0f) {
        if (sample > (((float)qrand()) / RAND_MAX)) {
          continue;
        }
      }

      uint iI = trafficPersonVec[p].init_intersection;
      intersectionToPeople[iI].push_back(p);
    }

    QHash<uint, std::vector<uint>>::const_iterator i =
                                    intersectionToPeople.constBegin();

    while (i != intersectionToPeople.constEnd()) {
      if (i.value().size() <= 0) {
        continue;
      }

      std::vector<uint> peopleStartInInter = i.value();
      //printf("peopleStartInInter.size() %d\n",peopleStartInInter.size());
      calculateSeveralPeopleRoute(roadGraph, trafficPersonVec, peopleStartInInter,
                                  edgeDescToLaneMapNum);//,nextEdgeM);
      ++i;
      printf(" one calculateSeveralPeopleRoute in %d ms\n", timer.elapsed());
    }

    //if(PATH_DEBUG)
    printf("<< generateRoutesMulti in %d ms\n", timer.elapsed());
    return;
  }

  printf("ERROR: Unknow generateRoutesMulti %d\n", weigthMode);
  exit(0);

}//
}


