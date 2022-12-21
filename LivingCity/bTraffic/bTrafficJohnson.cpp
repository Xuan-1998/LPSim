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

#include "bTrafficJohnson.h"

#include <boost/graph/exterior_property.hpp>


#define ROUTE_DEBUG 0

namespace LC {

/*
/////////////////////////////
template <typename UniquePairAssociativeContainer>
class default_assignment_associative_property_map
	: public boost::put_get_helper<
	typename UniquePairAssociativeContainer::value_type::second_type&,
	default_assignment_associative_property_map<UniquePairAssociativeContainer> >
{
	typedef UniquePairAssociativeContainer C;
public:
	typedef typename C::key_type key_type;
	typedef typename C::value_type::second_type value_type;
	typedef value_type& reference;
	typedef boost::lvalue_property_map_tag category;
	default_assignment_associative_property_map() : m_c(0) { }
	default_assignment_associative_property_map(C& c) : m_c(&c) { }

	reference operator[](const key_type& k) const
	{
		if(m_c->find(k) == m_c->end())
			(*m_c)[k] = FLT_MAX;

		return (*m_c)[k];
	}
private:
	C* m_c;
};

class dijkstra_finish : public std::exception {
};

class TargetedVisitor : public boost::default_dijkstra_visitor {
public:
	TargetedVisitor(int v_) : v(v_) {}
	template <typename Vertex, typename Graph>
	void finish_vertex(Vertex u, Graph& g) {
		if(u == v) {
			if(ROUTE_DEBUG==true)std::cout << "Found it" << u << "\n";
			throw dijkstra_finish();
		}
	}
private:
	int v; // target vertex
};

typedef std::map<int, float> DistanceMap;
typedef default_assignment_associative_property_map<DistanceMap> DistanceMapInternal;

typedef std::map<LC::RoadGraph::roadGraphVertexDesc_BI, LC::RoadGraph::roadGraphVertexDesc_BI>  PredecessorMap;
typedef default_assignment_associative_property_map<PredecessorMap> PredecessorMapInternal;

void BTrafficDijstra::calculateOneRoute(
		LC::RoadGraph::roadBGLGraph_BI& roadGraph,
		int p,
		BTrafficPeople& people,
		std::map<RoadGraph::roadGraphEdgeDesc_BI,uint>& edgeDescToLaneMapNum,
		std::vector<ushort>& nextEdgeM
	){

	LC::RoadGraph::roadGraphVertexDesc_BI srcvertex=people.init_intersection[p];
	LC::RoadGraph::roadGraphVertexDesc_BI tgtvertex=people.end_intersection[p];

	if(tgtvertex==srcvertex){//source same than target (we have arrived)
		//person.nextPathEdge=-1;
		return;
	}

	int n = boost::num_vertices(roadGraph);
	if(ROUTE_DEBUG==true)printf("1. Source %d Target %d Total %d\n",srcvertex,tgtvertex,n);

	if( srcvertex < 0 || srcvertex >= n) return;
	if( tgtvertex < 0 || tgtvertex >= n) return;
	if(srcvertex == tgtvertex) return;

	PredecessorMap mapPredecessor;
	boost::associative_property_map<PredecessorMap> pmPredecessor (mapPredecessor);

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
			boost::associative_property_map<VertexIndexMap> pmVertexIndex (mapVertexIndex);
			boost::associative_property_map<ColorMap> pmColor (mapColor);

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

		} catch(const dijkstra_finish&) {}

	}
	//std::cout << mapPredecessor.size() << "\n";
	if(mapPredecessor.size()<=0){
		//person.nextPathEdge=-1;
		return;
	}


	// create path
	ushort vertex =tgtvertex;
	std::vector<ushort> path;
	while(vertex != srcvertex) {
		if(ROUTE_DEBUG==true)printf("%d<- ",vertex);
		path.push_back(vertex);
		vertex=mapPredecessor[vertex];
	}
	path.push_back(srcvertex);
	if(ROUTE_DEBUG==true)printf("%d\n",srcvertex);
	// put path lanes in nextEdgeM
	people.indToEdge[p]=nextEdgeM.size();//the first path edge will be in that possition in nextEdge
	for(int pa=path.size()-1;pa>0;pa--){
		std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> edge_pair =boost::edge(path[pa],path[pa-1],roadGraph);
		if(edge_pair.second==false){
			if((path[pa]!=0)&&(path[pa-1]!=0)){
				printf("N Error: Edge %u %u should exists\n",path[pa],path[pa-1]);
			}
			//person.nextPathEdge=0xFFFF;
			break;
		}else{
			//std::map<RoadGraph::roadGraphEdgeDesc_BI,uint> edgeDescToLaneMapNum
			if(edgeDescToLaneMapNum.find(edge_pair.first)==edgeDescToLaneMapNum.end()){
				printf("****Unknown edge\n");
				break;
			}
			ushort lane=edgeDescToLaneMapNum[edge_pair.first];
			nextEdgeM.push_back(lane);
		}

	}
	nextEdgeM.push_back(0xFFFF);//ensure the perso
}
*/
////////////////
/////////////////////////////
using namespace boost;

typedef exterior_vertex_property<RoadGraph::roadBGLGraph_BI, float>
DistanceProperty;
typedef DistanceProperty::matrix_type DistanceMatrix;
typedef DistanceProperty::matrix_map_type DistanceMatrixMap;
//typedef constant_property_map<RoadGraph::roadGraphEdgeDesc_BI, float> WeightMap;

void BTrafficJohnson::generateRoutes(
  LC::RoadGraph::roadBGLGraph_BI &roadGraph,
  BTrafficPeople &people,
  std::map<RoadGraph::roadGraphEdgeDesc_BI, uint> &edgeDescToLaneMapNum,
  std::vector<ushort> &nextEdgeM) {
  if (people.numPeople <= 0) {
    printf("ERROR generateRoutes: people size<0");
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
  property_map<RoadGraph::roadBGLGraph_BI, float RoadGraphEdge::*>::type
  weight_pmap = boost::get(&RoadGraphEdge::edge_weight, roadGraph);

  for (boost::tie(ei, eiEnd) = boost::edges(roadGraph);
       ei != eiEnd; ++ei) {
    numEdges++;
    roadGraph[*ei].edge_weight = roadGraph[*ei].edgeLength /
                                 roadGraph[*ei].maxSpeedMperSec;//(p0-p1).length();
    weight_pmap[*ei] = roadGraph[*ei].edgeLength /
                       roadGraph[*ei].maxSpeedMperSec;//(p0-p1).length();
  }

  //printf("Updated length of edges %d\n",numEdges);

  //2. Generate route for each person

  /*for (int p = 0; p<people.numPeople; p++){
        calculateOneRoute(roadGraph, p, people, edgeDescToLaneMapNum, nextEdgeM);
        }*/
  //std::vector<float> D;
  int numVertex = boost::num_vertices(roadGraph);


  //vertex
  typedef LC::RoadGraph::roadGraphVertexDesc_BI VertexDescriptor;
  typedef std::map<VertexDescriptor, size_t> VertexIndexMap;
  VertexIndexMap mapVertexIndex;
  boost::associative_property_map<VertexIndexMap> pmVertexIndex(mapVertexIndex);

  DistanceMatrix distances(numVertex);
  DistanceMatrixMap dm(distances, roadGraph);
  //WeightMap wm(1);

  std::vector < float >d(numVertex, (std::numeric_limits < float >::max)());
  ////////////////////////
  // CALL JOHNSON
  //boost::johnson_all_pairs_shortest_paths(roadGraph, dm,
  //	weight_map(boost::get(&RoadGraphEdge::edge_weight, roadGraph)));
  //.distance_map(make_iterator_property_map(distances.begin(),
  //boost::get(vertex_index, roadGraph))));
  //boost::get(&LC::RoadGraphEdge::edge_weight, roadGraph));
  // , weight_map(wm));
  boost::johnson_all_pairs_shortest_paths(roadGraph, dm, weight_map(weight_pmap));
  // check maxDist
  printf("numVertex %d\n", numVertex);
  //return;
  float maxDist = -1.0f;

  for (int vN = 0; vN < numVertex; vN++) {
    for (int vN2 = 0; vN2 < numVertex; vN2++) {
      //printf("%.1f ", dm[vN][vN2]);
      maxDist = maxDist < dm[vN][vN2] ? dm[vN][vN2] : maxDist;
    }

    //printf("\n");
  }

  printf("maxDist %f\n", maxDist);
  ////////////////////////
  // Create routes
  uint noAccesible = 0;
  uint sameSrcDst = 0;
  QTime timer2;
  timer2.start();

  for (int p = 0; p < people.numPeople; p++) {
    if (people.numPeople > 200) {
      if ((p % (people.numPeople / 20)) == 0) {
        printf("Route %d of %d (%2.0f%%)\n", p, people.numPeople,
               (100.0f * p) / people.numPeople);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////
    //calculateOneRouteJohnson(roadGraph, p, people, edgeDescToLaneMapNum,dm);

    //set index to read
    people.indTo1stEdge[p] =
      people.nextEdge.size();//the first path edge will be in that possition in nextEdge

    LC::RoadGraph::roadGraphVertexDesc_BI srcvertex = people.init_intersection[p];
    LC::RoadGraph::roadGraphVertexDesc_BI tgtvertex = people.end_intersection[p];

    // check whether source same than target (we have arrived)
    if (tgtvertex == srcvertex) {
      people.nextEdge.push_back(0xFFFF);
      people.edgeExitOut.push_back(0xFF);
      people.edgeExitIn.push_back(0xFF);
      //printf("same source destination\n");
      sameSrcDst++;
      continue;
    }

    // check if accesible
    if (dm[srcvertex][tgtvertex] == (std::numeric_limits < float >::max)()) {
      people.nextEdge.push_back(0xFFFF);
      people.edgeExitOut.push_back(0xFF);
      people.edgeExitIn.push_back(0xFF);
      //printf("no accesible\n");
      noAccesible++;
      continue;
    }

    // find path
    LC::RoadGraph::roadGraphVertexDesc_BI currvertex = srcvertex;//init
    RoadGraph::out_roadGraphEdgeIter_BI Oei, Oei_end;
    LC::RoadGraph::roadGraphVertexDesc_BI srcPosEdgeV, tgtPosEdgeV;

    while (currvertex != tgtvertex) {
      //float currWeight = dm[srcvertex][tgtvertex];
      // check all outedges from currvertex which one continues to shortest path
      //printf("p %d new edge %u\n", p, currvertex);
      bool cont = false;

      for (boost::tie(Oei, Oei_end) = boost::out_edges(currvertex, roadGraph);
           Oei != Oei_end; ++Oei) {
        //if (roadGraph[*Oei].numberOfLanes == 0)
        //	continue;
        srcPosEdgeV = boost::source(*Oei, roadGraph);
        tgtPosEdgeV = boost::target(*Oei, roadGraph);

        //printf("Total %f --> to next %f next to dest %f == to dest %f\n", dm[srcvertex][tgtvertex], dm[currvertex][tgtPosEdgeV], dm[tgtPosEdgeV][tgtvertex], dm[currvertex][tgtvertex]);
        //printf("%f != %f\n", dm[currvertex][tgtPosEdgeV] + dm[tgtPosEdgeV][tgtvertex] , dm[currvertex][tgtvertex]);
        //if (dm[currvertex][tgtPosEdgeV] + dm[tgtPosEdgeV][tgtvertex] == dm[currvertex][tgtvertex]){//found link
        if (std::abs<float>(dm[currvertex][tgtPosEdgeV] + dm[tgtPosEdgeV][tgtvertex] -
                            dm[currvertex][tgtvertex]) < 0.1f) { //found link
          std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> edge_pair = boost::edge(
                srcPosEdgeV, tgtPosEdgeV, roadGraph);

          if (edge_pair.second == false) {
            printf("****edge not found\n");//this should not happen
            currvertex = tgtPosEdgeV;
            break;//break for
          } else {
            if (edgeDescToLaneMapNum.find(edge_pair.first) == edgeDescToLaneMapNum.end()) {
              printf("****Unknown edge\n");//this should not happen
              currvertex = tgtvertex;//end loop
              break;//break for
            }

            ushort edge = edgeDescToLaneMapNum[edge_pair.first];

            if (people.nextEdge.size() > 0 &&
                people.nextEdge.back() !=
                0xFFFF) { //skip first one (so Exit of next edge is align with current edge)
              people.edgeExitOut.push_back(roadGraph[edge_pair.first].outNum);
            }

            people.edgeExitIn.push_back(roadGraph[edge_pair.first].inNum);
            people.nextEdge.push_back(edge);

            currvertex = tgtPosEdgeV;
            //printf("found edge %u\n", edge);
            cont = true;
            break;//break for
          }
        }
      }

      if (cont == true) {
        continue;
      }

      // not foudn edge
      printf("****none edge works\n");//this should not happen
      //exit(0);//!!! REMOVE
      break;
    }//while find tgt

    if (people.nextEdge.back() !=
        0xFFFF) {//add first one (bc Exit is align with current edge)
      people.edgeExitOut.push_back(0xFF);//add at the end to balance first edge
    }

    people.nextEdge.push_back(0xFFFF);//ensure the perso
    people.edgeExitOut.push_back(0xFF);//
    people.edgeExitIn.push_back(0xFF);//
    ////////////////////////////////////////////////////////////////////////////////////////////
  }

  printf("<< generateRoutePathsJohnson: individual routes time %d ms --> numPeople %d (No Acc %d sameSrcDst %d)\n",
         timer2.elapsed(), people.numPeople, noAccesible, sameSrcDst);
  printf("<< generateRoutePathsJohnson--> edges %u exit out %u in %u (should be same) in %d ms\n",
         people.nextEdge.size(), people.edgeExitOut.size(), people.edgeExitIn.size(),
         timer.elapsed());
  /*
  //////////////////////////////////////
  using namespace boost;
  typedef adjacency_list<vecS, vecS, directedS, no_property,
        property< edge_weight_t, int, property< edge_weight2_t, int > > > Graph;
  const int V = 6;
  typedef std::pair < int, int >Edge;
  Edge edge_array[] =
  { Edge(0, 1), Edge(0, 2), Edge(0, 3), Edge(0, 4), Edge(0, 5),
  Edge(1, 2), Edge(1, 5), Edge(1, 3), Edge(2, 4), Edge(2, 5),
  Edge(3, 2), Edge(4, 3), Edge(4, 1), Edge(5, 4)
  };
  const std::size_t E = sizeof(edge_array) / sizeof(Edge);
  #if defined(BOOST_MSVC) && BOOST_MSVC <= 1300
  // VC++ can't handle the iterator constructor
  Graph g(V);
  for (std::size_t j = 0; j < E; ++j)
        add_edge(edge_array[j].first, edge_array[j].second, g);
  #else
  Graph g(edge_array, edge_array + E, V);
  #endif

  property_map < Graph, edge_weight_t >::type w = get(edge_weight, g);
  int weights[] = { 0, 0, 0, 0, 0, 3, -4, 8, 1, 7, 4, -5, 2, 6 };
  int *wp = weights;

  graph_traits < Graph >::edge_iterator e, e_end;
  for (boost::tie(e, e_end) = edges(g); e != e_end; ++e)
        w[*e] = *wp++;

  std::vector < int >d(V, (std::numeric_limits < int >::max)());
  int D[V][V];
  johnson_all_pairs_shortest_paths(g, D, distance_map(&d[0]));

  std::cout << "       ";
  for (int k = 0; k < V; ++k)
        std::cout << std::setw(5) << k;
  std::cout << std::endl;
  for (int i = 0; i < V; ++i) {
        std::cout << std::setw(3) << i << " -> ";
        for (int j = 0; j < V; ++j) {
                if (D[i][j] == (std::numeric_limits<int>::max)())
                        std::cout << std::setw(5) << "inf";
                else
                        std::cout << std::setw(5) << D[i][j];
        }
        std::cout << std::endl;
  }

  std::ofstream fout("figs/johnson-eg.dot");
  fout << "digraph A {\n"
        << "  rankdir=LR\n"
        << "size=\"5,3\"\n"
        << "ratio=\"fill\"\n"
        << "edge[style=\"bold\"]\n" << "node[shape=\"circle\"]\n";

  graph_traits < Graph >::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
        fout << source(*ei, g) << " -> " << target(*ei, g)
        << "[label=" << get(edge_weight, g)[*ei] << "]\n";

  fout << "}\n";
  return 0;*/


  //////////////////////////////////////
  //printf("<< generateRoutePaths in %d ms\n",timer.elapsed());
}//
}


