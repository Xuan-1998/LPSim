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

/************************************************************************************************
*
*		MTC Project - Geometry Project - RoadGraph Class
*
*
*		@desc Class containing the road graph information
*		@author cvanegas
*
************************************************************************************************/
#ifndef GEOMETRY_ROADGRAPH_H
#define GEOMETRY_ROADGRAPH_H

#include "misctools/misctools.h"

#include <vector>
#include <QSettings>
#ifndef Q_MOC_RUN
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/copy.hpp>
#endif

#include "roadGraphEdge.h"
#include "roadGraphVertex.h"

#include <QTemporaryFile>



namespace LC {

/**
* RoadGraph.
**/
class RoadGraph {

 public:
  /**
  * Constructor.
  **/
  RoadGraph();

  /**
  * Destructor.
  **/
  ~RoadGraph();

  /**
  * Initialize.
  **/
  void init();

  /**
  * Clear
  **/
  void clear();

  /**
  * Copy constructor.
  **/
  RoadGraph(const RoadGraph &ref) {
    //myRoadGraph = ref.myRoadGraph;
    //myRoadGraph_BI= ref.myRoadGraph_BI;
    boost::copy_graph(ref.myRoadGraph, myRoadGraph);
    boost::copy_graph(ref.myRoadGraph_BI, myRoadGraph_BI);

    updateDL = ref.updateDL;
    dlIdx = ref.dlIdx;
  }

  /**
  * Assignment operator.
  **/

  inline RoadGraph &operator=(const RoadGraph &ref) {
    //myRoadGraph = ref.myRoadGraph;
    //myRoadGraph_BI= ref.myRoadGraph_BI;
    boost::copy_graph(ref.myRoadGraph, myRoadGraph);
    boost::copy_graph(ref.myRoadGraph_BI, myRoadGraph_BI);

    updateDL = ref.updateDL;
    dlIdx = ref.dlIdx;
    return (*this);
  }

  inline void resetRoadGraph(void) {
    myRoadGraph.clear();
  }

  /**
  * Export to OSM
  **/
  void writeRoadNetworkToOSM(QTextStream &osmStream);


  /**
  * Adjacency list for road graph
  **/
  typedef boost::adjacency_list
  <boost::vecS, boost::vecS, boost::undirectedS, RoadGraphVertex, RoadGraphEdge>
  roadBGLGraph;//bidirectionalS//undirectedS//directedS

  typedef boost::graph_traits<roadBGLGraph>::vertex_descriptor
  roadGraphVertexDesc;
  typedef boost::graph_traits<roadBGLGraph>::edge_descriptor roadGraphEdgeDesc;

  typedef boost::graph_traits<roadBGLGraph>::vertex_iterator roadGraphVertexIter;
  typedef boost::graph_traits<roadBGLGraph>::edge_iterator roadGraphEdgeIter;
  typedef boost::graph_traits<roadBGLGraph>::adjacency_iterator roadGraphAdjIter;

  typedef boost::graph_traits<roadBGLGraph>::out_edge_iterator
  out_roadGraphEdgeIter;


  typedef boost::adjacency_list
  <boost::vecS, boost::vecS, boost::bidirectionalS, RoadGraphVertex, RoadGraphEdge>
  roadBGLGraph_BI;//bidirectionalS directedS
  typedef boost::graph_traits<roadBGLGraph_BI>::vertex_descriptor
  roadGraphVertexDesc_BI;
  typedef boost::graph_traits<roadBGLGraph_BI>::edge_descriptor
  roadGraphEdgeDesc_BI;

  typedef boost::graph_traits<roadBGLGraph_BI>::vertex_iterator
  roadGraphVertexIter_BI;
  typedef boost::graph_traits<roadBGLGraph_BI>::edge_iterator
  roadGraphEdgeIter_BI;
  typedef boost::graph_traits<roadBGLGraph_BI>::out_edge_iterator
  out_roadGraphEdgeIter_BI;
  typedef boost::graph_traits<roadBGLGraph_BI>::in_edge_iterator
  in_roadGraphEdgeIter_BI;
  typedef boost::graph_traits<roadBGLGraph_BI>::adjacency_iterator
  roadGraphAdjIter_BI;



  roadBGLGraph myRoadGraph;
  roadBGLGraph_BI myRoadGraph_BI;

  /**
  * Must be set to true every time the display list is to be regenerated
  **/
  bool updateDL;
  GLuint dlIdx;
  bool initialized;

  /**
  *
  **/
  bool findClosestGraphVertex(QVector3D &clickedPoint,
                              float maxDistance,
                              bool getSeedsOnly,
                              RoadGraph::roadGraphVertexIter &result);

  /**
  * fills the attribute inNum outNum to define complex intersections
  **/
  void fillInOutNumForEdges();

 private:

};

}

#endif
