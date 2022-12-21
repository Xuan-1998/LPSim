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
*		@desc Class to test road graph
*		@author igarciad
************************************************************************************************/

#include "../Geometry/client_geometry.h"
#include "../LC_UrbanMain.h"
#include "../LC_GLWidget3D.h"

#include "bGenerateTest.h"
#include "bPMTrafficPerson.h"
#include "../global.h"


namespace LC {


/*void addEdgeD(int ind1,int ind2,RoadGraph& inRoadGraph,std::vector<RoadGraph::roadGraphVertexDesc>& vertex,std::vector<RoadGraph::roadGraphVertexDesc>& vertex_SIM){
	std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e0_pair;
	std::pair<RoadGraph::roadGraphEdgeDesc, bool> e0_pair_SIMP;
	float length=(inRoadGraph.myRoadGraph_BI[vertex[ind1]].pt-inRoadGraph.myRoadGraph_BI[vertex[ind2]].pt).length();

	e0_pair = boost::add_edge(  vertex[ind1], vertex[ind2], inRoadGraph.myRoadGraph_BI );
	inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes=2;
	inRoadGraph.myRoadGraph_BI[e0_pair.first].edgeLength =length;
	inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeedMperSec =G::global().getFloat("cuda_arterial_edges_speed_ms");

	e0_pair = boost::add_edge(  vertex[ind2], vertex[ind1], inRoadGraph.myRoadGraph_BI );
	inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes=2;
	inRoadGraph.myRoadGraph_BI[e0_pair.first].edgeLength =length;
	inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeedMperSec =G::global().getFloat("cuda_arterial_edges_speed_ms");

	e0_pair_SIMP = boost::add_edge(  vertex_SIM[ind1], vertex_SIM[ind2], inRoadGraph.myRoadGraph );
	inRoadGraph.myRoadGraph[e0_pair_SIMP.first].numberOfLanes=4;


}//*/

void updateMinMax3(QVector3D &newPoint, QVector3D &minBox, QVector3D &maxBox) {
  if (newPoint.x() < minBox.x()) {
    minBox.setX(newPoint.x());
  }

  if (newPoint.y() < minBox.y()) {
    minBox.setY(newPoint.y());
  }

  if (newPoint.x() > maxBox.x()) {
    maxBox.setX(newPoint.x());
  }

  if (newPoint.y() > maxBox.y()) {
    maxBox.setY(newPoint.y());
  }
}//

struct Demand {
  uint numP;
  uint start;
  uint end;
  float startTime;
  float gapBetweenCars;
  Demand(uint n, uint s, uint e, float st, float gap) : numP(n), start(s), end(e),
    startTime(st), gapBetweenCars(gap) {};
};


void GenerateTest::generateTest(RoadGraph &inRoadGraph, BTrafficPeople &people,
                                LCGLWidget3D *glWidget3D) {
  printf(">>loadTestRoadGraph\n");
  printf(">>Remove\n");
  inRoadGraph.myRoadGraph.clear();
  inRoadGraph.myRoadGraph_BI.clear();
  glWidget3D->cg.geoZone.blocks.clear();
  glWidget3D->vboRenderManager.removeAllStreetElementName("tree");
  glWidget3D->vboRenderManager.removeAllStreetElementName("streetLamp");
  printf("<<Remove\n");
  /////////////////////////////////////////////////
  // CREATE
  // nodes
  float d = 100.0f;
  std::vector<QVector3D> vertexPos;
  vertexPos.push_back(QVector3D(0, 0, 0));
  vertexPos.push_back(QVector3D(0, -d, 0));
  vertexPos.push_back(QVector3D(d, 0, 0));
  vertexPos.push_back(QVector3D(0, d, 0));
  vertexPos.push_back(QVector3D(-d, 0, 0));

  // edges
  std::vector<std::pair<int, int>> edgesC;
  edgesC.push_back(std::make_pair(1, 0));
  edgesC.push_back(std::make_pair(2, 0));
  edgesC.push_back(std::make_pair(0, 3));
  edgesC.push_back(std::make_pair(0, 4));

  /////////////////////////////////////////////////
  // NODES
  std::vector<RoadGraph::roadGraphVertexDesc> vertex;
  std::vector<RoadGraph::roadGraphVertexDesc> vertex_SIM;
  vertex.resize(vertexPos.size());
  vertex_SIM.resize(vertexPos.size());
  QVector3D minBox(FLT_MAX, FLT_MAX, 0);
  QVector3D maxBox(-FLT_MAX, -FLT_MAX, 0);

  for (int v = 0; v < vertexPos.size(); v++) {

    vertex[v] = boost::add_vertex(inRoadGraph.myRoadGraph_BI);
    inRoadGraph.myRoadGraph_BI[vertex[v]].pt = vertexPos[v];
    inRoadGraph.myRoadGraph_BI[vertex[v]].prio = 0;
    inRoadGraph.myRoadGraph_BI[vertex[v]].type = 0;

    vertex_SIM[v] = boost::add_vertex(inRoadGraph.myRoadGraph);
    inRoadGraph.myRoadGraph[vertex_SIM[v]].pt = vertexPos[v];
    inRoadGraph.myRoadGraph[vertex_SIM[v]].prio = 0;
    inRoadGraph.myRoadGraph[vertex_SIM[v]].type = 0;

    updateMinMax3(vertexPos[v], minBox, maxBox);
  }

  // find limits
  float sqSideSz = std::max(maxBox.x() - minBox.x(),
                            maxBox.y() - minBox.y()) * 2.0f;
  printf("MinBox %f %f MaxBox %f %f--> %f %f -->sqSide %f\n", minBox.x(),
         minBox.y(), maxBox.x(), maxBox.y(), maxBox.x() - minBox.x(),
         maxBox.y() - minBox.y(), sqSideSz);

  // terrain
  G::boundingPolygon.clear();
  QVector3D tmpPt;
  tmpPt = QVector3D(-sqSideSz, -sqSideSz, 0.0f);
  G::boundingPolygon.push_back(tmpPt);
  tmpPt = QVector3D(-sqSideSz,  sqSideSz, 0.0f);
  G::boundingPolygon.push_back(tmpPt);
  tmpPt = QVector3D(sqSideSz,  sqSideSz, 0.0f);
  G::boundingPolygon.push_back(tmpPt);
  tmpPt = QVector3D(sqSideSz, -sqSideSz, 0.0f);
  G::boundingPolygon.push_back(tmpPt);

  //resize terrain
  glWidget3D->vboRenderManager.changeTerrainDimensions(sqSideSz * 4, 100.0f);
  QString flat_path("data/flat.png");
  glWidget3D->vboRenderManager.vboTerrain.loadTerrain(flat_path);
  printf("Resize Terrain %f\n", sqSideSz);

  /////////////////////////////////////////
  // EDGES
  std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e0_pair;
  std::pair<RoadGraph::roadGraphEdgeDesc, bool> e0_pair_SIMP;
  float totalLeng = 0;
  int numMaxLanes = 0;

  for (int eN = 0; eN < edgesC.size(); eN++) {

    int start = edgesC[eN].first;
    int end = edgesC[eN].second;
    int numLanes = 1;
    float speed = 20 * 0.44704f; //mph to mps
    QString label = "";
    float resfac = 1.0f;
    float lengh = (inRoadGraph.myRoadGraph[vertex_SIM[start]].pt -
                   inRoadGraph.myRoadGraph[vertex_SIM[end]].pt).length();

    totalLeng += lengh;

    // add edge if not already there or update num lanes
    if (boost::edge(vertex_SIM[start], vertex_SIM[end],
                    inRoadGraph.myRoadGraph).second == false) {
      e0_pair_SIMP = boost::add_edge(vertex[start], vertex[end],
                                     inRoadGraph.myRoadGraph);
      inRoadGraph.myRoadGraph[e0_pair_SIMP.first].numberOfLanes = numLanes;
      inRoadGraph.myRoadGraph[e0_pair_SIMP.first].edgeLength = lengh;

      inRoadGraph.myRoadGraph[e0_pair_SIMP.first].label = label;
      inRoadGraph.myRoadGraph[e0_pair_SIMP.first].resfac = resfac;
    } else {
      inRoadGraph.myRoadGraph[boost::edge(vertex_SIM[start], vertex_SIM[end],
                                                                         inRoadGraph.myRoadGraph).first].numberOfLanes += numLanes;
    }

    e0_pair = boost::add_edge(vertex[start], vertex[end],
                              inRoadGraph.myRoadGraph_BI);
    inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes = numLanes;
    inRoadGraph.myRoadGraph_BI[e0_pair.first].edgeLength = lengh;
    inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeedMperSec = speed;
  }

  /////////////////////////////////////////////////////
  // INTERSECTIONS

  printf("<< Test #Edges %d (Length %.0f km) #Vertex %d MaxNumLines %d\n",
         boost::num_edges(inRoadGraph.myRoadGraph_BI), totalLeng / 1000.0f,
         boost::num_vertices(inRoadGraph.myRoadGraph_BI), numMaxLanes);

  /////////////////////////////////////////
  // DEMAND
  float startTime = 4.5f;

  std::vector<Demand> demand;
  demand.push_back(Demand(100, 1, 4, startTime, 15.0f / 3600.0f));
  demand.push_back(Demand(100, 2, 3, startTime, 15.0f / 3600.0f));

  for (int dN = 0; dN < demand.size(); dN++) {
    int cNumPeople = people.numPeople;
    int numP = demand[dN].numP;
    people.resize(cNumPeople + numP);
    printf("gerate numP %d start %.2f (%.2f gap) vertex %u %u (%u %u)\n", numP,
           demand[dN].startTime, demand[dN].gapBetweenCars, demand[dN].start,
           demand[dN].end, vertex[demand[dN].start], vertex[demand[dN].end]);

    for (int p = 0; p < numP; p++) {
      float startTime = demand[dN].startTime + p * demand[dN].gapBetweenCars;
      BPMTrafficPerson::randomPerson(p + cNumPeople, people, vertex[demand[dN].start],
                                     vertex[demand[dN].end], startTime);
    }
  }//demand

  printf("<< Test numPeople %d\n", people.numPeople);

}//

}
