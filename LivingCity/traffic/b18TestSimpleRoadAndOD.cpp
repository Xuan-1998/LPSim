
/************************************************************************************************
*		@desc Class to create simple road and OD demmand (for simple testing)
*		@author igarciad
************************************************************************************************/

#include "b18TestSimpleRoadAndOD.h"

#include "../Geometry/client_geometry.h"
#include "../LC_GLWidget3D.h"
#include "b18TrafficOD.h"
#include "../global.h"


namespace LC {

namespace {
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
  Demand(uint n, uint s, uint e, float st, float gap): numP(n), start(s), end(e),
    startTime(st), gapBetweenCars(gap) {};
};

}  // namespace

void B18TestSimpleRoadAndOD::generateTest(RoadGraph &inRoadGraph,
    std::vector<B18TrafficPerson> &trafficPersonVec,
    float startTimeH, float endTimeH, LCGLWidget3D *glWidget3D) {
  printf(">>loadTestRoadGraph\n");
  printf(">>Remove\n");
  inRoadGraph.myRoadGraph.clear();
  inRoadGraph.myRoadGraph_BI.clear();

  if (glWidget3D != nullptr) {
    glWidget3D->cg.geoZone.blocks.clear();
    glWidget3D->vboRenderManager.removeAllStreetElementName("tree");
    glWidget3D->vboRenderManager.removeAllStreetElementName("streetLamp");
  }

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
  if (glWidget3D) {
    glWidget3D->vboRenderManager.changeTerrainDimensions(sqSideSz * 4, 100.0f);
    QString flat_path("data/flat.png");
    glWidget3D->vboRenderManager.vboTerrain.loadTerrain(flat_path);
  }

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

  std::vector<Demand> demand;
  float gap = 15.0f / 3600.0f;
  //demand.push_back(Demand(1, 1, 4, startTimeH, gap)); // super super simple
  demand.push_back(Demand(100, 1, 4, startTimeH, gap));
  demand.push_back(Demand(100, 2, 3, startTimeH, 15.0f / 3600.0f));

  B18TrafficOD b18OD;

  for (int dN = 0; dN < demand.size(); dN++) {
    int cNumPeople = trafficPersonVec.size();
    int numP = demand[dN].numP;
    trafficPersonVec.resize(cNumPeople + numP);
    printf("gerate numP %d start %.2f (%.2f gap) vertex %u %u (%u %u)\n", numP,
           demand[dN].startTime, demand[dN].gapBetweenCars, demand[dN].start,
           demand[dN].end, vertex[demand[dN].start], vertex[demand[dN].end]);

    for (int p = 0; p < numP; p++) {
      float startTimeH = demand[dN].startTime + p * demand[dN].gapBetweenCars;
      b18OD.randomPerson(p + cNumPeople, trafficPersonVec[p + cNumPeople],
                         vertex[demand[dN].start], vertex[demand[dN].end], startTimeH);
    }
  }//demand

  printf("<< Test numPeople %d\n", trafficPersonVec.size());

}//

}
