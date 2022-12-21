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
*		@desc Class to load the Dynameq road graph
*		@author igarciad
************************************************************************************************/

#include "Geometry/client_geometry.h"
#include "LC_UrbanMain.h"
#include "LC_GLWidget3D.h"

#include "roadGraphDynameqLoader.h"
#include "global.h"
#include "bTraffic/bTrafficIntersection.h"


namespace LC {

QHash<int, std::vector<uint>> RoadGraphDynameq::centroidsToVertex;


void addEdgeD(int ind1, int ind2, RoadGraph &inRoadGraph,
              std::vector<RoadGraph::roadGraphVertexDesc> &vertex,
              std::vector<RoadGraph::roadGraphVertexDesc> &vertex_SIM) {
  std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e0_pair;
  std::pair<RoadGraph::roadGraphEdgeDesc, bool> e0_pair_SIMP;
  float length = (inRoadGraph.myRoadGraph_BI[vertex[ind1]].pt -
                  inRoadGraph.myRoadGraph_BI[vertex[ind2]].pt).length();

  e0_pair = boost::add_edge(vertex[ind1], vertex[ind2],
                            inRoadGraph.myRoadGraph_BI);
  inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes = 2;
  inRoadGraph.myRoadGraph_BI[e0_pair.first].edgeLength = length;
  inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeedMperSec =
    G::global().getFloat("cuda_arterial_edges_speed_ms");

  e0_pair = boost::add_edge(vertex[ind2], vertex[ind1],
                            inRoadGraph.myRoadGraph_BI);
  inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes = 2;
  inRoadGraph.myRoadGraph_BI[e0_pair.first].edgeLength = length;
  inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeedMperSec =
    G::global().getFloat("cuda_arterial_edges_speed_ms");

  e0_pair_SIMP = boost::add_edge(vertex_SIM[ind1], vertex_SIM[ind2],
                                 inRoadGraph.myRoadGraph);
  inRoadGraph.myRoadGraph[e0_pair_SIMP.first].numberOfLanes = 4;


}//

typedef QHash<QString, QVariant> structD;

void updateMinMax2(QVector3D &newPoint, QVector3D &minBox, QVector3D &maxBox) {
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

void RoadGraphDynameq::loadDynameqRoadGraph(RoadGraph &inRoadGraph,
    LCGLWidget3D *glWidget3D) {
  //return; /////////// 201711
  printf(">>loadDynameqRoadGraph\n");
  printf(">>Remove\n");
  inRoadGraph.myRoadGraph.clear();
  inRoadGraph.myRoadGraph_BI.clear();
  glWidget3D->cg.geoZone.blocks.clear();
  glWidget3D->vboRenderManager.removeAllStreetElementName("tree");
  glWidget3D->vboRenderManager.removeAllStreetElementName("streetLamp");
  centroidsToVertex.clear();
  printf("<<Remove\n");
  /////////////////////////////////

  /*/////////////////////////////////

  /////////////////////////////////
  // roads
  int numGrid=ceil(sqSideSz*2.0f/gridSize);
  float actualGridSize=sqSideSz*2.0f/numGrid;
  numGrid++;
  //
  std::vector<RoadGraph::roadGraphVertexDesc> vertex;
  std::vector<RoadGraph::roadGraphVertexDesc> vertex_SIM;
  vertex.resize(numGrid*numGrid);
  vertex_SIM.resize(numGrid*numGrid);
  for(int yS=0;yS<numGrid;yS++){
        for(int xS=0;xS<numGrid;xS++){
                int index=yS*numGrid+xS;
                vertex[index]=boost::add_vertex(inRoadGraph.myRoadGraph_BI);
                QVector3D pos(xS*actualGridSize-sqSideSz,yS*actualGridSize-sqSideSz,0.0f);
                inRoadGraph.myRoadGraph_BI[vertex[index]].pt=pos;

                vertex_SIM[index]=boost::add_vertex(inRoadGraph.myRoadGraph);
                inRoadGraph.myRoadGraph[vertex_SIM[index]].pt=pos;

        }
  }
  // edges
  std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e0_pair;
  std::pair<RoadGraph::roadGraphEdgeDesc, bool> e0_pair_SIMP;
  for(int yS=0;yS<numGrid;yS++){
        for(int xS=0;xS<numGrid;xS++){
                int cInd=yS*numGrid+xS;
                int cInd_X=yS*numGrid+(xS+1);
                int cInd_Y=(yS+1)*numGrid+xS;

                if(xS!=numGrid-1){//last row does not have horizontal
                        addEdgeD(cInd,cInd_X,inRoadGraph,vertex,vertex_SIM);

                }

                if(yS!=numGrid-1){//last collumn does not have vertical
                        addEdgeD(cInd,cInd_Y,inRoadGraph,vertex,vertex_SIM);
                }
        }
        }*/
  printf("1\n");
  QString fileName = "data/Dynameq/sf_final_base.dqt";
  //QString fileName="data/Dynameq/smallTestNet_base.dqt";
  QFile baseFile(fileName); // Create a file handle for the file named

  QString line;

  if (!baseFile.open(QIODevice::ReadOnly | QIODevice::Text)) { // Open the file
    printf("Can't open file '%s'\n", fileName.toUtf8().constData());
    return;
  }

  QTextStream stream(&baseFile);   // Set the stream to read from myFile
  printf("2\n");
  /////////////////////////////////////////////////
  // READ FILE
  QTime timer;
  timer.start();

  QStringList fields;
  QStringList fieldsNames;
  QHash<int, structD> nodes;
  QHash<int, structD> centroids;
  QHash<int, structD> links;
  QHash<int, std::vector<int>> virtualLinks;

  printf("3\n");
  int mode = 0;
  int nodesFields;

  while (!stream.atEnd()) {
    line = stream.readLine();

    //qDebug()<<line;
    // MODES
    if (line.startsWith("NODES")) {
      mode = 1;
      nodesFields = 0;
      continue;
    }

    if (line.startsWith("CENTROIDS")) {
      mode = 2;
      nodesFields = 0;
      continue;
    }

    if (line.startsWith("LINKS")) {
      mode = 3;
      nodesFields = 0;
      continue;
    }

    if (line.startsWith("VIRTUAL_LINKS")) {
      mode = 4;
      nodesFields = 0;
      continue;
    }

    if (line.startsWith("LANE_PERMS")) {
      mode = 0;
      nodesFields = 0;
      continue;
    }

    if (line.startsWith("LINK_EVENTS")) {
      mode = 0;
      nodesFields = 0;
      continue;
    }

    if (line.startsWith("MOVEMENT_EVENTS")) {
      mode = 0;
      nodesFields = 0;
      continue;
    }

    if (line.startsWith("MOVEMENT")) { // !!! 201711
      mode = 0;
      nodesFields = 0;
      continue;
    }

    if (mode == 0) {
      continue;
    }

    fields = line.split(' ', QString::SkipEmptyParts);

    if (nodesFields == 0) {
      nodesFields = fields.size();
      fieldsNames = fields;
      continue;
    }

    int id = fields[0].toInt();

    for (int fN = 1; fN < fieldsNames.size(); fN++) {
      if (mode == 1) {
        nodes[id][fieldsNames[fN]] = fields[fN - 1];
      }

      if (mode == 2) {
        centroids[id][fieldsNames[fN]] = fields[fN - 1];
      }

      if (mode == 3) {
        links[id][fieldsNames[fN]] = fields[fN - 1];
      }

      if (mode == 4) {
        virtualLinks[id].push_back(fields[1].toInt());
      }
    }

    //return;
  }

  printf("4\n");
  printf("\nNodes readed in %d Nod %d Cen %d Link %d\n", timer.elapsed(),
         nodes.size(), centroids.size(), links.size());

  /////////////////////////////////////////////////
  // NODES
  // find limits
  QVector3D minBox(FLT_MAX, FLT_MAX, 0);
  QVector3D maxBox(-FLT_MAX, -FLT_MAX, 0);
  QHashIterator<int, structD> i(nodes);

  while (i.hasNext()) {
    i.next();
    float x = nodes[i.key()]["x-coordinate"].toFloat();
    float y = nodes[i.key()]["y-coordinate"].toFloat();
    QVector3D min_max_v(x, y, 0);
    updateMinMax2(min_max_v, minBox, maxBox);
  }

  printf("MinBox %f %f MaxBox %f %f--> %f %f\n", minBox.x(), minBox.y(),
         maxBox.x(), maxBox.y(), maxBox.x() - minBox.x(), maxBox.y() - minBox.y());
  // set up world
  float scale = 0.3048f; //feet to meters
  float sqSideSz = std::max(maxBox.x() - minBox.x(),
                            maxBox.y() - minBox.y()) * scale / 2.0f;
  QVector3D centerV(-minBox.x(), -minBox.y(), 0);
  QVector3D centerAfterSc(-sqSideSz, -sqSideSz, 0);
  // camera
  //clientMain->glWidget3D->spaceRadius=sqSideSz*2.0f;
  //clientMain->glWidget3D->resizeGL(clientMain->mGLWidget_3D->width(),clientMain->mGLWidget_3D->height());//force to change far plane

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
  glWidget3D->vboRenderManager.changeTerrainDimensions(sqSideSz * 2 + 400.0f,
      200);
  QString sfo_path("data/sfo.png");
  glWidget3D->vboRenderManager.vboTerrain.loadTerrain(sfo_path);
  printf("Resize Terrain %f\n", sqSideSz);

  // add nodes
  std::vector<RoadGraph::roadGraphVertexDesc> vertex;
  std::vector<RoadGraph::roadGraphVertexDesc> vertex_SIM;
  vertex.resize(nodes.size());
  vertex_SIM.resize(nodes.size());
  QHashIterator<int, structD> i2(nodes);
  int index = 0;
  QHash<int, int> dynIndToInd;
  QHash<int, std::pair<uint, uint>> dynEdgToEdge;
  QHash<int, int> indToDynInd;
  QSet<uchar> prio;

  while (i2.hasNext()) {
    i2.next();
    float x = nodes[i2.key()]["x-coordinate"].toFloat();
    float y = nodes[i2.key()]["y-coordinate"].toFloat();
    int ind = i2.key();

    QVector3D pos(x, y, 0);
    pos += centerV; //center
    pos *= scale;
    pos += centerAfterSc;

    vertex[index] = boost::add_vertex(inRoadGraph.myRoadGraph_BI);
    inRoadGraph.myRoadGraph_BI[vertex[index]].pt = pos;
    inRoadGraph.myRoadGraph_BI[vertex[index]].prio =
      nodes[i2.key()]["priority"].toInt();
    inRoadGraph.myRoadGraph_BI[vertex[index]].type =
      nodes[i2.key()]["type"].toInt();

    vertex_SIM[index] = boost::add_vertex(inRoadGraph.myRoadGraph);
    inRoadGraph.myRoadGraph[vertex_SIM[index]].pt = pos;
    inRoadGraph.myRoadGraph[vertex_SIM[index]].prio =
      nodes[i2.key()]["priority"].toInt();
    inRoadGraph.myRoadGraph[vertex_SIM[index]].type =
      nodes[i2.key()]["type"].toInt();

    prio.insert(inRoadGraph.myRoadGraph[vertex_SIM[index]].prio);

    dynIndToInd[ind] = index;
    indToDynInd[index] = ind;
    index++;
  }

  printf("3\n");
  /*printf("Dif Prios:\n");
  QSetIterator<uchar> prioI(prio);
  while (prioI.hasNext())
        qDebug() << prioI.next();
  exit(0);*/
  /////////////////////////////////////////
  // EDGES
  QHashIterator<int, structD> iL(links);
  std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e0_pair;
  std::pair<RoadGraph::roadGraphEdgeDesc, bool> e0_pair_SIMP;
  float totalLeng = 0;
  int numMaxLanes = 0;

  while (iL.hasNext()) {
    iL.next();
    int ind = iL.key();
    int start = links[iL.key()]["start"].toInt();
    int end = links[iL.key()]["end"].toInt();
    int numLanes = links[iL.key()]["lanes"].toInt();
    numMaxLanes = numMaxLanes > numLanes ? numMaxLanes : numLanes;
    float speed = links[iL.key()]["fspeed"].toFloat() * 0.44704f; //mph to mps

    QString label = links[iL.key()]["label"].toString(); //
    label = label.remove(QChar('"'));
    float resfac = links[iL.key()]["resfac"].toFloat(); //Response time factor
    uint faci = links[iL.key()]["faci"].toInt();//Link facility type (priority)
    //float lengh=(inRoadGraph.myRoadGraph[vertex_SIM[dynIndToInd[start]]].pt-inRoadGraph.myRoadGraph[vertex_SIM[dynIndToInd[end]]].pt).length();
    float lengh = links[iL.key()]["len"].toFloat() * 1609.344f;
    // LENGTH--> Extract from file?
    totalLeng += lengh;

    // add edge if not already there or update num lanes
    if (boost::edge(vertex_SIM[dynIndToInd[start]], vertex_SIM[dynIndToInd[end]],
                    inRoadGraph.myRoadGraph).second == false) {
      e0_pair_SIMP = boost::add_edge(vertex[dynIndToInd[start]],
                                     vertex[dynIndToInd[end]], inRoadGraph.myRoadGraph);
      inRoadGraph.myRoadGraph[e0_pair_SIMP.first].numberOfLanes = numLanes;
      inRoadGraph.myRoadGraph[e0_pair_SIMP.first].edgeLength = lengh;

      inRoadGraph.myRoadGraph[e0_pair_SIMP.first].label = label;
      inRoadGraph.myRoadGraph[e0_pair_SIMP.first].resfac = resfac;
      inRoadGraph.myRoadGraph[e0_pair_SIMP.first].faci = faci;
    } else {
      inRoadGraph.myRoadGraph[boost::edge(vertex_SIM[dynIndToInd[start]],
                                          vertex_SIM[dynIndToInd[end]],
                                          inRoadGraph.myRoadGraph).first].numberOfLanes += numLanes;
    }

    e0_pair = boost::add_edge(vertex[dynIndToInd[start]], vertex[dynIndToInd[end]],
                              inRoadGraph.myRoadGraph_BI);
    inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes = numLanes;
    inRoadGraph.myRoadGraph_BI[e0_pair.first].edgeLength = lengh;
    inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeedMperSec = speed;
    inRoadGraph.myRoadGraph_BI[e0_pair.first].label = label;
    inRoadGraph.myRoadGraph_BI[e0_pair.first].resfac = resfac;
    inRoadGraph.myRoadGraph_BI[e0_pair.first].faci = faci;

    // add to edge
    dynEdgToEdge[ind] = std::make_pair(dynIndToInd[start], dynIndToInd[end]);
  }

  /////////////////////////////////////////
  // CENTROIDS
  QHashIterator<int, std::vector<int>> iV(virtualLinks);

  while (iV.hasNext()) {
    iV.next();
    int centr = iV.key();

    for (int i = 0; i < virtualLinks[centr].size(); i++) {
      int linkId = virtualLinks[centr][i];

      int start = links[linkId]["start"].toInt();
      int end = links[linkId]["end"].toInt();
      centroidsToVertex[centr].push_back(dynIndToInd[start]);
      centroidsToVertex[centr].push_back(dynIndToInd[end]);
    }
  }

  //printf("<<loadDynameqRoadGraph #Edges %d (Length %.0f km) #Vertex %d MaxNumLines %d\n", boost::num_edges(inRoadGraph.myRoadGraph_BI), totalLeng / 1000.0f, boost::num_vertices(inRoadGraph.myRoadGraph_BI), numMaxLanes);
  //printf("<<loadDynameqRoadGraph #Edges %d #Vertex %d\n", boost::num_edges(inRoadGraph.myRoadGraph), boost::num_vertices(inRoadGraph.myRoadGraph));

  /////////////////////////////////////////
  // INTERSECTIONS
  //glWidget3D->bTrafficSimulator.
  //glWidget3D->bTrafficSimulator.createIntersections(true);
  inRoadGraph.fillInOutNumForEdges();
 /* BTrafficIntersection::createIntersectionsDynameq(inRoadGraph,
      glWidget3D->bTrafficSimulator.intersec, glWidget3D->bTrafficSimulator.stops,
      glWidget3D->bTrafficSimulator.trafficLights, dynIndToInd, dynEdgToEdge);
      // commented on 2018 04 07
      */
  return; /// !!!! REMOVE 'return' to check
  // check things
  RoadGraph::roadGraphVertexIter vi, vend;
  int numMaxOutEdges = 0;
  int numMaxInEdges = 0;

  for (boost::tie(vi, vend) = boost::vertices(inRoadGraph.myRoadGraph_BI);
       vi != vend; ++vi) {
    RoadGraph::out_roadGraphEdgeIter oei, oeend;;
    int numOutEdges = boost::out_degree(*vi, inRoadGraph.myRoadGraph_BI);

    if (numMaxOutEdges < numOutEdges) {
      numMaxOutEdges = numOutEdges;
    }

    int numInEdges = boost::in_degree(*vi, inRoadGraph.myRoadGraph_BI);

    if (numMaxInEdges < numInEdges) {
      numMaxInEdges = numInEdges;
    }
  }

  printf("<<loadDynameqRoadGraph numMaxOutEdges %d numMaxInEdges %d numMaxLanes %d\n",
         numMaxOutEdges, numMaxInEdges, numMaxLanes);
}//

}
