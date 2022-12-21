/************************************************************************************************
*		@desc Class to load the B2018 road graph
*		@author igarciad
************************************************************************************************/
#pragma once
#include <QHash>
#include <QVector2D>
#include <stdexcept>
#include <iostream>
#include <string>
using namespace std;

#include "Geometry/client_geometry.h"
#include "bTraffic/bTrafficIntersection.h"
#include "global.h"
#include "roadGraphB2018Loader.h"

namespace LC {

using namespace std::chrono;

std::vector<DemandB2018> RoadGraphB2018::demandB2018;
int RoadGraphB2018::totalNumPeople;
QHash<int, uint64_t> RoadGraphB2018::indToNodeIndex;

void updateMinMax2(const QVector2D &newPoint, QVector2D &minBox, QVector2D &maxBox) {
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

///////////////////////////////////////////////////////////

static QVector2D projLatLonToWorldMercator(float lat, float lon,
    bool isDeg = false) {
  // Ellipsoid model constants (actual values here are for WGS84)
  const float sm_a = 6378137.0f;
  const float sm_b = 6356752.314f;

  QVector2D result;
  float lon0 = 0;

  if (isDeg == true) {
    lat = lat / 180.0f * M_PI;
    lon = lon / 180.0f * M_PI;
  }

  //printf("lat %.2f lon %.2f\n", lat, lon);
  //result.setX(sm_a*(lon - lon0));
  //result.setY(sm_a*log((sin(lat) + 1) / cos(lat)));

  result.setX(sm_a * (cos(lat) * cos(lon)));
  result.setY(sm_a * (cos(lat) * sin(lon)));

  //qDebug() << result;
  return  result;
}//

void saveSetToFile(QSet<uint64_t> &set, QString &filename) {
  QFile file(filename);

  if (file.open(QIODevice::ReadWrite)) {
    QTextStream stream(&file);
    QSetIterator<uint64_t> nA(set);

    while (nA.hasNext()) {
      stream << nA.next() << "\n";
    }
  }
}

//////////////////////////////////////////////////////////

void RoadGraphB2018::loadB2018RoadGraph(RoadGraph &inRoadGraph, QString networkPath) {
  inRoadGraph.myRoadGraph.clear();
  inRoadGraph.myRoadGraph_BI.clear();

  QString nodesFileName = networkPath + "nodes.csv";
  QString edgeFileName = networkPath + "edges.csv";
  QString odFileName = networkPath + "od_demand.csv";

  std::cerr
    << "Using "
    << nodesFileName.toUtf8().constData() << " as nodes' file, "
    << edgeFileName.toUtf8().constData() << " as edges' file, "
    << odFileName.toUtf8().constData() << " as od demands' file"
    << std::endl;

  /////////////////////////////////////////////////
  // READ NODES

  QFile baseFile(nodesFileName); // Create a file handle for the file named
  QString line;
  if (!baseFile.open(QIODevice::ReadOnly | QIODevice::Text)) { // Open the file
    throw std::invalid_argument("RoadGraphB2018::loadB2018RoadGraph -> Can't open nodes files.");
  }

  QTextStream stream(&baseFile); // Set the stream to read from myFile
  QTime timer;
  timer.start();
  QVector2D minBox(FLT_MAX, FLT_MAX);
  QVector2D maxBox(-FLT_MAX, -FLT_MAX);
  QHash<uint64_t, QVector2D> nodeIndexToVertexLoc;
  QHash<uint64_t, uchar> nodeIndexToBType; // node type

  QHash<QString, uchar> bTypeStringTobType;
  bTypeStringTobType[""] = 0;
  bTypeStringTobType["motorway_junction"] = 1;
  bTypeStringTobType["traffic_signals"] = 2;
  bTypeStringTobType["stop"] = 3;
  bTypeStringTobType["turning_circle"] = 4;

  QStringList headers = stream.readLine().split(",");
  const int indexOsmid = headers.indexOf("osmid"); // the original value of osmid
  const int indexX = headers.indexOf("x");
  const int indexY = headers.indexOf("y");
  const int indexHigh = headers.indexOf("highway");
  const int indexNodeIndex = headers.indexOf("index"); // the normalized index used to identify nodes across the code

  while (!stream.atEnd()) {
    line = stream.readLine();
    QStringList fields = line.split(',', QString::SkipEmptyParts);

    if (indexX >= fields.size() || indexY >= fields.size()) {
      qDebug() << "ERROR line " << line << " --> SKIP";
      continue;
    }

    float x = fields[indexX].toFloat();
    float y = fields[indexY].toFloat();
    //qDebug() << "x " << x << " y " << y;
    uint64_t nodeIndex = fields[indexNodeIndex].toLongLong();
    nodeIndexToVertexLoc[nodeIndex] = QVector2D(x, y);
    updateMinMax2(QVector2D(x, y), minBox, maxBox);

    if (indexHigh >= fields.size()) {
      nodeIndexToBType[nodeIndex] = 0;
    } else {
      QString bType = fields[indexHigh];
      nodeIndexToBType[nodeIndex] = (!bTypeStringTobType.contains(bType)) ? 0 :
                            bTypeStringTobType[bType];
    }
  }

  // Update coordenades to East-North-Up coordinates;
  const float lat0 = (maxBox.x() + minBox.x()) * 0.5f;
  const float lon0 = (maxBox.y() + minBox.y()) * 0.5f;
  minBox = QVector2D(FLT_MAX, FLT_MAX);
  maxBox = QVector2D(-FLT_MAX, -FLT_MAX);
  QHash<uint64_t, QVector2D>::iterator i;

  for (i = nodeIndexToVertexLoc.begin(); i != nodeIndexToVertexLoc.end(); ++i) {
    nodeIndexToVertexLoc[i.key()] = projLatLonToWorldMercator(i.value().x(),
                                i.value().y(), /*isDeg=*/true);
    updateMinMax2(nodeIndexToVertexLoc[i.key()], minBox, maxBox);
  }

  // TERRAIN
  float scale = 1.0f;
  float sqSideSz = std::max<float>(maxBox.x() - minBox.x(),
                            maxBox.y() - minBox.y()) * scale * 0.5f; // half side
  QVector3D centerV(-minBox.x(), -minBox.y(), 0);
  QVector3D centerAfterSc(-sqSideSz, -sqSideSz, 0);
  G::boundingPolygon.clear();
  G::boundingPolygon.push_back(QVector3D(sqSideSz, -sqSideSz, 0.0f));

  ///////////////////////////////
  // ADD NODES
  std::vector<RoadGraph::roadGraphVertexDesc> vertex;
  std::vector<RoadGraph::roadGraphVertexDesc> vertex_SIM;
  vertex.resize(nodeIndexToVertexLoc.size());
  vertex_SIM.resize(nodeIndexToVertexLoc.size());

  int index = 0;
  QHash<uint64_t, int> dynIndToInd;
  
  for (i = nodeIndexToVertexLoc.begin(); i != nodeIndexToVertexLoc.end(); ++i) {
    uint64_t ind = i.key();

    float x = nodeIndexToVertexLoc[ind].x();
    float y = nodeIndexToVertexLoc[ind].y();
    uchar bType = nodeIndexToBType[ind];

    QVector3D pos(x, y, 0);
    pos += centerV;//center
    pos *= scale;
    pos += centerAfterSc;
    pos.setX(pos.x() * -1.0f); // seems vertically rotated

    vertex[index] = boost::add_vertex(inRoadGraph.myRoadGraph_BI);
    inRoadGraph.myRoadGraph_BI[vertex[index]].pt = pos;
    inRoadGraph.myRoadGraph_BI[vertex[index]].bType = bType;

    vertex_SIM[index] = boost::add_vertex(inRoadGraph.myRoadGraph);
    inRoadGraph.myRoadGraph[vertex_SIM[index]].pt = pos;
    inRoadGraph.myRoadGraph[vertex_SIM[index]].bType = bType;

    dynIndToInd[ind] = index;
    indToNodeIndex[index] = ind;
    index++;
  }

  ///////////////////////////////
  // EDGES
  QFile linkFile(edgeFileName); // Create a file handle for the file named
  if (!linkFile.open(QIODevice::ReadOnly | QIODevice::Text)) { // Open the file
    throw std::invalid_argument("RoadGraphB2018::loadB2018RoadGraph -> Can't open edges files.");
  }

  QTextStream streamL(&linkFile); // Set the stream to read

  headers = (streamL.readLine()).split(",");
  const int indexId = headers.indexOf("uniqueid");
  const int indexU = headers.indexOf("u");
  const int indexV = headers.indexOf("v");
  const int indexLen = headers.indexOf("length");
  const int indexLanes = headers.indexOf("lanes");
  const int indexSpeedMH = headers.indexOf("speed_mph");

  QHash<int, std::pair<uint, uint>> dynEdgToEdge;
  std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e0_pair;
  std::pair<RoadGraph::roadGraphEdgeDesc, bool> e0_pair_SIMP;
  float totalLeng = 0;
  int numEdges = 0;
  QSet<uint64_t> noAvailableNodes;
  const bool saveNoAvailableNodes = false;

  while (!streamL.atEnd()) {
    line = streamL.readLine();
    QStringList fields = line.split(',', QString::SkipEmptyParts);

    if (fields.size() < 3) {
      qDebug() << "ERROR line " << line << " --> SKIP";
      continue;
    }

    uint ind = fields[indexId].toInt();

    uint64_t start = fields[indexU].toLongLong();
    uint64_t end = fields[indexV].toLongLong();

    if ((!dynIndToInd.contains(start)) || (!dynIndToInd.contains(end))) {
      if (saveNoAvailableNodes) {
        if (!dynIndToInd.contains(start)) {
          noAvailableNodes.insert(start);
        }

        if (!dynIndToInd.contains(end)) {
          noAvailableNodes.insert(end);
        }
      }

      qDebug() << "NO CONTAINS: start" << start << " end " << end;
      exit(-1);
      continue;
    }

    float length = fields[indexLen].toFloat();
    int numLanes = std::max<int>(fields[indexLanes].toInt(),1); // at least one
    float speedMS = std::max<float>(0.01f, fields[indexSpeedMH].toFloat() * 0.44704f); //m/h --> m/sec // force to have a speed

    //printf("%d %d of %d): Leng %.2f #lanes %d speed %.2f\n", dynIndToInd[start], dynIndToInd[end], index, length, numLanes, speedMS);

    totalLeng += length;

    // add edge if not already there or update num lanes
    if (boost::edge(vertex_SIM[dynIndToInd[start]], vertex_SIM[dynIndToInd[end]],
                    inRoadGraph.myRoadGraph).second == false) {
      e0_pair_SIMP = boost::add_edge(vertex[dynIndToInd[start]],
                                     vertex[dynIndToInd[end]], inRoadGraph.myRoadGraph);
      inRoadGraph.myRoadGraph[e0_pair_SIMP.first].numberOfLanes = numLanes;
      inRoadGraph.myRoadGraph[e0_pair_SIMP.first].edgeLength = length;

    } else {
      inRoadGraph.myRoadGraph[boost::edge(vertex_SIM[dynIndToInd[start]],
                                          vertex_SIM[dynIndToInd[end]],
                                          inRoadGraph.myRoadGraph).first].numberOfLanes += numLanes;
    }

    e0_pair = boost::add_edge(vertex[dynIndToInd[start]], vertex[dynIndToInd[end]],
                              inRoadGraph.myRoadGraph_BI);
    inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes = numLanes;
    inRoadGraph.myRoadGraph_BI[e0_pair.first].edgeLength = length;
    inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeedMperSec = speedMS;
    inRoadGraph.myRoadGraph_BI[e0_pair.first].faci = ind;
    // add to edge
    dynEdgToEdge[ind] = std::make_pair(dynIndToInd[start], dynIndToInd[end]);
  }

  // Save no available nodes to file.
  if (saveNoAvailableNodes) {
    QString filename = "noAvailableNodes.txt";
    saveSetToFile(noAvailableNodes, filename);
  }

  ///////////////////////////////
  // DEMAND
  QFile demandFile(odFileName); // Create a file handle for the file named

  if (!demandFile.open(QIODevice::ReadOnly | QIODevice::Text)) { // Open the file
    throw std::invalid_argument("RoadGraphB2018::loadB2018RoadGraph -> Can't open od demand files.");
  }

  QTextStream streamD(&demandFile); // Set the stream to read
  headers = (streamD.readLine()).split(",");
  const int numPeopleIndex = headers.indexOf("PERNO");
  const int origIndex = headers.indexOf("origin");
  const int destIndex = headers.indexOf("destination");

  QSet<uint64_t> noAvailableNodesDemand;
  const bool saveNoAvailableNodesDemand = true;
  totalNumPeople = 0;

  while (!streamD.atEnd()) {
    line = streamD.readLine();
    QStringList fields = line.split(',', QString::SkipEmptyParts);

    if (fields.size() < 4) {
      qDebug() << "ERROR line " << line << " --> SKIP";
      continue;
    }

    uint64_t start = fields[origIndex].toLongLong();
    uint64_t end = fields[destIndex].toLongLong();

    if ((!dynIndToInd.contains(start)) || (!dynIndToInd.contains(end))) {
      if (saveNoAvailableNodesDemand) {
        if (!dynIndToInd.contains(start)) {
          noAvailableNodesDemand.insert(start);
        }
        if (!dynIndToInd.contains(end)) {
          noAvailableNodesDemand.insert(end);
        }
      }
      continue;
    }
    int numPeople = fields[numPeopleIndex].toInt();
    totalNumPeople += numPeople;
    demandB2018.push_back(DemandB2018(numPeople, dynIndToInd[start], dynIndToInd[end]));
  }

  // Save no available nodes to file.
  if (saveNoAvailableNodesDemand) {
    QString filename = "noAvailableNodesDemand.txt";
    saveSetToFile(noAvailableNodesDemand, filename);
  }

  std::cerr 
    << "Network loaded in " << timer.elapsed() << " milliseconds with "
    << num_vertices(inRoadGraph.myRoadGraph_BI) << " vertices, "
    << num_edges(inRoadGraph.myRoadGraph_BI) << " edges, "
    << demandB2018.size() <<  " pairs of demand and "
    << totalNumPeople << " people in total." << std::endl;

}

void RoadGraphB2018::loadABMGraph(
  const std::string& networkPath,
  const std::shared_ptr<abm::Graph>& graph_,
  int start_time, int end_time) {
  
  const std::string& edgeFileName = networkPath + "edges.csv";
  std::cout << edgeFileName << " as edges file\n";

  const std::string& nodeFileName = networkPath + "nodes.csv";
  std::cout << nodeFileName << " as nodes file\n";

  auto start = high_resolution_clock::now();
  //EDGES
  graph_->read_graph_osm(edgeFileName);
  //printf("# of edges: %d\n", graph_->nedges());
  
  //NODES
  graph_->read_vertices(nodeFileName);
  
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(stop - start);
}




}  // Closing namespace LC

