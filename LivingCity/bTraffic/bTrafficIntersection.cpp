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
*		@desc Class that contains the structure of the lane maps
*		@author igaciad
************************************************************************************************/


#include "bTrafficIntersection.h"

#include "bTrafficConstants.h"
#include "qstringlist.h"

#define LANE_DEBUG 0

namespace LC {

//bool compareSecondPartTuple (const std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI,float> &i, const std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI,float> &j) {
//	return (i.second<j.second);
//}

void BTrafficIntersection::createIntersections(
  RoadGraph &inRoadGraph,
  BIntersectionsData &intersec,
  BStopsData &stops) {

  //////////////////////////////////////////////////////////
  // GENERATE INTERSECTION INFO
  RoadGraph::roadGraphVertexIter_BI vi, viEnd;
  RoadGraph::in_roadGraphEdgeIter_BI Iei, Iei_end;
  RoadGraph::out_roadGraphEdgeIter_BI Oei, Oei_end;
  intersec.resize(boost::num_vertices(
                    inRoadGraph.myRoadGraph_BI));//as many as vertices
  stops.resize(boost::num_vertices(
                 inRoadGraph.myRoadGraph_BI));//!!! this should not be the case

  if (LANE_DEBUG) {
    printf(">>Generate Intersection Info\n");
  }

  for (boost::tie(vi, viEnd) = boost::vertices(inRoadGraph.myRoadGraph_BI);
       vi != viEnd; ++vi) {
    intersec.numIn[*vi] = boost::in_degree(*vi,
                                           inRoadGraph.myRoadGraph_BI);//(*vi,inRoadGraph.myRoadGraph_BI)+boost::in_degree(*vi,inRoadGraph.myRoadGraph_BI);

    if (intersec.numIn[*vi] <= 1) {
      intersec.trafficLight[*vi] = -1;
      continue;//always green
    }

    intersec.nextEvent[*vi] = 0.0f;
    intersec.req[*vi] = 0;
    intersec.trafficLight[*vi] = 0;

    int intersecType = 0;//stop

    if (intersecType == 0) {
      intersec.type[*vi] = 0;
      stops.state[*vi] = 0;
      stops.intersNumber[*vi] = *vi;
    }



  }

  if (LANE_DEBUG) {
    printf("<<Generate Intersection Info\n");
  }

}//

void errorLine(QString text, int lN) {
  printf("\nERROR Line[%d]: %s\n", lN, text.toLocal8Bit().data());
  exit(0);
}

void BTrafficIntersection::createIntersectionsDynameq(
  RoadGraph &inRoadGraph,
  BIntersectionsData &intersec,
  BStopsData &stops,
  BTrafficLightData &trafficLights,
  QHash<int, int> &dynIndToInd,
  QHash<int, std::pair<uint, uint>> &dynEdgToEdge) {

  ////////////////////////////////////////////////////////
  // GENERATE TRAFFIC LIGHTS
  QSet<uint> processed;
  QString fileName = "data/Dynameq/sf_final_ctrl.dqt";
  QFile cntFile(fileName); // Create a file handle for the file named

  if (!cntFile.open(QIODevice::ReadOnly | QIODevice::Text)) { // Open the file
    printf("Can't open file '%s'\n", fileName.toUtf8().constData());
    return;
  }

  QTextStream stream(&cntFile); // Set the stream to read from myFile
  QStringList lines;

  while (stream.atEnd() ==
         false) {//we could do it per lines but more clear text this way
    lines.push_back(stream.readLine());
  }

  bool start = false;
  int lN = 0;
  bool ok;
  float maxPhase = 0;

  while (true) {

    if (lN >= lines.size()) { //this should not happen
      break;
    }

    if (start == false) {//skip starting lines
      if (lines[lN].startsWith("NODE") == false) {
        lN++;
        continue;
      } else {
        start = true;
        continue;
      }
    }

    // 1. read node number
    ++lN;//skip NODE LINE
    int nodeNum = lines[lN++].toInt(&ok);

    if (ok == false) {
      errorLine("nodeNum", lN);
    }

    //printf("nodeNum %d\n",nodeNum);
    // 2. read plan offset
    if (lines[lN++].startsWith("PLAN") == false) {
      errorLine("PLAN", lN);
    }

    QStringList plS = lines[lN++].split(' ', QString::SkipEmptyParts);

    if (plS.size() != 4) {
      errorLine("plane line", lN);
    }

    int planOff = plS[1].toInt(&ok);

    if (ok == false) {
      errorLine("planOff", lN);
    }

    if (plS[0].toInt() == 0) {
      printf("ERROR: All were constant plan\n");
    }

    if (plS[3].toInt() == 0) {
      printf("ERROR: All were turn on red\n");
    }

    // 3. read phases
    if (lines[lN].startsWith("NODE") == true) {
      printf("ERROR: NODE AND PLAN BUT NOT PAHSES");
      continue;
    }

    std::vector<QString> phaseTimes;
    std::vector<std::vector<QString>> phases;

    while (true) {
      //printf("lN %d\n",lN);
      //qDebug() << lines[lN];
      if (lN >= lines.size() || lines[lN].startsWith("NODE")) {
        break;
      }

      if (lines[lN].startsWith("PHASE") == false) {
        errorLine("PHASE", lN);
      }

      ++lN;//skip PHASE
      QStringList phS = lines[lN].split(' ', QString::SkipEmptyParts);

      if (plS.size() != 4) {
        errorLine("phase line", lN);
      }

      //qDebug() << lines[lN];
      phaseTimes.push_back(lines[lN++]);

      std::vector<QString> curPh;

      while ((lN < lines.size()) &&
             (lines[lN].split(' ', QString::SkipEmptyParts).size() == 3)) {
        curPh.push_back(lines[lN++]);
      }

      phases.push_back(curPh);
    }

    if (phaseTimes.size() == 0) {
      printf("ERRROR: No phases!!\n");
    }

    processed.insert(dynIndToInd[nodeNum]);
    trafficLights.intersNumber.push_back(dynIndToInd[nodeNum]);
    trafficLights.offSet.push_back(planOff);
    trafficLights.phaseInd.push_back(trafficLights.phases.size());
    trafficLights.numPhases.push_back(phaseTimes.size());
    trafficLights.curPhase.push_back(0);
    {
      //print debug
      printf("Node[%d]: Off %d Phases %d\n", nodeNum, planOff, phaseTimes.size());

      for (int i = 0; i < phaseTimes.size(); i++) {
        qDebug() << phaseTimes[i];
        float phaseTime = phaseTimes[i].split(' ',
                                              QString::SkipEmptyParts)[0].toFloat() + phaseTimes[i].split(' ',
                                                  QString::SkipEmptyParts)[1].toFloat();
        maxPhase = maxPhase > phaseTime ? maxPhase : phaseTime;

        for (QString var : phases[i]) {
          qDebug() << " " << var;
        }
      }
    }

    for (int i = 0; i < phaseTimes.size(); i++) {
      //uint phaseTime = phaseTimes[i];
      // Phase time
      float phaseTime = phaseTimes[i].split(' ',
                                            QString::SkipEmptyParts)[0].toFloat() + phaseTimes[i].split(' ',
                                                QString::SkipEmptyParts)[1].toFloat();
      qDebug() << phaseTimes[i] << ": " << phaseTime;
      // Phase itself
      unsigned long phaseV = 0;

      for (QString var : phases[i]) {
        QStringList linkP = var.split(' ', QString::SkipEmptyParts);

        if (linkP.size() != 3) {
          errorLine("phase itself " + var, -1);
        }

        uint linkIn = linkP[0].toInt();
        uint linkOut = linkP[1].toInt();
        bool turnProtected = linkP[2].toInt() == 1;
        std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> eI_pair = boost::edge(
              dynEdgToEdge[linkIn].first, dynEdgToEdge[linkIn].second,
              inRoadGraph.myRoadGraph_BI);
        std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> eO_pair = boost::edge(
              dynEdgToEdge[linkOut].first, dynEdgToEdge[linkOut].second,
              inRoadGraph.myRoadGraph_BI);

        if (eI_pair.second == false || eO_pair.second == false) {
          printf("ERROR: Unkwon edges %u %u\n", linkIn, linkOut);
          continue;
        }

        int in = inRoadGraph.myRoadGraph_BI[eI_pair.first].outNum;
        int out = inRoadGraph.myRoadGraph_BI[eO_pair.first].inNum;
        phaseV |= (1 << (8 * in) + out);
      }

      if (phaseV == 0) {
        printf("ERROR IN PHASE = 0\n");
      } else {
        trafficLights.phaseTime.push_back(uchar(phaseTime *
                                                2.0f)); //*2 to make them uchar
        trafficLights.phases.push_back(phaseV);
      }
    }//

  }

  printf("maxPhase %f\n", maxPhase);
  //exit(0);
  ////////////////////////////


  //////////////////////////////////////////////////////////
  // GENERATE STOPS
  RoadGraph::roadGraphVertexIter_BI vi, viEnd;
  RoadGraph::in_roadGraphEdgeIter_BI Iei, Iei_end;
  RoadGraph::out_roadGraphEdgeIter_BI Oei, Oei_end;
  intersec.resize(boost::num_vertices(
                    inRoadGraph.myRoadGraph_BI));//as many as vertices

  //stops.resize(boost::num_vertices(inRoadGraph.myRoadGraph_BI));//!!! this should not be the case
  if (LANE_DEBUG) {
    printf(">>Generate Intersection Info\n");
  }

  for (boost::tie(vi, viEnd) = boost::vertices(inRoadGraph.myRoadGraph_BI);
       vi != viEnd; ++vi) {

    intersec.numIn[*vi] = boost::in_degree(*vi,
                                           inRoadGraph.myRoadGraph_BI);//(*vi,inRoadGraph.myRoadGraph_BI)+boost::in_degree(*vi,inRoadGraph.myRoadGraph_BI);

    if (intersec.numIn[*vi] <= 1) {
      intersec.trafficLight[*vi] = -1;
      continue;//always green
    }

    intersec.nextEvent[*vi] = 0.0f;
    intersec.req[*vi] = 0;
    intersec.trafficLight[*vi] = 0;

    if (!processed.contains(*vi)) {// not processed yet
      int intersecType = 0;//stop

      ////////////////////////////////////
      // STOP
      if (intersecType == 0) {
        intersec.type[*vi] = 0;
        //stops.state[*vi] = 0;
        //stops.intersNumber[*vi] = *vi;
        stops.state.push_back(0);
        stops.intersNumber.push_back(*vi);
      }
    }


  }

  if (LANE_DEBUG) {
    printf("<<Generate Intersection Info\n");
  }

}//









/*
//sort by angle
QVector3D referenceVector(0,1,0);
QVector3D p0,p1;
std::vector<std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI,float>> edgeAngleOut;
//printf("Out\n");
int numOutEdges=0;

float angleRef=atan2(referenceVector.y(),referenceVector.x());
for(boost::tie(Oei, Oei_end) = boost::out_edges(*vi,inRoadGraph.myRoadGraph_BI); Oei != Oei_end; ++Oei){
if(inRoadGraph.myRoadGraph_BI[*Oei].numberOfLanes==0)
continue;
p0 = inRoadGraph.myRoadGraph_BI[boost::source(*Oei, inRoadGraph.myRoadGraph_BI)].pt;
p1 = inRoadGraph.myRoadGraph_BI[boost::target(*Oei, inRoadGraph.myRoadGraph_BI)].pt;
QVector3D edgeDir=(p1-p0).normalized();// NOTE p1-p0
//float angle=QVector3D::dotProduct(referenceVector,edgeDir);
float angle=angleRef-atan2(edgeDir.y(),edgeDir.x());
edgeAngleOut.push_back(std::make_pair(*Oei,angle));
if(edgeDescToLaneMapNum.find(*Oei)==edgeDescToLaneMapNum.end()){
printf("->ERROR OUT\n");//edge desc not found in map
}
numOutEdges++;
}
//printf("Out %d\n",numOutEdges);
std::vector<std::pair<LC::RoadGraph::roadGraphEdgeDesc_BI,float>> edgeAngleIn;
//printf("In\n");
int numInEdges=0;
for(boost::tie(Iei, Iei_end) = boost::in_edges(*vi,inRoadGraph.myRoadGraph_BI); Iei != Iei_end; ++Iei){
if(inRoadGraph.myRoadGraph_BI[*Iei].numberOfLanes==0)
continue;
p0 = inRoadGraph.myRoadGraph_BI[boost::source(*Iei, inRoadGraph.myRoadGraph_BI)].pt;
p1 = inRoadGraph.myRoadGraph_BI[boost::target(*Iei, inRoadGraph.myRoadGraph_BI)].pt;
QVector3D edgeDir=(p0-p1).normalized();// NOTE p0-p1
float angle=angleRef-atan2(edgeDir.y(),edgeDir.x());
edgeAngleIn.push_back(std::make_pair(*Iei,angle));
if(edgeDescToLaneMapNum.find(*Iei)==edgeDescToLaneMapNum.end()){
printf("->ERROR IN\n");//edge desc not found in map
continue;
}
numInEdges++;
}
intersections[*vi].totalInOutEdges=numOutEdges+numInEdges;
//printf("In %d\n",numInEdges);
//printf("Sort\n");
//save in sorterd way as lane number
if(edgeAngleOut.size()>0){
std::sort( edgeAngleOut.begin(), edgeAngleOut.end(), compareSecondPartTuple);
}
if(edgeAngleIn.size()>0){
std::sort( edgeAngleIn.begin(), edgeAngleIn.end(), compareSecondPartTuple);
}
//!!!!
int outCount=0;
int inCount=0;
int totalCount=0;
//printf("count %d\n",);
// INTERSECTION
//      0xFF00 0000 Num lines
//      0x00FF 0000 in out
//      0x0000 FFFF Edge number
for(int iter=0;iter<edgeAngleOut.size()+edgeAngleIn.size();iter++){
if((outCount<edgeAngleOut.size()&&inCount<edgeAngleIn.size()&&edgeAngleOut[outCount].second<=edgeAngleIn[inCount].second)||
(outCount<edgeAngleOut.size()&&inCount>=edgeAngleIn.size())){
intersections[*vi].edge[totalCount]=edgeDescToLaneMapNum[edgeAngleOut[outCount].first];
intersections[*vi].edge[totalCount]|=(edgesData[intersections[*vi].edge[totalCount]].numLines<<24);//put the number of lines in each edge
intersections[*vi].edge[totalCount]|=0x00000;//mask to define out edge
outCount++;
}else{
intersections[*vi].edge[totalCount]=edgeDescToLaneMapNum[edgeAngleIn[inCount].first];
intersections[*vi].edge[totalCount]|=(edgesData[intersections[*vi].edge[totalCount]].numLines<<24);//put the number of lines in each edge
intersections[*vi].edge[totalCount]|=0x010000;//mask to define in edge
inCount++;
}
totalCount++;
}
if(totalCount!=intersections[*vi].totalInOutEdges){
printf("Error totalCount!=intersections[*vi].totalInOutEdges %d %d\n",totalCount,intersections[*vi].totalInOutEdges);
}*/
}
