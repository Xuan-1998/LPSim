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
//#include "stdafx.h"
#include "roadGraph.h"
#include "../global.h"
//#include "client_global_variables.h"

#define FBO_LENGTH 4096

namespace LC {

/**
* Constructor.
**/
RoadGraph::RoadGraph() {
  initialized = false;
}

/**
* Destructor.
**/
RoadGraph::~RoadGraph() {
  this->clear();
}

/**
* Initialize.
**/
void RoadGraph::init() {
  RoadGraph::roadGraphEdgeIter ei, eiEnd;
  RoadGraph::roadGraphEdgeIter_BI ei_BI, eiEnd_BI;

  //Add lanes to edges
  for (boost::tie(ei, eiEnd) = boost::edges(myRoadGraph);
       ei != eiEnd; ++ei) {
    myRoadGraph[*ei].init();
  }

  for (boost::tie(ei_BI, eiEnd_BI) = boost::edges(myRoadGraph_BI);
       ei_BI != eiEnd_BI; ++ei_BI) {
    myRoadGraph_BI[*ei_BI].init();
  }

  initialized = true;
}//

/**
* Clear
**/
void RoadGraph::clear() {
  this->myRoadGraph.clear();
  this->myRoadGraph_BI.clear();
}//


/**
* Export to OSM
**/
void RoadGraph::writeRoadNetworkToOSM(QTextStream &osmStream) {
  RoadGraph::roadGraphVertexIter vi, viEnd;
  RoadGraph::roadGraphEdgeIter ei, eiEnd;

  int currentWayID = 1;  //index for current id

  // write file header
  osmStream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  osmStream << "<osm version=\"0.6\" generator=\"CGImap 0.0.2\">\n";

  // Temp file to store ways to be printed at end of osm file
  QTemporaryFile wayText("temp.txt");

  if (!wayText.open()) {
    std::cout << "ERROR: Cannot open temporary file";
    return;
  }

  QTextStream wayStream(&wayText);

  osmStream.setRealNumberPrecision(7);
  wayStream.setRealNumberPrecision(7);
  osmStream.setRealNumberNotation(QTextStream::FixedNotation);
  wayStream.setRealNumberNotation(QTextStream::FixedNotation);


  for (boost::tie(ei, eiEnd) = boost::edges(myRoadGraph); ei != eiEnd; ++ei) {
    // store way to print below
    wayStream << "<way id=\"" << currentWayID << "\">\n";

    // manually put start and end nodes into roadSegmentGeometry (until roadSegmentGeometry is fully implemented)
    myRoadGraph[*ei].roadSegmentGeometry.push_back(myRoadGraph[boost::source(*ei,
        myRoadGraph)].pt);
    myRoadGraph[*ei].roadSegmentGeometry.push_back(myRoadGraph[boost::target(*ei,
        myRoadGraph)].pt);

    // Go through geometry nodes and print them
    for (int i = 0; i < myRoadGraph[*ei].roadSegmentGeometry.size(); i++) {
      //print nodes to top of file
      osmStream << "<node id=\"" << currentWayID << i
                << "\" lat=\"" << myRoadGraph[*ei].roadSegmentGeometry.at(i).y()
                << "\" lon=\"" << myRoadGraph[*ei].roadSegmentGeometry.at(
                  i).x()  << "\">\n"; //must flip y-value for CE
      osmStream << "<tag k=\"ele\" v=\"" << myRoadGraph[*ei].roadSegmentGeometry.at(
                  i).z() << "\"/>\n";
      osmStream << "</node>\n";

      // store nd to print in way below
      wayStream << "<nd ref=\"" << currentWayID << i << "\"/>\n";
    }



    double width = myRoadGraph[*ei].roadSegmentWidth;

    // Map road width to highway type (can be changed, based on CE OSM import)
    if (width >= 11) {
      wayStream << "<tag k=\"highway\" v=\"primary\"/>\n";
      width = width -
              4; //Add space for sidewalks of width 2 (default on CE OSM Import)
    } else if (width >= 9) {
      wayStream << "<tag k=\"highway\" v=\"secondary\"/>\n";
      width = width -
              4; //Add space for sidewalks of width 2 (default on CE OSM Import)
    } else if (width >= 8) {
      wayStream << "<tag k=\"highway\" v=\"tertiary\"/>\n";
      width = width -
              4; //Add space for sidewalks of width 2 (default on CE OSM Import)
      wayStream << "<tag k=\"type\" v=\"MINOR\"/>\n"; //type defaults to MAJOR
    } else {
      wayStream << "<tag k=\"highway\" v=\"unknown\"/>";
      wayStream << "<tag k=\"type\" v=\"MINOR\"/>\n";
    }

    // write tag for road segment width
    wayStream << "<tag k=\"width\" v=\"" << width << "\"/>\n";


    wayStream << "</way>\n";

    currentWayID++;
  }

  wayText.close();

  //print ways
  wayText.open();
  osmStream << wayStream.readAll();
  wayText.close();

  //write file footer
  osmStream << "</osm>";
}//

bool compare2ndPartTupleEFO(const
                            std::pair<RoadGraph::out_roadGraphEdgeIter_BI, float> &i,
                            const std::pair<RoadGraph::out_roadGraphEdgeIter_BI, float> &j) {
  return (i.second < j.second);
}
bool compare2ndPartTupleEFI(const
                            std::pair<RoadGraph::in_roadGraphEdgeIter_BI, float> &i,
                            const std::pair<RoadGraph::in_roadGraphEdgeIter_BI, float> &j) {
  return (i.second < j.second);
}

void RoadGraph::fillInOutNumForEdges() {
  RoadGraph::roadGraphVertexIter_BI vi, vend;

  QVector2D referenceVector(0, 1);
  float angleRef = atan2(referenceVector.y(), referenceVector.x());
  QVector2D p0, p1;
  QTime timer;
  timer.start();

  for (boost::tie(vi, vend) = boost::vertices(myRoadGraph_BI); vi != vend; ++vi) {
    //printf("Vertex\n");
    /////////////////
    // IN LINKS
    std::vector<std::pair<RoadGraph::in_roadGraphEdgeIter_BI, float>> edgeAngleIn;
    RoadGraph::in_roadGraphEdgeIter_BI Iei, Iei_end;

    for (boost::tie(Iei, Iei_end) = boost::in_edges(*vi, myRoadGraph_BI);
         Iei != Iei_end; ++Iei) {
      //printf("  In Edge\n");
      if (myRoadGraph_BI[*Iei].roadSegmentGeometry.size() > 1) {
        p1 = myRoadGraph_BI[*Iei].roadSegmentGeometry[0].toVector2D();//note p1 p0 interchanged
        p0 = myRoadGraph_BI[*Iei].roadSegmentGeometry[1].toVector2D();
      } else {
        p1 = myRoadGraph_BI[boost::source(*Iei,
                                          myRoadGraph_BI)].pt.toVector2D();//note p1 p0 interchangedc
        p0 = myRoadGraph_BI[boost::target(*Iei, myRoadGraph_BI)].pt.toVector2D();
      }

      QVector2D edgeDir = (p1 - p0).normalized();// NOTE p1-p0
      //p1 = p0 + edgeDir*30.0f;//expand edge to make sure it is long enough

      float angle = angleRef - atan2(edgeDir.y(), edgeDir.x());

      if (angle < 0) {
        angle += 2 * M_PI;  //to positive
      }

      angle *= 57.2957795f;//to degrees
      angle = 360.0f - angle;
      //printf("  I Angle %f Vector %f %f\n",angle,edgeDir.x(),edgeDir.y());
      //edgeAngleOut.push_back(std::make_pair(std::make_pair(QVector3D(p0.x(), p0.y(),0), p1), angle));//z as width
      edgeAngleIn.push_back(std::make_pair(Iei, angle));
    }

    //sort
    if (edgeAngleIn.size() > 0) {
      std::sort(edgeAngleIn.begin(), edgeAngleIn.end(), compare2ndPartTupleEFI);

      for (int iN = 0; iN < edgeAngleIn.size(); iN++) {
        myRoadGraph_BI[*edgeAngleIn[iN].first].inNum = iN;
        myRoadGraph_BI[*edgeAngleIn[iN].first].inAngle = edgeAngleIn[iN].second;
        //printf(" I Num %d Angle %f\n", iN, edgeAngleIn[iN].second);
      }

    }

    /////////////////
    // OUT LINKS
    std::vector<std::pair<RoadGraph::out_roadGraphEdgeIter_BI, float>> edgeAngleOut;
    RoadGraph::out_roadGraphEdgeIter_BI Oei, Oei_end;

    for (boost::tie(Oei, Oei_end) = boost::out_edges(*vi, myRoadGraph_BI);
         Oei != Oei_end; ++Oei) {
      if (myRoadGraph_BI[*Oei].roadSegmentGeometry.size() > 1) {
        p0 = myRoadGraph_BI[*Oei].roadSegmentGeometry[0].toVector2D();
        p1 = myRoadGraph_BI[*Oei].roadSegmentGeometry[1].toVector2D();
      } else {
        p0 = myRoadGraph_BI[boost::source(*Oei, myRoadGraph_BI)].pt.toVector2D();
        p1 = myRoadGraph_BI[boost::target(*Oei, myRoadGraph_BI)].pt.toVector2D();

      }

      QVector2D edgeDir = (p1 - p0).normalized();// NOTE p1-p0
      //p1 = p0 + edgeDir*30.0f;//expand edge to make sure it is long enough
      float angle = angleRef - atan2(edgeDir.y(), edgeDir.x());

      if (angle < 0) {
        angle += 2 * M_PI;  //to positive
      }

      angle *= (180.0f / M_PI);//to degrees
      angle = 360.0f - angle;
      //edgeAngleOut.push_back(std::make_pair(std::make_pair(QVector3D(p0.x(), p0.y(),0), p1), angle));//z as width
      //printf("  O Angle %f Vector %f %f\n", angle, edgeDir.x(), edgeDir.y());
      edgeAngleOut.push_back(std::make_pair(Oei, angle));
    }

    //sort
    if (edgeAngleOut.size() > 0) {
      std::sort(edgeAngleOut.begin(), edgeAngleOut.end(), compare2ndPartTupleEFO);

      for (int oN = 0; oN < edgeAngleOut.size(); oN++) {
        myRoadGraph_BI[*edgeAngleOut[oN].first].outNum = oN;
        myRoadGraph_BI[*edgeAngleOut[oN].first].outAngle = edgeAngleOut[oN].second;
        //printf(" O Num %d Angle %f\n", oN, edgeAngleOut[oN].second);
      }

    }

  }

  printf("<<fillInOutNumForEdges in %d ms\n", timer.elapsed());
}//






}// LC
