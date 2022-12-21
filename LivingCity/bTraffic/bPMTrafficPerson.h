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
*		@desc Class that generates procedually a traffic person
*		@author igaciad
************************************************************************************************/

#pragma once

#include "bTrafficPeople.h"
#include "../VBOPeopleJobInfoLayer.h"
#include "../RoadGraph/roadGraph.h"
#include <vector>

namespace LC {
class BPMTrafficPerson {

 public:
  //BPMTrafficPerson();
  //~BPMTrafficPerson();

  // generate with layer info
   static void generateRandomTrafficPeople(float start_time, float end_time,
                                    int numberPeople,
                                    PeopleJobInfoLayers &peopleJobInfoLayers, RoadGraph::roadBGLGraph_BI &roadGraph,
                                    BTrafficPeople &people);

   // generate from Dynameq
   static void generateDynameqTrafficPeople(RoadGraph::roadBGLGraph_BI &roadGraph,
     BTrafficPeople &people);

  // generate from b2018
   static void generateB2018TrafficPeople(float start_time, float end_time, 
     RoadGraph::roadBGLGraph_BI &roadGraph,
                                    BTrafficPeople &people);

  static void resetTrafficPerson(BTrafficPeople &trafficPersonVec);

  static void randomPerson(int p, BTrafficPeople &people, QVector3D housePos3D,
                           QVector3D jobPos3D, float startTime, LC::RoadGraph::roadBGLGraph_BI &roadGraph);
  static void randomPerson(int p, BTrafficPeople &people, uint srcvertex,
                           uint tgtvertex, float startTime, int carType = 0);
 private:
  static void sampleDistribution(int numberToSample,
                                 PeopleJobOneLayer &distribution, std::vector<QVector2D> &samples,
                                 QString &name);
  static void generateTrafficPersonFromStream(float startTime, float endTime,
      QFile &file, RoadGraph::roadBGLGraph_BI &roadGraph, BTrafficPeople &people,
      float factor = 1.0f);
};
}
