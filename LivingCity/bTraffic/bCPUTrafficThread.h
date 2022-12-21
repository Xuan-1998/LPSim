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
*		@desc Class that contains the traffic simulator
*		@author igaciad
************************************************************************************************/

#pragma once

#define BOOST_TT_HAS_OPERATOR_HPP_INCLUDED

#include "GL/glew.h"
#include "qobject.h"
#include <qthread.h>

#include "bPMTrafficPerson.h"
#include "../RoadGraph/roadGraph.h"
#include "bEdgeIntersectionData.h"


namespace LC {



class BCPUTrafficThread: public QThread {
  Q_OBJECT
 public:

  //~BCPUTrafficThread();

  // init data
  BCPUTrafficThread();
  void init(
    const int threadId,
    const int pMin, const int pMax,
    //const float cellSize,
    //const float deltaTime,
    const ushort maxWidthL,
    //uint mapToReadShift,
    //uint mapToWriteShift,
    SimulationSt &_simulationSt,
    BTrafficPeople &_people,
    BEdgesData &_edgesData,
    BIntersectionsData &_intersections,
    std::vector<unsigned long>(&_laneMapL)[2]
  );

  // Simulate

  void run();

  //signals:
  //void finished();
  //void threadFinish(int);
  //void msg(QString);
 private:
  int threadId;
  int pMin, pMax;
  //float cellSize;//these two too?
  //float deltaTime;
  ushort maxWidthL;//really
  //uint mapToReadShift;
  //uint mapToWriteShift;
  SimulationSt *simulationSt;
  BTrafficPeople *people;
  BEdgesData *edgesData;
  BIntersectionsData *intersections;
  unsigned long *laneMapL[2];
};


}

