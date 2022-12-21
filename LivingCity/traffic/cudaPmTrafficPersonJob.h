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
*		LC Project - Procedural Machine Traffic Person
*
*
*		@desc Class that generates procedually a traffic person
*		@author igaciad
*
************************************************************************************************/

#ifndef LC_CUDA_PM_TRAFFIC_PERSON_H
#define LC_CUDA_PM_TRAFFIC_PERSON_H

#include "../misctools/misctools.h"

#include <QtGlobal>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "cudaTrafficPerson.h"
#include "RoadGraph/roadGraph.h"

#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "../VBOPeopleJobInfoLayer.h"
#include <random>

namespace LC {
class CUDAPMTrafficPersonJob {

 public:
  CUDAPMTrafficPersonJob();
  ~CUDAPMTrafficPersonJob();

  cv::Mat *peopleDistribution;
  cv::Mat *jobDistribution;

  void randomPerson(int p, CUDATrafficPerson &person, QVector3D housePos3D,
                    QVector3D jobPos3D, float startTime, LC::RoadGraph::roadBGLGraph_BI &roadGraph);
  void sampleDistribution(int numberToSample, PeopleJobOneLayer &distribution,
                          std::vector<QVector2D> &samples, QString &name);
  void generateTrafficPersonJob(int numberPerGen,
                                std::vector<CUDATrafficPerson> &trafficPersonVec,
                                PeopleJobInfoLayers &simPeopleJobInfoLayersn,
                                LC::RoadGraph::roadBGLGraph_BI &roadGraph);

  void resetTrafficPersonJob(std::vector<CUDATrafficPerson> &trafficPersonVec);
  std::vector<ushort> backUpInitEdge;
};
}

#endif
