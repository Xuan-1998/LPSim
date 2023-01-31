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
*		LC Project - CUDA Traffic Person
*
*
*		@desc Class that contains the info of a person
*		@author igaciad
*
************************************************************************************************/

#ifndef LC_CUDA_TRAFFIC_PERSON_H
#define LC_CUDA_TRAFFIC_PERSON_H



//#include "misctools.h"

namespace LC {

  const unsigned int kMaxPersonPath = 140;

struct CUDATrafficPerson {
  unsigned short init_intersection;
  unsigned short end_intersection;
  float time_departure;

  unsigned short active;//0 inactive 1 active 2 finished
  //unsigned short curEdgeLane;// edge number
  unsigned short numOfLaneInEdge;//number of lane in that edge

  float posInLaneM;

  //////////////////////////
  // current edge (from edgeData)
  unsigned short edgeNumLanes;  //total number of lanes in that edge
  unsigned int edgeNextInters;
  float length;
  float maxSpeedMperSec;
  /////////////////////////
  // to check next edge
  //unsigned short nextEdge;
  unsigned short nextEdgeNumLanes;
  unsigned short nextEdgeNextInters;
  float nextEdgeLength;
  float nextEdgemaxSpeedMperSec;
  ///////////////////////////


  unsigned int currPathEdge;
  unsigned int personPath[kMaxPersonPath];//change CUDATrafficPersonShortestPath::calculateSeveralPeopleRoute
  // data
  unsigned short num_steps;
  unsigned int color;
  float gas;
  // IDM
  float v;//current velocity
  float a;//acceleration
  float b;//break
  float T;// Time heading

  // lane changing
  unsigned short LC_initOKLanes;
  unsigned short LC_endOKLanes;
  unsigned short LC_stateofLaneChanging;

  int isInIntersection;
};

}

#endif
