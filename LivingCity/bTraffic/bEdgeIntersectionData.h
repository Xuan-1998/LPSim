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

#pragma once

#include <vector>

#define S_TOSTART	0x00
#define S_MOVE		0x01
#define S_PASSINT	0x02
#define	S_END	0xFF

#include "stdint.h"

#define ushort uint16_t
#define uint uint32_t
#define uchar uint8_t

namespace LC {

struct BEdgesDataCUDA {
  uchar *numLinesB;

  ushort *nextInters;
  uchar *nextIntersType;

  ushort *lengthC;//length in cells
  float *maxSpeedCpSec;//speed in cells per delta time


};

struct BEdgesData {
  std::vector<uchar> numLinesB;

  std::vector<ushort> nextInters;
  std::vector<uchar> nextIntersType;

  std::vector<ushort> lengthC;//length in cells
  std::vector<float> maxSpeedCpSec;//speed in cells per delta time

  void resize(int size) {
    numLinesB.resize(size);

    nextInters.resize(size);
    nextIntersType.resize(size);

    lengthC.resize(size);
    maxSpeedCpSec.resize(size);
  }

  void clear() {
    numLinesB.clear();
    nextInters.clear();
    nextIntersType.clear();
    lengthC.clear();
    maxSpeedCpSec.clear();
  }
};

/////////////////////////////////
// STOP

struct  BStopsData {
  std::vector<uchar>	state;
  std::vector<ushort> intersNumber;//number of intersect
  void resize(int size) {
    state.resize(size);
    intersNumber.resize(size);
  }
};

/////////////////////////////////
// TRAFFIC LIGHT
struct BTrafficLightData {
  std::vector<ushort> intersNumber;//number of intersect
  std::vector<uchar>	offSet;//time offset
  std::vector<ushort> phaseInd;
  std::vector<uchar>	numPhases;
  std::vector<uchar>	curPhase;

  std::vector<unsigned long> phases;
  std::vector<uchar>	 phaseTime;//note that it is *2
};

/////////////////////////////////
// INTERSECTION

struct BIntersectionsData {

  std::vector<unsigned long> req;
  std::vector<unsigned long> trafficLight;
  // common
  std::vector<uchar> numIn;
  std::vector<uchar> type;
  std::vector<float> nextEvent;


  void resize(int size) {
    req.resize(size);
    trafficLight.resize(size);

    numIn.resize(size);
    type.resize(size);
    nextEvent.resize(size);
  }//

  void resetReq() {
    memset((uchar *)(&req[0]), 0, req.size()*sizeof(unsigned long));
  }//

  void resetTraff() {
    memset((uchar *)(&trafficLight[0]), 0x00,
           trafficLight.size()*sizeof(unsigned long));
  }//
};
}
