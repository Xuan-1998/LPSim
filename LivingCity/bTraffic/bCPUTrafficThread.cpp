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

#define BOOST_TT_HAS_OPERATOR_HPP_INCLUDED

#include "bCPUTrafficThread.h"

#include "bPMTrafficPerson.h"
#include "bTrafficLaneMap.h"

#include "bTrafficDijkstra.h"
#include "bTrafficJohnson.h"

#include "bCPUTrafficThread.h"

#include "bTrafficConstants.h"

#include <qmutex.h> //for atomic add
#include <qthread.h>

#define DEBUG_TRAFFIC 0
#define DEBUG_SIMULATOR 0
#define DEBUG_1CAR 0



namespace LC {

//bool calculatePollution=true;

//BCPUTrafficThread::BCPUTrafficThread(){
/*initialized=false;
simRoadGraph=0;
state = 0;*/
//}//

//BCPUTrafficThread::~BCPUTrafficThread(){
//}//

/////////////////////////////////////////////////////////////////////
// DATA
////////////////////////////////////////////////////////////////////

BCPUTrafficThread::BCPUTrafficThread() {

}//
void BCPUTrafficThread::init(
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
) {
  this->threadId = threadId;
  this->pMin = pMin;
  this->pMax = pMax;
  //this->cellSize = cellSize;
  //this->deltaTime = deltaTime;
  this->maxWidthL = maxWidthL;

  this->simulationSt = &_simulationSt;
  this->people = &_people;
  this->edgesData = &_edgesData;
  this->intersections = &_intersections;
  this->laneMapL[0] = &_laneMapL[0][0];
  this->laneMapL[1] = &_laneMapL[1][0];
  //this->mapToReadShift = mapToReadShift;
  //this->mapToWriteShift = mapToWriteShift;
  //this->people = _people;
  //this->laneMap = _laneMap;

}//

/////////////////////////////////////////////////////////////////////
// SIMULATE
////////////////////////////////////////////////////////////////////

using namespace std;

void onePersonFinished(SimulationSt &simulationSt) {
  QMutex sum;
  sum.lock();
  simulationSt.numPeopleFinished++;
  sum.unlock();
}//

template <typename T>
inline T mRound(T num, T round) {
  return (num + round - 1) & (~(round - 1));
}//
/*inline uchar firstDifV(unsigned long laneL){
	if (laneL & 0xFF00000000000000)return laneL >> 56;
	if (laneL & 0x00FF000000000000)return laneL >> 48;
	if (laneL & 0x0000FF0000000000)return laneL >> 40;
	if (laneL & 0x000000FF00000000)return laneL >> 32;
	if (laneL & 0x00000000FF000000)return laneL >> 24;
	if (laneL & 0x0000000000FF0000)return laneL >> 16;
	if (laneL & 0x000000000000FF00)return laneL >> 8;
	return laneL;
}//*/

inline uchar firstDifByte(unsigned long laneL) {
  /*unsigned long index;
  //no necessary to check if all 1s (already checked before)
  _BitScanForward64(&index, !laneL);//Search the mask data from least significant bit (LSB) to the most significant bit (MSB) for a set bit (1).
  return uchar(index / 8);//return the byte*/
  if ((laneL & 0x00000000000000FF) != 0x00000000000000FF) {
    return 0;
  }

  if ((laneL & 0x000000000000FF00) != 0x000000000000FF00) {
    return 1;
  }

  if ((laneL & 0x0000000000FF0000) != 0x0000000000FF0000) {
    return 2;
  }

  if ((laneL & 0x00000000FF000000) != 0x00000000FF000000) {
    return 3;
  }

  if ((laneL & 0x000000FF00000000) != 0x000000FF00000000) {
    return 4;
  }

  if ((laneL & 0x0000FF0000000000) != 0x0000FF0000000000) {
    return 5;
  }

  if ((laneL & 0x00FF000000000000) != 0x00FF000000000000) {
    return 6;
  }

  //if ((laneL & 0xFF00000000000000) != 0xFF00000000000000)return 7;
  return 7;
}//

void simulateOnePersonCPU(
  uint p,
  //const float cellSize,
  //const float deltaTime,
  SimulationSt &simulationSt,
  const ushort maxWidthL,
  BTrafficPeople &people,
  BEdgesData &edgesData,
  unsigned long *laneMapR,//,
  unsigned long *laneMapW,
  BIntersectionsData &intersec
  //std::vector<uchar>& trafficLights
) {
  //if(DEBUG_TRAFFIC==1)printf("currentTime %f   0 Person: %d State %d Time Dep %f\n",currentTime,p,trafficPersonVec[p].active, trafficPersonVec[p].time_departure);

  uchar active = people.active[p];
  uchar *laneMapRC = (uchar *)laneMapR;
  uchar *laneMapWC = (uchar *)laneMapW;

  ///////////////////////////////
  //0. check if finished
  if (active == S_END) {
    return;
  }

  ///////////////////////////////
  //1. check if person should still wait or should start
  if (active == S_TOSTART) {

    //printf("  1. Person: %d active==0\n",p);
    if (people.time_departure[p] > simulationSt.currentTime) { //wait
      //1.1 just continue waiting
      //printf("   1.1 Person: %d wait\n",p);
      return;
    } else { //start
      //1.2 find first edge
      people.currIndEdge[p] = people.indTo1stEdge[p];//pointer to 1st edge in nextEdge
      ushort firstEdge = people.nextEdge[people.currIndEdge[p]];
      ushort cEdgeLengthC =
        edgesData.lengthC[firstEdge];// ceil(trafficPersonVec[p].length / cellSize);

      if (firstEdge == 0xFFFF ||
          cEdgeLengthC < 8) { //already in destination or not edge
        people.active[p] = S_END;
        onePersonFinished(simulationSt);
        return;
      }

      if (DEBUG_TRAFFIC == 1 || (p == 0 && DEBUG_1CAR)) {
        printf("   1.2 Person: %d TRY put in first edge\n", p, firstEdge);
      }

      //1.4 try to place it in middle of edge

      //ushort numOfCellsL = ((numOfCells +7) & ~7u)/8;//round up
      /*LONG64 laneL;
      ushort b;
      //ushort lN = edgesData[firstEdge].numLinesB - 1;//just right LANE !!!!!!!
      if (DEBUG_TRAFFIC == 1 || (p == 0 && DEBUG_1CAR)) printf("   1.2 Person: %d TRY put in first edge--> %u numOfCells %u\n", p, firstEdge, numOfCells);
      ushort numOfCellsL = ((numOfCells) / 8);//not try last
      for (ushort L = (ushort)numOfCellsL*0.25f; L<numOfCellsL; L++){//(~half of road)

        laneL = laneMapR[maxWidthL*(firstEdge + 0) + L];//get byte of edge (proper line)

        b = std::min<ushort>(
                (laneL & 0x00000000FFFFFFFF == 0x00000000FFFFFFFF) * 1,
                (laneL & 0xFFFFFFFF00000000 == 0xFFFFFFFF00000000) * 5);

        if (b == 0)
                continue;
        people.posInLaneC[p] = b+L*8;//cells
        laneMapWC[(maxWidthL*(firstEdge + 0) + L) * 8 + b] = 0;// (uchar)(trafficPersonVec[p].v * 3);//speed in m/s *3 (to keep more precision
        laneMapRC[(maxWidthL*(firstEdge + 0) + L) * 8 + b] = 0;// also in read to avoid put two in the same place
        people.v[p] = 0;
        people.active[p] = 1;
        people.laneInEdge[p] = 0;//0 is right line
        if (DEBUG_TRAFFIC == 1 || (p == 0 && DEBUG_1CAR)) printf("   1.2 Person: %d PUT in first edge--> %u numOfCells %u\n", p, firstEdge, numOfCells);
        return;//placed
      }
      return;//no posible to place now*/
      /*uchar laneChar;
      int count = 0;
      for (ushort b = 0; b<numOfCells; b++){
        laneChar = laneMapRC[(maxWidthL*(firstEdge + 0)) * 8 + b];
        if (laneChar != 0xFF){
                count = 0;
                continue;
        }else{
                count++;
        }
        if (count >= 6){
                people.posInLaneC[p] = b-4;//cells
                laneMapWC[(maxWidthL*(firstEdge + 0)) * 8 + b] = 1;// (uchar)(trafficPersonVec[p].v * 3);//speed in m/s *3 (to keep more precision
                laneMapRC[(maxWidthL*(firstEdge + 0)) * 8 + b] = 1;// also in read to avoid put two in the same place
                people.v[p] = 0;
                people.active[p] = S_MOVE;
                people.laneInEdge[p] = 0;//0 is right line
                if (DEBUG_TRAFFIC == 1 || (p == 0 && DEBUG_1CAR)) printf("   1.2 Person: %d PUT in first edge--> %u numOfCells %u\n", p, firstEdge, numOfCells);
                return;//placed
        }
      }*/
      ushort cEdgeLengthCL = ((cEdgeLengthC + 7) & ~7u) / 8;//round up
      ushort LtoCheck = std::min<ushort>(cEdgeLengthCL, 1);
      unsigned long laneL = laneMapR[maxWidthL * (firstEdge + 0) +
                                               LtoCheck]; //get byte of edge (proper line)

      if ((laneL & 0xFFFFFFFF00000000) == 0xFFFFFFFF00000000) {
        laneMapWC[(maxWidthL * (firstEdge + 0) + LtoCheck) * 8 + 5] =
          0x01; // (uchar)(trafficPersonVec[p].v * 3);//speed in m/s *3 (to keep more precision
        laneMapRC[(maxWidthL * (firstEdge + 0) + LtoCheck) * 8 + 5] =
          0x01; // also in read to avoid put two in the same place
        people.posInLaneC[p] = LtoCheck * 8 + 5;
        people.v[p] = 0;
        people.active[p] = S_MOVE;
        people.laneInEdge[p] = 0;//0 is right line
      }

      return;
    }
  }

  ///////////////////////////////
  //2. it is moving
  ushort cEdge = people.nextEdge[people.currIndEdge[p]];//curent edge
  ushort cEdgeLengthC = edgesData.lengthC[cEdge];
  float v = people.v[p];//velocity
  uint lN = people.laneInEdge[p];//lane
  float posInLaneC = people.posInLaneC[p];//position in lane
  ushort posInLaneCS = ushort(posInLaneC);
  //////////////////////////////////////////////////////
  // rought formula to check distance
  //float dToCheck = (v*v) / (2.0f*0.8f*gC);//calculate cells to check
  /*unsigned long laneL;
  ushort dToCheckL = (ushort((v*v) / (2.0f*0.8f*gC) + s_0C) / 8);//calculate cells to check (break+minDist) hyperphysics.phy-astr.gsu.edu/hbase/crstp.html#c2
  dToCheckL = dToCheckL > 1 ? dToCheckL : 2;//at least 2 long
  if ((p == 0 && DEBUG_1CAR)) printf("\n   2 Person: %d Moving v %f--> posInLaneC %f cEdgeLengthC %u (round) %u dToCheckL %u\n", p, v, posInLaneC, cEdgeLengthC, mRound<ushort>(cEdgeLengthC, 8) / 8, dToCheckL);
  for (ushort L = ushort(posInLaneCS / 8); L<(mRound<ushort>(cEdgeLengthC, 8) / 8) && dToCheckL>0; L++, dToCheckL--){//for(posInCL to end
        laneL = laneMapR[maxWidthL*(cEdge + lN) + L];//get byte of edge (proper line)
        if ((p == 0 && DEBUG_1CAR)) printf("    **  laneL %#018llx Pos %u\n", laneL, (maxWidthL*(cEdge + lN) + L)*8);
        if (L == ushort(posInLaneCS / 8)){//not take into account this car
                laneL |= (0xFFFFFFFFFFFFFFFF >> 8 * int(7 - (posInLaneCS - L * 8)));
                if ((p == 0 && DEBUG_1CAR)) printf("    ** Mask   %#018llx %u\n", (0xFFFFFFFFFFFFFFFF >> 8 * int(7 - (posInLaneCS - L * 8))), (posInLaneCS - L * 8));
        }
        if (laneL == 0xFFFFFFFFFFFFFFFF)//no cars
                continue;
        uchar firstDif = firstDifByte(laneL);
        vFront = uchar(laneL >> (firstDif*8)) / 3.0f;
        sFront = (L * 8 + firstDif) - posInLaneCS;//
        if ((p == 0 && DEBUG_1CAR)){
                printf("\n   found L p %d\n", p);
                printf("    **  laneL %#018llx Pos %u\n", laneMapR[maxWidthL*(cEdge + lN) + L], (maxWidthL*(cEdge + lN) + L) * 8);
                printf("    ** Mask   %#018llx %u\n", (0xFFFFFFFFFFFFFFFF >> 8 * int(7 - (posInLaneCS - L * 8))), (posInLaneCS - L * 8));
                printf("    ** firstDif %u vF %f sF %u\n", firstDif, vFront, sFront);
        }
        break;
  }*/
  ///////////////
  // FIND FOLLOWING CAR
  float vFront = FLT_MAX;
  float sFront = FLT_MAX;
  bool carFound = false;
  ushort bytesToCheck = std::min<ushort>(cEdgeLengthC,
                                         std::min<ushort>(((v * v) / (2.0f * 0.8f * gC)) + s_0C * 2,
                                             v * 2.0f + s_0C * 2)); //(v*v) / (2.0f*0.8f*gC)
  uchar laneChar;

  for (ushort b = posInLaneCS + 1; b < cEdgeLengthC; b++) {
    laneChar = laneMapRC[(maxWidthL * (cEdge + lN)) * 8 + b];

    if (laneChar != 0xFF) {
      vFront = laneChar / 3.0f;//laneChar is in 3*ms (to save space in array)
      sFront = b - posInLaneC;//c
      carFound = true;

      if (DEBUG_TRAFFIC == 1) {
        printf("*%d***   found\n", p);
      }

      //printf("    **  laneL %#018llx Pos %u\n", laneMapR[maxWidthL*(cEdge + lN) + b/8], (maxWidthL*(cEdge + lN)) * 8+b);
      break;
    }
  }

  //printf("****   vFront %f-->%f sFront %f-->%f\n", vFront, vFrontT, sFront,sFrontT);
  if (DEBUG_TRAFFIC == 1 || (p == 0 && DEBUG_1CAR)) {
    printf("****C   vFront %f sFront %u\n", vFront, sFront);
  }

  //
  //////////////////////////////////////////////////////////
  ///////////////////////////
  // TRAFFIC LIGHT
  if (DEBUG_TRAFFIC == 1)
    printf("p %d pos before traffic light %d checking %d (car nF %d or fC %d)\n", p,
           posInLaneCS < ushort(cEdgeLengthC - intersClearanceC),
           ((posInLaneCS + bytesToCheck) > ushort(cEdgeLengthC - intersClearanceC)),
           carFound == false,
           ((carFound == true) &&
            (sFront > (cEdgeLengthC - intersClearanceC - posInLaneC))));

  if ((active == S_MOVE) && //car is moving and not passed intersection
      (posInLaneCS < ushort(cEdgeLengthC - intersClearanceC)) && //before traffic
      ((posInLaneCS + bytesToCheck) > ushort(cEdgeLengthC - intersClearanceC)) &&
      //and checking traffic light
      ((carFound == false) || ((carFound == true) &&
                               (sFront > (cEdgeLengthC - intersClearanceC -
                                          posInLaneC))))//car in front farther than traffic light
     ) { //((carFound == false) ||	(carFound==true))) ////not found

    uint currIndEdge = people.currIndEdge[p];
    uchar exitN = people.edgeExitOut[currIndEdge];
    uchar in = people.edgeExitIn[currIndEdge];

    if (exitN != 0xFF) {//not intersection

      ushort nexInters = edgesData.nextInters[cEdge];
      uchar typeInters = intersec.type[nexInters];

      bool stop = false;
      bool req = true;

      if (typeInters == 0) {
        if (v > 0) {//moving-->STOP
          if (DEBUG_TRAFFIC == 1) {
            printf("Moving--> STOP\n");
          }

          stop = true;
          req = false;
        }
      }

      if (stop == false) {
        // check traffic light
        uchar light = uchar((intersec.trafficLight[nexInters] >> (in * 8)));

        if (((light >> exitN) & 0x01) == 0) {//red traffic light
          stop = true;

          if (DEBUG_TRAFFIC == 1) {
            printf("Red traffic light--> STOP\n");
          }
        } else {//green
          if (typeInters == 0) {//STOP Green (pass and not look back)
            if (DEBUG_TRAFFIC == 1) {
              printf("++++++++++ GREEEN--> PASS\n");
            }

            req = false;
            people.active[p] = S_PASSINT;
          }
        }
      }

      if (stop == true) {
        float distanceToLight = (float)((cEdgeLengthC - intersClearanceC) - posInLaneC);
        vFront = 0;//"Red"
        sFront = distanceToLight +
                 2.0f;//+2 to avoid gap to intersection (otherwise s0=car*1.5 instead of car)
      }

      if (req == true) {
        if (DEBUG_TRAFFIC == 1) {
          printf("ABOUT TO REQUEST\n");
        }

        ((uchar *)(&intersec.req[0]))[nexInters * 8 + in] |= uchar(1 << exitN);

        if (DEBUG_TRAFFIC == 1) {
          printf("REQUEST---> %016llX\n", intersec.req[nexInters]);
        }
      }

      if (DEBUG_TRAFFIC == 1 || (p == 0 && DEBUG_1CAR)) {
        printf("****T   vFront %f sFront %f (%u-%f) \n", vFront, sFront, cEdgeLengthC,
               posInLaneC);
      }
    }
  }

  float thirdTerm = 0.0f;

  if (v >= vFront) { //car in front and slower than us
    // 2.1.2 calculate dv_dt
    //float s_star;
    //s_star = s_0 + std::max(0.0f, (v*people.T[p] + (v*(v-vFront)) / (2 * std::sqrt(people.a[p]*people.b[p]))));
    //thirdTerm = std::pow(((s_star) / (sFront)), 2);
    thirdTerm = pow(((s_0 + max(0.0f,
                                (v * people.T[p] + (v * (v - vFront)) / (2 * sqrt(people.a[p] *
                                    people.b[p]))))) / (sFront)), 2);
  }

  float dv_dt = people.a[p] * (1.0f - std::pow((v /
                               edgesData.maxSpeedCpSec[cEdge]), 4.0f) - thirdTerm);
  float numCToMove = max(0.0f,
                         v * deltaTime + 0.5f * (dv_dt) * deltaTime * deltaTime);

  v = max(v + dv_dt * deltaTime, 0.0f);

  if (DEBUG_TRAFFIC == 1 || (p == 0 && DEBUG_1CAR)) {
    printf("   p[%d] v %.4f--> v %.4f (max %.4f)|| dv_dt %f numCToMove %f third %f\n",
           p, people.v[p], v, edgesData.maxSpeedCpSec[cEdge], dv_dt, numCToMove,
           thirdTerm);
  }

  people.v[p] = v;

  //2.1 was not really moving
  if (v == 0) {
    laneMapWC[(maxWidthL * (cEdge + lN)) * 8 + uint(posInLaneC)] = 0;

    //laneMapC[(simulationSt.mapToReadShiftL + maxWidthL*(cEdge + lN)) * 8 + uint(posInLaneC)] = 0;
    //if (DEBUG_TRAFFIC == 1 || (p == 0 && DEBUG_1CAR))
    if (DEBUG_TRAFFIC == 1) {
      printf("  no really moving\n");
    }

    return;
  }

  //2.2 move
  bool goNextEdge = false;
  posInLaneC += numCToMove;//new possition

  if (posInLaneC > cEdgeLengthC) { //reach intersection
    posInLaneC -= cEdgeLengthC;
    goNextEdge = true;
  }

  // move next edge
  if (goNextEdge == true) {
    people.currIndEdge[p]++;//move next edge

    if (people.nextEdge[people.currIndEdge[p]] == 0xFFFF) {
      //if (DEBUG_TRAFFIC == 1)
      if (DEBUG_TRAFFIC == 1) {
        printf("    2.2 Person: %d FINISHED\n", p);
      }

      people.active[p] = S_END;//finished
      onePersonFinished(simulationSt);
      return;
    }

    if (DEBUG_TRAFFIC == 1) {
      printf("    2.2 Person: %d new edge %u\n", p, cEdge);
    }

    people.active[p] = S_MOVE;//reset state
  }

  laneMapWC[(maxWidthL * (cEdge + lN)) * 8 + uint(posInLaneC)] = uchar((
        v * 3 + 0.5f));//to ms triple

  //laneMapC[(simulationSt.mapToReadShiftL + maxWidthL*(cEdge + lN)) * 8 + uint(posInLaneC)] = uchar(v * 3 * vCT2MS);//to ms triple
  if (DEBUG_TRAFFIC == 1 || (p == 0 && DEBUG_1CAR)) {
    printf("     Pos to write %u--> v %u\n",
           (maxWidthL * (cEdge + lN)) * 8 + uint(posInLaneC), uchar(v * 3 * vCT2MS));
  }

  if (DEBUG_TRAFFIC == 1 || (p == 0 && DEBUG_1CAR)) {
    printf("     maxW %u cEdge %u lN %u posInLaneC f %f\n", maxWidthL, cEdge, lN,
           posInLaneC);
  }

  people.posInLaneC[p] =
    posInLaneC;//position in lane (we assume that it is long enought to hold it)

  /*//for (int lN = 0; lN < edgesData[cEdge].numLinesB; lN++) {
        printf("SAV ");
        for (ushort sL = 0; sL < cEdgeLengthC / 8 + 1; sL++) {
                printf(" ");
                printf("%016llX", laneMapW[(maxWidthL*(cEdge + lN)) + sL]);
                //printf("%016llX", laneMapW[sL]);
        }
        printf("\n");
  //}
        printf("lineShift %d cEd %d shift %d\n", (maxWidthL*(cEdge + lN)), cEdge, (cEdge + lN));
        */
}//

void BCPUTrafficThread::run() {
  int p;
  //printf(">>[%d] start simulate %d %d\n",threadId,pMin,pMax);
  //printf(">>[%d] address %d %d\n", threadId, simulationSt.mapToReadShiftL, simulationSt.mapToWriteShiftL);
  QTime timer;
  timer.start();

  for (p = pMin; p < pMax; p++) {
    //if (p==3583)
    simulateOnePersonCPU(p,/* cellSize, deltaTime,*/ *simulationSt, maxWidthL,
                         *people, *edgesData, laneMapL[simulationSt->cArray],
                         laneMapL[!simulationSt->cArray], *intersections);

    //if(p==2)break;// !!! REMOVE//8
    //break;
  }

  //printf(">> SImulate[%d] %d in %d ms\n",threadId,pMax-pMin,timer.elapsed());
  //printf(">>[%d] end\n",threadId);
  //emit msg("**end: "+QString::number(threadId));
  //this->finished();
  //emit finished();
  //emit threadFinish(threadId);
  simulationSt->threadFinished[threadId] = 1;
  //printf(">> th %d fin %d\n", threadId, simulationSt->threadFinished[threadId]);
  //printf(".");
  //QThread::quit();
  //printf("SImulate[%d] %d in %d ms\n", threadId, pMax - pMin, timer.elapsed());
  //printf(">>end2\n");
  return;
}//


}
