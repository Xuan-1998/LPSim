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

#include "bTrafficSimulator.h"


#include "VBORenderManager.h"

#include "../global.h"
#include "../LC_GLWidget3D.h"
#include "../LC_UrbanMain.h"

#include "bPMTrafficPerson.h"
#include "bTrafficLaneMap.h"
#include "bTrafficIntersection.h"

#include "bTrafficDijkstra.h"
#include "bTrafficJohnson.h"

#include "bTrafficConstants.h"

#include "bCUDA_trafficSimulator.h"

#define DEBUG_TRAFFIC 0
#define DEBUG_SIMULATOR 0

namespace LC {

BTrafficSimulator::BTrafficSimulator() {
  initialized = false;
  simRoadGraph = 0;
  state = 0;
}//

BTrafficSimulator::~BTrafficSimulator() {
}//

/////////////////////////////////////////////////////////////////////
// GENERATE
////////////////////////////////////////////////////////////////////

void BTrafficSimulator::init(RoadGraph *roadGraph,
                             LCUrbanMain *_clientMain) {
  //if (simRoadGraph == 0)
  //	simRoadGraph = new RoadGraph(*roadGraph);
  simRoadGraph = roadGraph;//no copy
  clientMain = _clientMain;
  initialized = true;
}//

void BTrafficSimulator::createRandomPeople(
  float startTime, float endTime,
  int _numberPeople,
  PeopleJobInfoLayers &peopleJobInfoLayers) {
  if (initialized == false) {
    printf("Error: initSimulator was not called\n");
    return;
  }

  people.clear();
  BPMTrafficPerson::generateRandomTrafficPeople(startTime, endTime, _numberPeople,
      peopleJobInfoLayers, simRoadGraph->myRoadGraph_BI, people);
}//

void BTrafficSimulator::createB2018People(float startTime, float endTime) {
  if (initialized == false) {
    printf("Error: initSimulator was not called\n");
    return;
  }

  people.clear();
  BPMTrafficPerson::generateB2018TrafficPeople(startTime, endTime,
      simRoadGraph->myRoadGraph_BI, people);
}//

void BTrafficSimulator::createLaneMap() { //
  if (initialized == false) {
    printf("Error: initSimulator was not called\n");
    return;
  }

  BTrafficLaneMap::createLaneMap(*simRoadGraph, laneMapL, edgesData,
                                 laneMapNumToEdgeDesc, edgeDescToLaneMapNum, maxWidthL);
}//

void BTrafficSimulator::createIntersections() {
  if (initialized == false) {
    printf("Error: initSimulator was not called\n");
    return;
  }

  // Add in out num for edges
  simRoadGraph->fillInOutNumForEdges();

  BTrafficIntersection::createIntersections(*simRoadGraph, intersec, stops);
}//

void BTrafficSimulator::generateCarDijstra() { //
  if (initialized == false) {
    printf("Error: initSimulator was not called\n");
    return;
  }

  routesChanged = true;
  BTrafficDijstra::generateRoutes(simRoadGraph->myRoadGraph_BI, people,
                                  edgeDescToLaneMapNum);// , people.nextEdge);//nextEdgeM);
}//

void BTrafficSimulator::generateCarJohnson() {
  if (initialized == false) {
    printf("Error: initSimulator was not called\n");
    return;
  }

  routesChanged = true;
  BTrafficJohnson::generateRoutes(simRoadGraph->myRoadGraph_BI, people,
                                  edgeDescToLaneMapNum, people.nextEdge);//nextEdgeM);
}//


/////////////////////////////////////////////////////////////////////
// SIMULATE
////////////////////////////////////////////////////////////////////
void BTrafficSimulator::tFinish() {
  printf("tFinish\n");
}//

void BTrafficSimulator::msgString(QString msg) {
  printf("THREAD: %s\n", msg.toLatin1().constData());
}

void BTrafficSimulator::threadFinish(int threadId) {
  printf("threadFinish reached %d\n", threadId);
  //QMutex sum;
  //sum.lock();
  numThreadsFinished++;
  //threads[threadId]->quit();
  //printf("threadFinished\n");
  //if(numThreadsFinished==numThreads){
  //	mutex.unlock();
  //}
  //sum.unlock();
}//

void BTrafficSimulator::simulateGPU(float _startTime, float _endTime) {
  //printf("Simulate GPU\n");
  //bExampleCUDA();
  //return;

  float startTime = _startTime * 3600.0f;
  float endTime = _endTime * 3600.0f;
  state = 1;
  clientMain->ui.b18ProgressBar->show();
  // init data structures
  //////////////////////////////////////////////////////////////////
  int numPeople = people.numPeople;
  float currentTime = 24.0f * 3600.0f;

  for (int p = 0; p < people.numPeople; p++) {
    if (currentTime > people.time_departure[p]) {
      currentTime = people.time_departure[p];
    }
  }

  // round time to be delta divisible
  int numInt = currentTime / deltaTime;//floor
  currentTime = numInt * deltaTime; //round
  printf("start time %f actual start time %f\n", startTime, currentTime);

  // start as early as starting time
  if (currentTime < startTime) { //as early as the starting time
    currentTime = startTime;
  }

  people.currIndEdge.resize(
    numPeople);//to allocated in GPU (and be consistent with rest)
  people.posInLaneC.resize(numPeople);
  people.laneInEdge.resize(numPeople);
  //////////////////////////////////////////
  // init CUDA
  // FIXME
  // @ffernandez: re-enable CUDA

  bInitCUDA(maxWidthL, people, edgesData, laneMapL);// , intersections);

  QTime timer;
  timer.start();
  uint steps = 0;

  while (currentTime < endTime) {
    if (steps % int(60.0f / deltaTime) == 0) { //update each minute
      printf("**********\n*GPU*[%d] current time: %f --> end time %f  Sim time %d ms\n",
             steps, currentTime / 3600.0f, endTime / 3600.0f, timer.elapsed());
      QCoreApplication::processEvents(QEventLoop::AllEvents, 1000);
      timer.restart();
    }

    //

    // run threads
    bSimulateTrafficCUDA(currentTime, numPeople);

    //render
    if (clientMain->ui.b18RenderSimulationCheckBox->isChecked()) {
      bGetDataCUDA(people);//trafficPersonVec, trafficLights);

      if ((clientMain->ui.b18RenderStepSpinBox->value() == 0) ||
        (steps % clientMain->ui.b18RenderStepSpinBox->value() == 0)) { //each value steps
        //printf("render cuda\n");
        clientMain->glWidget3D->updateGL();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1000);
        QApplication::processEvents();
      }

      // update clock
      if (steps % int(60.0f / deltaTime) == 0) { //update each minute
        QString timeT;
        int timeH = currentTime / 3600.0f;
        int timeM = (currentTime - timeH * 3600.0f) / 60.0f;
        timeT.sprintf("%d:%02d", timeH, timeM);
        clientMain->ui.b18TimeLCD->display(timeT);
      }
    }//render

    // update time
    currentTime += deltaTime;
    steps++;
  }//time

  bFinishCUDA();
  state = 0;


}//

//////////////////////////////////////////////////////////////////////////////////////
// CPU
///////////////////////////////////////////////////////////////////////////////
#define DEBUG_INTERSEC_STOP 0
void simulateOneStopIntersectionCPU(uint sN, float currentTime,
                                    BIntersectionsData &intersec, BStopsData &stops) {
  const float deltaEvent = 20.0f;// / 3600.0f;

  ushort i = stops.intersNumber[sN];

  if (currentTime > intersec.nextEvent[i] && intersec.numIn[i] > 1) {
    if (DEBUG_INTERSEC_STOP == 1) {
      printf("Check Intersction %d Stop %d\n", i, sN);
    }

    unsigned long req = (intersec.req[i]);

    if (req == 0x00) { //none demand
      return;
    }

    if (DEBUG_INTERSEC_STOP == 1) {
      printf("i %u sN %u --> %016llX\n", i, sN, req);
    }

    for (int cI = 0; cI < intersec.numIn[i] + 1; cI++) {//one whole loop in all IN
      stops.state[sN] = (stops.state[sN] + 1) % intersec.numIn[i];//
      ushort nIN = stops.state[sN];
      unsigned long ff = 0xFF;
      unsigned long traff = req & (ff << (nIN * 8));

      if (traff != 0x00) {
        intersec.trafficLight[i] = traff;
        intersec.nextEvent[i] = currentTime + deltaEvent;
        return;
      }
    }

    printf("ERROR: this should not happen!!\n");
  }
}//

void simulateOneTrafficLightIntersectionCPU(uint sN, float currentTime,
    BIntersectionsData &intersec, BTrafficLightData &trafficLights) {
  const float deltaEvent = 20.0f;

  ushort i = trafficLights.intersNumber[sN];

  if (currentTime > intersec.nextEvent[i] && intersec.numIn[i] > 1) {
    // make offset
    if (intersec.nextEvent[i] == 0) {
      float offSet = float(trafficLights.offSet[sN]);
      intersec.nextEvent[i] = currentTime + offSet;
      return;
    }

    int phaseInd = 0;
    float phaseTime = trafficLights.phaseTime[phaseInd];
    intersec.nextEvent[i] = currentTime + phaseTime;
    return;

    if (DEBUG_INTERSEC_STOP == 1) {
      printf("Check Intersction %d Stop %d\n", i, sN);
    }

    unsigned long req = (intersec.req[i]);

    if (req == 0x00) { //none demand
      return;
    }

    if (DEBUG_INTERSEC_STOP == 1) {
      printf("i %u sN %u --> %016llX\n", i, sN, req);
    }

    for (int cI = 0; cI < intersec.numIn[i] + 1; cI++) {//one whole loop in all IN
      //stops.state[sN] = (stops.state[sN] + 1) % intersec.numIn[i];// !!!
      ushort nIN;// = stops.state[sN]; 201711
      unsigned long ff2 = 0xFF;
      unsigned long traff = req & (ff2 << (nIN * 8));

      if (traff != 0x00) {
        intersec.trafficLight[i] = traff;
        intersec.nextEvent[i] = currentTime + deltaEvent;
        return;
      }
    }

    printf("ERROR: this should not happen!!\n");
  }
}//

//
void BTrafficSimulator::simulateCPU(float _startTime, float _endTime) {

  printf("Simulate CPU\n");
  float startTime = _startTime * 3600.0f;
  float endTime = _endTime * 3600.0f;
  state = 1;
  clientMain->ui.b18ProgressBar->show();
  //
  int numPeople = people.numPeople;
  simulationSt.numPeopleFinished = 0;//for early finish
  people.currIndEdge.resize(numPeople);
  people.posInLaneC.resize(numPeople);
  people.laneInEdge.resize(numPeople);

  if (people.checkAllSameSize() == false) { //error
    exit(0);
  }

  printf("startTime %f endTime %f\n", startTime, endTime);
  //////////////////////////////////////////////////////////////////
  //float startTime = 6.5f*3600.0f;//7.0f
  //float endTime = 8.5f*3600.0f;//8.0f//10.0f

  float currentTime = 24.0f * 3600.0f;

  for (int p = 0; p < people.numPeople; p++) {
    if (currentTime > people.time_departure[p]) {
      currentTime = people.time_departure[p];
    }
  }

  // round time to be delta divisible
  int numInt = currentTime / deltaTime;//floor
  currentTime = numInt * deltaTime; //round
  printf("start time %f actual start time %f\n", startTime, currentTime);

  // start as early as starting time
  if (currentTime < startTime) { //as early as the starting time
    currentTime = startTime;
  }

  //uint stepsT = (currentTime - startTime) / deltaTime;//number of steps


  //////////////////////////////////////////
  // create threads and init data
  numThreads = std::min<int>(1,
                             QThread::idealThreadCount() - 1);//not to use the 100%
  simulationSt.threadFinished.resize(numThreads);
  printf("**numThreads %d\n", numThreads);
  //std::vector<QThread*> threads(numThreads);
  threads.resize(numThreads);
  int peoplePerThread = std::ceil(float(people.numPeople) / numThreads);
  int pMin, pMax;

  for (int i = 0; i < numThreads; i++) {
    threads[i] = new BCPUTrafficThread();
    threads[i]->setPriority(QThread::TimeCriticalPriority);//!!! MAYBE REMOVE
    // init simulator
    pMin = i * peoplePerThread;
    pMax = std::min<int>((i + 1) * peoplePerThread, people.numPeople);
    printf("**[%d] pMin %d pMax %d\n", i, pMin, pMax);

    //i, pMin, pMax, /*cellSize, deltaTime,*/ maxWidthL, &simulationSt, people, edgesData, laneMapL);

    threads[i]->init(i, pMin, pMax, /*cellSize, deltaTime,*/ maxWidthL,
                     simulationSt, people, edgesData, intersec, laneMapL);
    //threads[i]->moveToThread(threads[i]);
    //connect(sim[i], SIGNAL(msg(QString)), this, SLOT(msgString(QString)));

    //QObject::connect(threads[i], SIGNAL(finished()), threads[i], SLOT(deleteLater()));
    //QObject::connect(threads[i], SIGNAL(finished()), threads[i], SLOT(quit()));


    //threads[i]->start();
    printf("**[%d] created\n", i);
  }

  // check if some not to be simulated
  for (int p = 0; p < people.numPeople; p++) {
    if (people.time_departure[p] < endTime) {
      people.active[p] = 0;
    } else {
      people.active[p] = 2;  //not simulate
    }
  }

  bool readFirstMap = true;
  //uint mapToReadShift;
  //uint mapToWriteShift;
  //uint halfLaneMapL = laneMapL[.size() / 2;//it is multiple of 2
  memset((uchar *)&laneMapL[0].data()[0], 0xFF,
         laneMapL[0].size()*sizeof(unsigned long));//clean all
  memset((uchar *)&laneMapL[1].data()[0], 0xFF,
         laneMapL[1].size()*sizeof(unsigned long));//clean all
  //std::fill(laneMapL[0].begin(), laneMapL[0].end(), 0xFFFFFFFFFFFFFFFF);
  //std::fill(laneMapL[1].begin(), laneMapL[1].end(), 0xFFFFFFFFFFFFFFFF);
  printf("**START: current time: %f --> end time %f\n", currentTime, endTime);
  {
    uint32_t i = 0x01234567;
    printf("-->Little endian %d\n", (*((uint8_t *)(&i))) == 0x67);
    printf("-->unsigned long size %d\n", sizeof(unsigned long));
    printf("laneMapL size %d  maxWidthL %d\n", laneMapL[0].size(), maxWidthL);
    ushort cEdge = 0;
    ushort cEdgeLengthC = edgesData.lengthC[cEdge];
    uchar nL = edgesData.numLinesB[cEdge];
    uchar *laneMapRC = (uchar *)laneMapL[0].data();

    for (ushort sL = 0; sL < maxWidthL * 8; sL++) {
      if (sL % 8 == 0) {
        printf(" ");
      }

      printf("%hhX", laneMapRC[sL]);
    }

    printf("\n");

    for (int lN = 0; lN < nL; lN++) {
      for (ushort sL = 0; sL < cEdgeLengthC; sL++) {
        if (sL % 8 == 0) {
          printf(" ");
        }

        printf("%hhX", laneMapRC[(maxWidthL * (cEdge + nL)) * 8 + sL]);
      }

      printf("\n");
    }

    printf("\n");//extra
  }
  mutex.lock();//gather
  uint steps = 0;
  QTime timer;
  timer.start();

  while (currentTime < endTime) {
    if (steps % int(60.0f / deltaTime) == 0) { //update each minute
      printf("**********\n*CPU*[%d] current time: %f --> end time %f  Sim time %d ms\n",
             steps, currentTime / 3600.0f, endTime / 3600.0f, timer.elapsed());
      QCoreApplication::processEvents(QEventLoop::AllEvents, 1000);
      timer.restart();
    }

    // 1. clean memory
    simulationSt.currentTime = currentTime;
    simulationSt.cArray = readFirstMap;//read
    memset((uchar *)&laneMapL[!readFirstMap].data()[0], 0xFF,
           laneMapL[!readFirstMap].size()*sizeof(unsigned long));//clean write
    readFirstMap = !readFirstMap;//next iteration invert use


    ///////////////////////////////
    // Intersections
    intersec.resetTraff();//clear traffic lights

    for (int sN = 0; sN < stops.state.size(); sN++) {
      simulateOneStopIntersectionCPU(sN, currentTime, intersec, stops);
    }

    intersec.resetReq();//reset to be black for new simulation step
    ///////////////////////////////
    // PEOPLE
    // 2. run threads
    QTime timer2;
    timer2.start();
    numThreadsFinished = 0;

    for (int i = 0; i < numThreads; i++) {
      simulationSt.threadFinished[i] = 0;
      //printf("about to start th %d init %d\n", i, simulationSt.threadFinished[i]);
      //threads[i]->start();
      threads[i]->run();
    }

    /*while (true){
    //QCoreApplication::processEvents();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1000);
    bool finish = true;
    for (int i = 0; i < numThreads; i++){
    //printf("th %d fin %d\n", i, simulationSt.threadFinished[i]);
    if (simulationSt.threadFinished[i] == 0){
    finish = false;
    break;
    }
    }
    if (finish == true){
    break;
    }
    }*/

    //render //printstate
    if (clientMain->ui.b18RenderSimulationCheckBox->isChecked()) {
      {
        //printCurrentState();
      }

      if ((clientMain->ui.b18RenderStepSpinBox->value() == 0) ||
        (steps % clientMain->ui.b18RenderStepSpinBox->value() == 0)) { //each value steps
        //printf("render cuda\n");
        clientMain->glWidget3D->updateGL();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1000);
        QApplication::processEvents();
      }

      // update clock
      if (steps % int(60.0f / deltaTime) == 0) {//update each minute
        QString timeT;
        int timeH = currentTime / 3600.0f;
        int timeM = (currentTime - timeH * 3600.0f) / 60.0f;
        timeT.sprintf("%d:%02d", timeH, timeM);
        clientMain->ui.b18TimeLCD->display(timeT);
      }
    }//render

    //check if all finished
    if (steps % int(60.0f / deltaTime) == 0) { //each min
      /*int st0=0,st1=0,st2=0;
      for (int p = 0; p < people.numPeople; p++){
      st0 += people.active[p] == 0;
      st1 += people.active[p] == 1;
      st2 += people.active[p] == 2;
      }
      if (st2 == 9999 && st0 == 2){
      for (int p = 0; p < people.numPeople; p++){
      if (people.active[p] == 0)
      printf("start time [%d] %f\n",p,people.time_departure[p]);
      }
      }
      printf("st0 %d st1 %d st2 %d\n", st0,st1,st2);*/
      printf("Finished %d of %d --> %.0f%%\n\n", simulationSt.numPeopleFinished,
             people.numPeople, (100.0f * simulationSt.numPeopleFinished) / people.numPeople);

      if (simulationSt.numPeopleFinished == people.numPeople) { //everyone finished!
        printf("everyone finished\n");
        break;
      }
    }

    // update time
    currentTime += deltaTime;
    steps++;
  }

  // cleaup
  printf("clean up\n");

  for (int i = 0; i < numThreads; i++) {
    delete threads[i];
  }

  threads.clear();
  /////////////////////////////////////////////////////////////////
  clientMain->ui.b18ProgressBar->hide();
  state = 0;
}//

/////////////////////////////////////////////////////////////////////
// RENDER
////////////////////////////////////////////////////////////////////
void BTrafficSimulator::render(VBORenderManager &rendManager) {

  if (state == 0) {
    return;
  }

  rendManager.removeStaticGeometry("Sim_Points", false);
  rendManager.removeStaticGeometry("Sim_Box", false);
  /////////////////////////////
  // RENDER CARS
  bool renderMultiEdge = false;
  bool renderBoxes = false;

  /*// init render people
  if(cudaTrafficSimulatorRender.size()!=trafficPersonVec.size()&&trafficPersonVec.size()>0){
  cudaTrafficSimulatorRender.clear();
  cudaTrafficSimulatorRender.resize(trafficPersonVec.size());
  }

  if(cudaTrafficLightRender.size()!=trafficLights.size()&&trafficLights.size()>0){
  cudaTrafficLightRender.clear();
  cudaTrafficLightRender.resize(trafficLights.size());
  }*/
  //printf("BTrafficSimulator::render %d\n", people.numPeople);
  //////////////////////////
  // RENDER CAR AS BOXES
  if (renderBoxes == true && people.numPeople > 0) {


    std::vector<Vertex> carQuads;
    QVector3D carColor;
    int numPeople = people.numPeople;
    //carQuads.resize(4*numPeople);//quad per car many not active

    const float heightPoint = 5.0f;

    for (int p = 0; p < numPeople; p++) {

      //0. check if finished
      if (people.active[p] == S_TOSTART || people.active[p] == S_END) {
        continue;
      }

      carColor = QVector3D(1, 0,
                           0);//(trafficPersonVec[p].color>>24)&0xFF,(trafficPersonVec[p].color>>16)&0xFF,(trafficPersonVec[p].color>>8)&0xFF)/255.0f;
      //position
      ushort cEdge = people.nextEdge[people.currIndEdge[p]];

      if (laneMapNumToEdgeDesc.find(cEdge) == laneMapNumToEdgeDesc.end()) {
        printf("ERROR\n");//edge not found in map
        continue;
      }

      QVector3D p0, p1;
      //ushort posInLane =;
      float posInLaneM = people.posInLaneC[p] * cellSize;
      RoadGraph::roadGraphEdgeDesc_BI ei = laneMapNumToEdgeDesc[cEdge];

      // MULTI EDGE
      if (renderMultiEdge == true &&
          simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry.size() > 0) {

        for (int gN = 0;
             gN < simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry.size() - 1;
             gN++) { //note -1
          float currLe = (simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN] -
                          simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN + 1]).length();

          if (posInLaneM > currLe) { //next edge
            posInLaneM -= currLe;
            continue;
          }

          p0 = simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN];
          p1 = simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN + 1];
          break;
        }
      } else {
        // ONE EDGE
        p0 = simRoadGraph->myRoadGraph_BI[boost::source(ei,
                                          simRoadGraph->myRoadGraph_BI)].pt;
        p1 = simRoadGraph->myRoadGraph_BI[boost::target(ei,
                                          simRoadGraph->myRoadGraph_BI)].pt;
      }

      QVector3D dir = (p1 - p0).normalized();
      QVector3D per = (QVector3D::crossProduct(QVector3D(0, 0, 1.0f),
                       dir).normalized());
      //printf("trafficPersonVec[p].numOfLaneInEdge %u\n",trafficPersonVec[p].numOfLaneInEdge);
      float perShift = -0.5f * G::global().getFloat("roadLaneWidth") *
                       (1 + 2 * people.laneInEdge[p]);
      QVector3D v = p0 + dir * posInLaneM + perShift *
                    per; // center of the back of the car
      float widthHalf = 1.82f / 2.0f;
      float length = 4.12f;
      QVector3D v0 = v - widthHalf * per;
      QVector3D v1 = v - widthHalf * per + length * dir;
      QVector3D v2 = v + widthHalf * per + length * dir;
      QVector3D v3 = v + widthHalf * per;



      carQuads.push_back(Vertex(QVector3D(v0.x(), v0.y(), heightPoint), carColor,
                                QVector3D(), QVector3D()));
      carQuads.push_back(Vertex(QVector3D(v1.x(), v1.y(), heightPoint), carColor,
                                QVector3D(), QVector3D()));
      carQuads.push_back(Vertex(QVector3D(v2.x(), v2.y(), heightPoint), carColor,
                                QVector3D(), QVector3D()));
      carQuads.push_back(Vertex(QVector3D(v3.x(), v3.y(), heightPoint), carColor,
                                QVector3D(), QVector3D()));

    }//for people

    //printf("carQuads %d\n",carQuads.size());

    if (carQuads.size() > 0) {
      rendManager.addStaticGeometry("Sim_Box", carQuads, "", GL_QUADS,
                                    1 | mode_AdaptTerrain);
      glDisable(GL_DEPTH_TEST);
      rendManager.renderStaticGeometry("Sim_Box");
      glEnable(GL_DEPTH_TEST);
    }
  }//boxes


  //////////////////////////
  // RENDER CAR AS POINTS
  if (renderBoxes == false && people.numPeople > 0) {
    //rendManager.removeStaticGeometry("Sim_Points",false);
    const float heightPoint = 5.0f;
    std::vector<Vertex> carPoints;
    QVector3D pointColor(1.0f, 0.2f, 0.2f);
    int numPeople = people.numPeople;
    //carPoints.resize(numPeople);//many not active
    int activeCars = 0;

    for (int p = 0; p < numPeople; p++) {
      //printf(" 0. Person: %d\n",p);
      //0. check if finished
      if (people.active[p] != S_TOSTART && people.active[p] != S_END) {
        //printf("%x\n",trafficPersonVec[p].color);
        //pointColor = QVector3D((trafficPersonVec[p].color >> 24) & 0xFF, (trafficPersonVec[p].color >> 16) & 0xFF, (trafficPersonVec[p].color >> 8) & 0xFF) / 255.0f;
        ushort cEdge = people.nextEdge[people.currIndEdge[p]];

        //position
        if (laneMapNumToEdgeDesc.find(cEdge) == laneMapNumToEdgeDesc.end()) {
          printf("ERROR\n");//edge not found in map
          continue;
        }

        RoadGraph::roadGraphEdgeDesc_BI ei = laneMapNumToEdgeDesc[cEdge];
        // MULTI EDGE
        //ushort posInLane = ;
        float posInLaneM = people.posInLaneC[p] * cellSize;

        //printf("pos %f lN %u\n", people.posInLaneC[p], people.laneInEdge[p]);
        if (renderMultiEdge == true &&
            simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry.size() > 0) {
          for (int gN = 0;
               gN < simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry.size() - 1;
               gN++) { //note -1
            float currLe = (simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN] -
                            simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN + 1]).length();

            if (posInLaneM > currLe) { //next edge
              posInLaneM -= currLe;
              continue;
            }

            QVector3D p0 = simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN];
            QVector3D p1 = simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry[gN + 1];
            QVector3D dir = (p1 - p0).normalized();
            QVector3D per = (QVector3D::crossProduct(QVector3D(0, 0, 1.0f),
                             dir).normalized());
            //printf("trafficPersonVec[p].numOfLaneInEdge %u\n",trafficPersonVec[p].numOfLaneInEdge);
            float perShift = -0.5f * G::global().getFloat("roadLaneWidth") *
                             (1 + 2 * people.laneInEdge[p]);
            QVector3D v = p0 + dir * posInLaneM + perShift * per;
            //glVertex3f(v.x(),v.y(),heightPoint);
            carPoints[p] = Vertex(QVector3D(v.x(), v.y(), heightPoint), pointColor,
                                  QVector3D(), QVector3D());
            activeCars++;
            break;
          }
        } else {
          // ONE EDGE
          QVector3D p0 = simRoadGraph->myRoadGraph_BI[boost::source(ei,
                         simRoadGraph->myRoadGraph_BI)].pt;
          QVector3D p1 = simRoadGraph->myRoadGraph_BI[boost::target(ei,
                         simRoadGraph->myRoadGraph_BI)].pt;
          QVector3D dir = (p1 - p0).normalized();
          QVector3D per = (QVector3D::crossProduct(QVector3D(0, 0, 1.0f),
                           dir).normalized());
          //printf("trafficPersonVec[p].numOfLaneInEdge %u\n",trafficPersonVec[p].numOfLaneInEdge);
          float perShift = -0.5f * G::global().getFloat("roadLaneWidth") *
                           (1 + 2 * people.laneInEdge[p]);
          QVector3D v = p0 + dir * posInLaneM + perShift * per;
          //glVertex3f(v.x(),v.y(),heightPoint);
          carPoints.push_back(Vertex(QVector3D(v.x(), v.y(), heightPoint), pointColor,
                                     QVector3D(), QVector3D()));
          activeCars++;
        }
      }
    }

    //glEnd();
    if (carPoints.size() > 0) {
      rendManager.addStaticGeometry("Sim_Points", carPoints, "", GL_POINTS,
                                    1 | mode_AdaptTerrain);
      glDisable(GL_DEPTH_TEST);
      glPointSize(15.0f);
      rendManager.renderStaticGeometry("Sim_Points");
      //printf("carPoints %d\n",carPoints.size());
      glEnable(GL_DEPTH_TEST);
    }
  }

  /*/////////////////////////////
  // RENDER INTERSECTIONS
  RoadGraph::roadGraphEdgeIter_BI ei, eiEnd;
  QVector3D p0;
  QVector3D p1;

  //Render edges ===============
  if (false &&edgeDescToLaneMapNum.size()>0){
  int numEdges = 0;

  for (boost::tie(ei, eiEnd) = boost::edges(simRoadGraph->myRoadGraph_BI);
  ei != eiEnd; ++ei)
  {


  p0 = simRoadGraph->myRoadGraph_BI[boost::source(*ei, simRoadGraph->myRoadGraph_BI)].pt;
  p1 = simRoadGraph->myRoadGraph_BI[boost::target(*ei, simRoadGraph->myRoadGraph_BI)].pt;
  /// N
  QVector3D dir = (p1 - p0).normalized();
  QVector3D per = (QVector3D::crossProduct(QVector3D(0, 0, 1.0f), dir).normalized());

  ///
  int numL = simRoadGraph->myRoadGraph_BI[*ei].numberOfLanes;
  ushort laneMapNum = edgeDescToLaneMapNum[*ei];
  for (int lN = 0; lN<numL; lN++){
  uchar trafficLight = 0x01;//default
  //uint lN=0;
  if (trafficLights.size()>0){
  trafficLight = trafficLights[laneMapNum + lN];
  }
  cudaTrafficLightRender[laneMapNum + lN].getInterpolated(trafficLight, trafficLight);

  switch (trafficLight){
  case 0x00:
  glColor3ub(255, 0, 0);
  break;
  case 0xFF:
  glColor3ub(0, 255, 0);
  break;
  default:
  glColor3ub(0, 0, 0);
  //printf("Unknown traffic light state\n");
  }
  float perShift = -0.5f*G::global().getFloat("roadLaneWidth")*(1 + 2 * lN);
  QVector3D p1S = QVector3D(p1.x() + per.x()*perShift, p1.y() + per.y()*perShift, p1.z());
  p1S -= intersectionClearance*dir;//move to be brefore clearance
  /// CONE
  //if(LC::misctools::Global::global()->view_arterials_arrows_render==true){
  LC::misctools::drawCone(dir, p1S - dir*0.5f, 2.0f, 1.0f, 16);
  //}
  }
  //
  }
  }*/
}//

void BTrafficSimulator::renderRoutes(VBORenderManager &rendManger) {
  if (people.numPeople <= 0) {
    return;
  }

  if (routesChanged == false) {//flag to see whether regenerate routes render
    printf("renderRoutes\n");
    glLineWidth(5.0f);
    rendManger.renderStaticGeometry("routes");
    return;
  }

  rendManger.removeStaticGeometry("routes", false);

  printf("people %d indToEdge %d nextEdge %d\n", people.numPeople,
         people.indTo1stEdge.size(), people.nextEdge.size());
  std::vector<Vertex> routes;
  QVector3D p0, p1;

  for (int p = 0; p < people.numPeople; p++) {

    //if (p == 5)break;
    //printf("p %d nextEdge %d initE %d\n", p, people.nextEdge.size(), people.indToEdge[p]);
    //continue;
    QVector3D color(float(qrand()) / RAND_MAX, float(qrand()) / RAND_MAX,
                    float(qrand()) / RAND_MAX);
    float displ = 5.0f * (float(qrand()) / RAND_MAX);
    uint iEdge = people.indTo1stEdge[p];

    //printf("iEdge %u\n",iEdge);
    while (true) {
      ushort edge = people.nextEdge[iEdge];
      //printf("  edge %u\n", edge);
      iEdge++;

      if (edge == 0xFFFF) {
        break;//break person
      }

      if (iEdge >= people.nextEdge.size()) {
        printf("ERROR: reach end of nextEdges and still looking\n");
        return;
      }

      //draw
      if (laneMapNumToEdgeDesc.find(edge) == laneMapNumToEdgeDesc.end()) {
        printf("ERROR edge not found in map\n");//edge not found in map
        continue;
      }

      RoadGraph::roadGraphEdgeDesc_BI ei = laneMapNumToEdgeDesc[edge];
      p0 = simRoadGraph->myRoadGraph_BI[boost::source(ei,
                                        simRoadGraph->myRoadGraph_BI)].pt;
      p1 = simRoadGraph->myRoadGraph_BI[boost::target(ei,
                                        simRoadGraph->myRoadGraph_BI)].pt;
      p0.setZ(40.0f + displ);
      p1.setZ(40.0f + displ);
      // add line
      routes.push_back(Vertex(p0, color, QVector3D(), QVector3D()));
      routes.push_back(Vertex(p1, color, QVector3D(), QVector3D()));
    }//route
  }//people

  routesChanged = false;//render

  if (routes.size() > 0) {
    rendManger.addStaticGeometry("routes", routes, "", GL_LINES,
                                 1 | mode_AdaptTerrain);
    rendManger.renderStaticGeometry("routes");
    //rendManger.removeStaticGeometry("routes");
    printf("Render routes %d line\n", routes.size() / 2);//(two vertex per line)
  }
}//



void BTrafficSimulator::printCurrentState() {
  /////////////////////////////////
  // PRINT EDGES
  RoadGraph::roadGraphEdgeIter_BI ei, eiEnd;

  //printf("Edges State %d\n", boost::num_edges(simRoadGraph->myRoadGraph_BI));
  for (boost::tie(ei, eiEnd) = boost::edges(simRoadGraph->myRoadGraph_BI);
       ei != eiEnd; ++ei) {
    uint s = boost::source(*ei, simRoadGraph->myRoadGraph_BI);
    uint d = boost::target(*ei, simRoadGraph->myRoadGraph_BI);
    int numLines = simRoadGraph->myRoadGraph_BI[*ei].numberOfLanes;
    ushort cEdge = edgeDescToLaneMapNum[*ei];

    for (int lN = 0; lN < numLines; lN++) {
      ushort cEdgeLengthC = edgesData.lengthC[cEdge];
      // check if print
      bool printL = false;

      for (ushort sL = 0; sL < cEdgeLengthC / 8 + 1; sL++) {
        if ((laneMapL[!simulationSt.cArray][(maxWidthL * (cEdge + lN)) + sL]) !=
            0xFFFFFFFFFFFFFFFF) {
          printL = true;
          break;
        }
      }

      if (printL == true) {
        printf("  E %2u (%u-%u) L %d", cEdge, s, d, lN);

        for (ushort sL = 0; sL < cEdgeLengthC / 8 + 1; sL++) {
          printf(" ");
          printf("%016llX", laneMapL[!simulationSt.cArray][(maxWidthL *
                 (cEdge + lN)) + sL]);
        }

        printf("\n");
      }
    }
  }

  /////////////////////////////////
  // PRINT INTERSECTIONS
  for (int iN = 0; iN < intersec.numIn.size(); iN++) {
    bool printI = false;

    if (intersec.req[iN] || intersec.trafficLight[iN]) {
      printf("  I %2d ", iN);//request
      printf("Req: %016llX ", intersec.req[iN]);
      printf("TL : %016llX ", intersec.trafficLight[iN]);
      printf("\n");
    }
  }
}//

void BTrafficSimulator::clearSimulator() {
  laneMapL[0].clear();
  laneMapL[1].clear();
  edgesData.clear();
  edgeDescToLaneMapNum.clear();
  laneMapNumToEdgeDesc.clear();
  people.resize(0);
}//
}
