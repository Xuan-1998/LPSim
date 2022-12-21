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

#include "cudaGridPollution.h"
#include "../global.h"
#include "../LC_GLWidget3D.h"
#include "../LC_UrbanMain.h"

namespace LC {

CUDAGridPollution::CUDAGridPollution() {
  initialized = false;
}//

void CUDAGridPollution::initPollution(LCUrbanMain *_clientMain) {
  this->gridSize = 250.0f;
  float worldWidth = G::global().getFloat("worldWidth");
  this->gridNumSide = ceil(worldWidth /
                           gridSize);
  printf("**>> gridNumSide %d\n", gridNumSide);
  clientMain = _clientMain;
  maxValue = -FLT_MAX;
  initialized = true;
}//

CUDAGridPollution::~CUDAGridPollution() {
}//

void CUDAGridPollution::addValueToGrid(float currTime,
                                       std::vector<CUDATrafficPerson> &trafficPersonVec,
                                       RoadGraph *simRoadGraph,
                                       LCUrbanMain *_clientMain,
                                       std::map<uint, RoadGraph::roadGraphEdgeDesc_BI> &laneMapNumToEdgeDesc) {
  if (initialized == false) {
    initPollution(_clientMain);
  }

  //printf(">>A\n");
  bool renderMultiEdge = true;
  // create new vector
  std::vector<float> gridValues;
  gridValues.resize(gridNumSide * gridNumSide);
  memset(gridValues.data(), 0, sizeof(float) * gridValues.size());//set zero

  if (lastPersonValue.size() != trafficPersonVec.size()) {
    lastPersonValue.resize(trafficPersonVec.size());
    memset(lastPersonValue.data(), 0, sizeof(float) * lastPersonValue.size());
  }

  // for each active person add the correspondent grid
  for (int p = 0; p < trafficPersonVec.size(); p++) {
    if (trafficPersonVec[p].active != 1) {
      continue;
    }

    float lastValueGas = lastPersonValue[p];
    float newValueGas = trafficPersonVec[p].gas;
    float gasThisCar = newValueGas - lastValueGas;
    lastPersonValue[p] = newValueGas;
    /////////////////////////////////////////////////////////
    // position
    int xIndex, yIndex;
    {
      QVector3D p0, p1;
      float posInLaneM = trafficPersonVec[p].posInLaneM;
      float lenghtLane = trafficPersonVec[p].length;//meters?
      RoadGraph::roadGraphEdgeDesc_BI ei =
        laneMapNumToEdgeDesc[trafficPersonVec[p].personPath[trafficPersonVec[p].currPathEdge]];

      // multi edge
      if (renderMultiEdge == true &&
          simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry.size() > 0) {

        for (int gN = 0;
             gN < simRoadGraph->myRoadGraph_BI[ei].roadSegmentGeometry.size() - 1;
             gN++) {//note -1
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
        // one edge
        p0 = simRoadGraph->myRoadGraph_BI[boost::source(ei,
                                          simRoadGraph->myRoadGraph_BI)].pt;
        p1 = simRoadGraph->myRoadGraph_BI[boost::target(ei,
                                          simRoadGraph->myRoadGraph_BI)].pt;
      }

      float worldWidth = G::global().getFloat("worldWidth");
      float roadLaneWidth = G::global().getFloat("roadLaneWidth");
      QVector3D dir = (p1 - p0).normalized();
      QVector3D per = (QVector3D::crossProduct(QVector3D(0, 0, 1.0f),
                       dir).normalized());
      float perShift = -0.5f * roadLaneWidth *
                       (1 + 2 * trafficPersonVec[p].numOfLaneInEdge);
      QVector3D v = p0 + dir * posInLaneM + perShift *
                    per; // center of the back of the car

      xIndex = (v.x() + worldWidth / 2.0f) /
               gridSize;
      xIndex = xIndex > 0 ? xIndex : 0;
      xIndex = xIndex >= gridNumSide ? gridNumSide - 1 : xIndex;

      yIndex = (v.y() + worldWidth / 2.0f) /
               gridSize;
      yIndex = yIndex > 0 ? yIndex : 0;
      yIndex = yIndex >= gridNumSide ? gridNumSide - 1 : yIndex;
    }
    int index = yIndex * gridNumSide + xIndex;
    gridValues[index] += gasThisCar;
  }

  //update maxValue (used for render)
  for (int gV = 0; gV < gridValues.size(); gV++) {
    maxValue = gridValues[gV] > maxValue ? gridValues[gV] : maxValue;
  }

  // add to temporal vectors
  timeStamp.push_back(currTime);
  gridTimeValues.push_back(gridValues);
  //printf(">>B\n");
}//

void CUDAGridPollution::renderPollution(int valueToRender) {
  if (gridTimeValues.size() <= valueToRender) {// do not do anything
    printf("no render %d\n", gridTimeValues.size());
    return;
  }

  glDisable(GL_CULL_FACE);
  printf("renderPollution maxValue %f index %d\n", maxValue, valueToRender);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  std::vector<float> toRender = gridTimeValues[valueToRender];

  float worldWidth = G::global().getFloat("worldWidth");

  for (int gY = 0; gY < gridNumSide; gY++) {
    for (int gX = 0; gX < gridNumSide; gX++) {
      int index = gY * gridNumSide + gX;
      float x = gX * gridSize - worldWidth / 2.0f;
      float y = gY * gridSize - worldWidth / 2.0f;
      float rC, gC, bC;
      float valueToRender = 1.0f - (toRender[index] / (maxValue *
                                    0.25f)); //0.75 REMOVE!!!!

      LC::misctools::HSVtoRGB(&rC, &gC, &bC, 120.0f * valueToRender, 0.9f,
                              0.8f); //0 red 120` green
      glColor4f(rC, gC, bC, 0.8f);
      glBegin(GL_QUADS);
      glVertex3f(x, y, 20.0f);
      glVertex3f(x + gridSize, y, 20.0f);
      glVertex3f(x + gridSize, y + gridSize, 20.0f);
      glVertex3f(x, y + gridSize, 20.0f);
      glEnd();
    }
  }

  glDisable(GL_BLEND);
  glEnable(GL_CULL_FACE);
}//

void CUDAGridPollution::saveToFile(QString fileName) {
  printf(">>CUDAGridPollution::saveToFile\n");

  if (gridTimeValues.size() <= 0) {
    printf("ERROR: saveToFile when not pollution calculated\n");
    return;
  }

  QFile file(fileName);

  if (!file.open(QIODevice::WriteOnly)) {
    printf("ERROR: Not possible to read %s\n", fileName.toLatin1().constData());
    return;
  }

  QTextStream out(&file);
  out << "NumTimeStamps:" << timeStamp.size() << ";";
  float worldWidth = G::global().getFloat("worldWidth");
  out << "WorldWidth:" << worldWidth << ";";
  out << "gridSize:" << this->gridSize << ";";
  out << "gridNumSide:" << this->gridNumSide << ";";
  out << "\n";

  for (int tN = 0; tN < gridTimeValues.size(); tN++) {
    out << "timeStamp:" << timeStamp[tN] << ";";

    for (int gY = 0; gY < gridNumSide; gY++) {
      for (int gX = 0; gX < gridNumSide; gX++) {
        int index = gY * gridNumSide + gX;
        out << gridTimeValues[tN][index] << ";";
      }
    }

    out << "\n";
  }

  printf("<<CUDAGridPollution::saveToFile\n");
}//

void CUDAGridPollution::loadSimSave(QString fileName, LCUrbanMain *_clientMain) {
  clientMain = _clientMain;
  printf(">>CUDAGridPollution::loadSimSave\n");
  fileName = "data/schedule.txt";
  QFile file(fileName);

  if (!file.open(QIODevice::ReadOnly)) {
    printf("ERROR: Not possible to read %s\n", fileName.toLatin1().constData());
    return;
  }

  /*// set up to run
  clientMain->cudaWeatherRoadsSlot(true); //REMOVE comment!!
  clientMain->ui.pollutionCheckBox->setChecked(true);
  QCoreApplication::processEvents();
  QCoreApplication::processEvents();

  printf(">>Init simulator\n");
  clientMain->ui.cudaNumPeopleSpinBox->setValue(10);
  int numPeople = clientMain->ui.cudaNumPeopleSpinBox->value();

  if (clientMain->mGLWidget_3D->infoLayers.layers.size() <= 0) {
    clientMain->mGLWidget_3D->infoLayers.initInfoLayer();
  }
  float cuda_delta_time = G::global().getFloat("cuda_delta_time");
  float cellSize = clientMain->ui.cudaCellSizeSpinBox->value();//meter
  clientMain->mGLWidget_3D->cudaTrafficSimulator.initSimulator(deltaTime,
      cellSize, &clientMain->cg.roadGraph, numPeople,
      clientMain->mGLWidget_3D->infoLayers);

  QTextStream in(&file);
  ////////////////////////////////////
  // READ FILE
  printf("READ FILE\n");
  QStringList lines;// read complete file

  while (!in.atEnd()) {
    QString line = in.readLine().trimmed();

    if (line.length() > 0 && !line.startsWith("#")) {
      int ind = line.indexOf('#');

      if (ind > 0) {
        line = line.left(ind);
      }

      lines.push_back(line);
    }
  }

  printf("num lines %d\n", lines.size());
  int currLine = 0;
  int totalPeople = lines[currLine].split(':')[1].toInt();//total population
  currLine++;
  printf("total People %d\n", totalPeople);
  QTime timer;
  //////////////////////////////////
  // Generate PEOPLE
  timer.start();
  std::vector<QVector2D> peopleDis;
  clientMain->mGLWidget_3D->cudaTrafficSimulator.cudaPMTrafficPersonJob.sampleDistribution(
    totalPeople,
    clientMain->mGLWidget_3D->cudaTrafficSimulator.simPeopleJobInfoLayers.layers[1],
    peopleDis, QString("test/people_dist.png"));
  //////////////////////////////////
  // Generate JOBS
  std::vector<QVector2D> jobDis;
  clientMain->mGLWidget_3D->cudaTrafficSimulator.cudaPMTrafficPersonJob.sampleDistribution(
    totalPeople,
    clientMain->mGLWidget_3D->cudaTrafficSimulator.simPeopleJobInfoLayers.layers[0],
    jobDis, QString("test/job_dist.png"));
  printf("Sampling done in %d ms\n", timer.elapsed());
  //////////////////////////////
  // PROCESS FILE
  timer.restart();
  std::vector<CUDATrafficPerson> newTrafPerVec;

  while (currLine < lines.size()) {
    QString dayName = lines[currLine];
    printf("===== %s =====\n", dayName.toLatin1().constData());
    currLine++;
    // read distributions
    float probability, mean, variance;
    int numPeoProb;//number of people work/home

    while (currLine < lines.size()) {
      if (lines[currLine].startsWith("TOWORK") ||
          lines[currLine].startsWith("TOHOME") ||
          lines[currLine].startsWith("RANDOMERRAND") ||
          lines[currLine].startsWith("NORUSHHOURTRIP")) {
        QStringList fields = lines[currLine].split(':');

        if (lines[currLine].startsWith("TOWORK")) {
          QStringList fields = lines[currLine].split(':');
          probability = fields[3].toFloat();
          numPeoProb = probability * totalPeople;
          printf("numPeoProb %d\n", numPeoProb);
          mean = fields[5].toFloat();
          variance = fields[7].toFloat();

          int beforeNumber = newTrafPerVec.size();
          newTrafPerVec.resize(beforeNumber + numPeoProb);

          for (int nP = 0; nP < numPeoProb; nP++) {
            int index = beforeNumber + nP;
            float goToWork = LC::misctools::genRandNormal(mean, variance);
            clientMain->mGLWidget_3D->cudaTrafficSimulator.cudaPMTrafficPersonJob.randomPerson(
              nP, newTrafPerVec[index], peopleDis[nP], jobDis[nP], goToWork,
              clientMain->mGLWidget_3D->cudaTrafficSimulator.simRoadGraph->myRoadGraph_BI);
          }

          printf("WORK B %d A %d prob %f mean %f var %f\n", beforeNumber,
                 newTrafPerVec.size(), probability, mean, variance);
        }

        if (lines[currLine].startsWith("TOHOME")) {
          mean = fields[3].toFloat();
          variance = fields[5].toFloat();

          int beforeNumber = newTrafPerVec.size();
          newTrafPerVec.resize(beforeNumber + numPeoProb);

          for (int nP = 0; nP < numPeoProb; nP++) {
            int index = beforeNumber + nP;
            float goToHome = LC::misctools::genRandNormal(mean, variance);
            clientMain->mGLWidget_3D->cudaTrafficSimulator.cudaPMTrafficPersonJob.randomPerson(
              nP, newTrafPerVec[index], peopleDis[nP], jobDis[nP], goToHome,
              clientMain->mGLWidget_3D->cudaTrafficSimulator.simRoadGraph->myRoadGraph_BI);
          }

          printf("HOME B %d A %d prob %f mean %f var %f\n", beforeNumber,
                 newTrafPerVec.size(), probability, mean, variance);
        }

        if (lines[currLine].startsWith("RANDOMERRAND")) {
          float erranProb = fields[3].toFloat();
          mean = fields[5].toFloat();
          variance = fields[7].toFloat();

          int numPeopleErrand = numPeoProb * erranProb;
          //printf("ERRAND erranProb %f mean %f var %f\n",erranProb,mean,variance);
          //printf("numPeopleErrand %d numPeoProb %d erranProb %f\n",numPeopleErrand,numPeoProb,erranProb);
          std::vector<int> randErrand;
          randErrand.resize(numPeoProb);

          for (int eN = 0; eN < numPeoProb; eN++) {
            randErrand[eN] = eN;
          }

          std::random_shuffle(randErrand.begin(), randErrand.end());//random shuffle
          randErrand.resize(numPeopleErrand);
          printf("numPeopleErrand %d\n", numPeopleErrand);
          int beforeNumber = newTrafPerVec.size();
          newTrafPerVec.resize(beforeNumber + numPeopleErrand * 2);//2 go and go back

          //QSetIterator<int> i(personErrand);
          int curIndex = 0;

          //while (i.hasNext()){
          for (int i = 0; i < randErrand.size(); i++) {
            int nP = randErrand[i];
            //int nP = i.next();
            int index = beforeNumber + curIndex;
            curIndex++;
            float errandTime = LC::misctools::genRandNormal(mean, variance);
            //printf("errandTime %f %f %f\n",errandTime,mean,variance);
            // GO
            QVector3D randomErrandPos(400.0f * ((float) qrand() / RAND_MAX),
                                      400.0f * ((float) qrand() / RAND_MAX), 0);

            if (errandTime < 16) {
              clientMain->mGLWidget_3D->cudaTrafficSimulator.cudaPMTrafficPersonJob.randomPerson(
                nP, newTrafPerVec[index], jobDis[nP], jobDis[nP] + randomErrandPos, errandTime,
                clientMain->mGLWidget_3D->cudaTrafficSimulator.simRoadGraph->myRoadGraph_BI);
            } else {
              clientMain->mGLWidget_3D->cudaTrafficSimulator.cudaPMTrafficPersonJob.randomPerson(
                nP, newTrafPerVec[index], peopleDis[nP], peopleDis[nP] + randomErrandPos,
                errandTime,
                clientMain->mGLWidget_3D->cudaTrafficSimulator.simRoadGraph->myRoadGraph_BI);
            }

            // RETURN
            index = beforeNumber + curIndex;
            curIndex++;
            errandTime += LC::misctools::genRand(1.0, 1.2);//stay a bit doing errand

            if (errandTime < 16) {
              clientMain->mGLWidget_3D->cudaTrafficSimulator.cudaPMTrafficPersonJob.randomPerson(
                nP, newTrafPerVec[index], jobDis[nP] + randomErrandPos, jobDis[nP], errandTime,
                clientMain->mGLWidget_3D->cudaTrafficSimulator.simRoadGraph->myRoadGraph_BI);
            } else {
              clientMain->mGLWidget_3D->cudaTrafficSimulator.cudaPMTrafficPersonJob.randomPerson(
                nP, newTrafPerVec[index], peopleDis[nP] + randomErrandPos, peopleDis[nP],
                errandTime,
                clientMain->mGLWidget_3D->cudaTrafficSimulator.simRoadGraph->myRoadGraph_BI);
            }
          }

          printf("ERRAND B %d A %d erranProb %f mean %f var %f\n", beforeNumber,
                 newTrafPerVec.size(), erranProb, mean, variance);
        }

        if (lines[currLine].startsWith("NORUSHHOURTRIP")) {
          float nrProb = fields[3].toFloat();
          mean = fields[5].toFloat();
          variance = fields[7].toFloat();
          float durMean = fields[9].toFloat();
          float durVar = fields[11].toFloat();
          int numPeopleNR = numPeoProb * nrProb;
          //printf("numPeopleNR %d nrProb %f\n",numPeopleNR,nrProb);
          int beforeNumber = newTrafPerVec.size();
          newTrafPerVec.resize(beforeNumber + numPeopleNR * 2);//2 go and go back

          for (int nrP = 0; nrP < numPeopleNR; nrP++) {
            int index = beforeNumber + (nrP * 2);
            int nP = totalPeople - 1 - nrP;
            QVector3D randomErrandPos(1000.0f * ((float) qrand() / RAND_MAX),
                                      1000.0f * ((float) qrand() / RAND_MAX), 0);
            float nrTime = LC::misctools::genRandNormal(mean, variance);
            // GO
            clientMain->mGLWidget_3D->cudaTrafficSimulator.cudaPMTrafficPersonJob.randomPerson(
              nP, newTrafPerVec[index], peopleDis[nP], peopleDis[nP] + randomErrandPos,
              nrTime, clientMain->mGLWidget_3D->cudaTrafficSimulator.simRoadGraph->myRoadGraph_BI);
            index = beforeNumber + (nrP * 2) + 1;
            nrTime += LC::misctools::genRandNormal(durMean, durVar);
            clientMain->mGLWidget_3D->cudaTrafficSimulator.cudaPMTrafficPersonJob.randomPerson(
              nP, newTrafPerVec[index], peopleDis[nP] + randomErrandPos, peopleDis[nP],
              nrTime, clientMain->mGLWidget_3D->cudaTrafficSimulator.simRoadGraph->myRoadGraph_BI);
          }

          printf("NR B %d A %d nrProb %f mean %f var %f durMean %f durVar %f\n",
                 beforeNumber, newTrafPerVec.size(), nrProb, mean, variance, durMean, durVar);
        }

        currLine++;
      } else {
        break;
      }
    }

    std::vector<int> countH;
    countH.resize(24);
    memset(countH.data(), 0, sizeof(countH[0]) * countH.size());

    for (int pN = 0; pN < newTrafPerVec.size(); pN++) {
      int hour = newTrafPerVec[pN].time_departure / 3600.0f;

      if (hour > 23) {
        printf("Time %d F: %f\n", hour, newTrafPerVec[pN].time_departure);
        hour = 23;
      }

      countH[hour]++;
    }

    for (int tN = 0; tN < countH.size(); tN++) {
      printf("%d;%d\n", tN, countH[tN]);
    }

    newTrafPerVec.resize(10);//// REMOVE line
    // simulate
    printf("simulate\n");
    clientMain->mGLWidget_3D->cudaTrafficSimulator.trafficPersonVec = newTrafPerVec;
    int numPasses = clientMain->ui.cudaShortestPathNumPassesSpinBox->value();
    printf("simulate2\n");
    clientMain->mGLWidget_3D->cudaTrafficSimulator.simulateInCPU_MultiPass(
      numPasses, false);// false=not calculate inside peoplejobs
    // save
    saveToFile("data/pollution_" + dayName + ".txt");
    newTrafPerVec.clear();//clear after used
  }

  file.close();
  printf("<<CUDAGridPollution::loadSimSave %d\n", timer.elapsed());
  */
}//

}  // namespace LC
