/************************************************************************************************
*		@desc Class to create people for B2018.
*		@author igarciad
************************************************************************************************/

#include "bPMTrafficPerson.h"

#define ROUTE_DEBUG 0
#define PERSON_DEBUG 0

#include <QTime>
#include "qstringlist.h"
#include <random>
#include "../roadGraphDynameqLoader.h"
#include "../roadGraphB2018Loader.h"

#include "bTrafficConstants.h"

namespace LC {


void BPMTrafficPerson::sampleDistribution(int numberToSample,
    PeopleJobOneLayer &dist, std::vector<QVector2D> &samples, QString &name) {

  QMap<int, int> posInArrayIndexInSamples;
  int posInArray = 0;
  std::vector<float> accProbability;

  for (int mP = 0; mP < dist.amplitudes.size(); mP++) {
    float ampl = dist.amplitudes[mP];

    if (ampl > 0) {
      float accProb = ampl;

      if (accProbability.size() > 0) {
        accProb += accProbability[accProbability.size() - 1];
      }

      accProbability.push_back(accProb);
      posInArrayIndexInSamples.insert(posInArray, mP);
      posInArray++;
    }
  }

  int maxAccValue = accProbability[accProbability.size() - 1];

  samples.resize(numberToSample);
  std::random_device rd;
  std::normal_distribution<float> distSam(0, dist.stdev);
  std::mt19937 rand_engine(rd());

  for (int nSam = 0; nSam < numberToSample; nSam++) {
    // find amplitude
    float randAmpl = (((float)qrand()) / RAND_MAX) *
                     maxAccValue; //sample in the acc
    int aN;
    bool found = true;

    for (aN = 0; aN < accProbability.size(); aN++) {
      if (accProbability[aN] > randAmpl) {
        found = true;
        break;
      }
    }

    //printf("aN %d randAmpl %f accPro[aN] %f\n",aN,randAmpl,accProbability[aN]);
    if (found == false) {
      printf("ERROR randAmpl not found\n");
    }

    // sample gaussian nad create sample
    samples[nSam].setX(distSam(rand_engine) +
                       dist.samplePosition[posInArrayIndexInSamples[aN]].x());
    samples[nSam].setY(distSam(rand_engine) +
                       dist.samplePosition[posInArrayIndexInSamples[aN]].y());
  }

  if (name.length() > 0) {
    if (PERSON_DEBUG) {
      printf(">> >> >> 2.1. normalize image to check and save maxAccValue %d numSample %d\n",
             maxAccValue, numberToSample);
    }

    cv::Mat peopleSelecF = cv::Mat::zeros(1024, 1024, CV_32FC1);
    cv::Mat peopleSelecU = cv::Mat::zeros(1024, 1024, CV_8UC1);
    float maxDesity = 0;
    int r, c;

    for (int nSam = 0; nSam < numberToSample; nSam++) {
      r = ((samples[nSam].y() / G::global().getFloat("worldWidth")) + 0.5f) *
          peopleSelecF.rows;
      c = ((samples[nSam].x() / G::global().getFloat("worldWidth")) + 0.5f) *
          peopleSelecF.cols;

      if (r < 0) {
        r = 0;
      }

      if (r >= peopleSelecF.rows) {
        r = peopleSelecF.rows - 1;
      }

      if (c < 0) {
        c = 0;
      }

      if (c >= peopleSelecF.rows) {
        c = peopleSelecF.cols - 1;
      }

      peopleSelecF.at<float>(r, c) += 1.0f;

      if (maxDesity < peopleSelecF.at<float>(r, c)) {
        maxDesity = peopleSelecF.at<float>(r, c);
      }
    }

    if (PERSON_DEBUG) {
      printf("maxDensity %f\n", maxDesity);
    }

    for (int c = 0; c < peopleSelecU.cols; c++) {
      for (int r = 0; r < peopleSelecU.rows; r++) {
        peopleSelecU.at<uchar>(r, c) = (uchar)(255.0f * peopleSelecF.at<float>(r,
                                               c) / maxDesity);
      }
    }

    cv::imwrite(name.toLatin1().constData(), peopleSelecU);
  }

  if (PERSON_DEBUG) {
    printf("<<sampleDistribution\n");
  }
}//

void BPMTrafficPerson::randomPerson(int p, BTrafficPeople &people,
                                    QVector3D housePos3D, QVector3D jobPos3D,
                                    float startTime,
                                    LC::RoadGraph::roadBGLGraph_BI &roadGraph) {
  LC::RoadGraph::roadGraphVertexDesc srcvertex, tgtvertex;
  RoadGraph::roadGraphVertexIter vi, viEnd;

  // 3.3 find closest nodes
  QVector2D hP = housePos3D.toVector2D();
  QVector2D jP = jobPos3D.toVector2D();
  float minHouseDis = FLT_MAX;
  float minJobDis = FLT_MAX;

  for (boost::tie(vi, viEnd) = boost::vertices(roadGraph);
       vi != viEnd; ++vi) {
    QVector2D roadVert(roadGraph[*vi].pt);
    float newHDist = (hP - roadVert).lengthSquared();

    if (newHDist < minHouseDis) {
      srcvertex = *vi;
      minHouseDis = newHDist;
    }

    float newJDist = (jP - roadVert).lengthSquared();

    if (newJDist < minJobDis) {
      tgtvertex = *vi;
      minJobDis = newJDist;
    }
  }

  // Data
  people.init_intersection[p] = srcvertex;
  people.end_intersection[p] = tgtvertex;
  people.time_departure[p] = startTime * 3600.0f; //seconds
  //printf("Person %d: init %u end %u Time %f\n",p,srcvertex,tgtvertex,goToWork);
  // Status
  qsrand(p);
  people.a[p] = 1.0f + ((float)qrand()) / RAND_MAX;//acceleration 1-2m/s2
  people.b[p] = 1.0f + ((float)qrand()) / RAND_MAX;//break 1-2m/s2
  people.T[p] = 0.8f + 1.2f * (((float)qrand()) / RAND_MAX); //time heading 0.8-2s
  people.v[p] = 0;
  //person.num_steps=0;
  //person.gas=0;
  people.active[p] = 0;
  //person.numOfLaneInEdge=0;
  //person.color=p<<8;
  //person.color=(((float)qrand())/RAND_MAX)*0xFFFFFFFF;
  //person.LC_stateofLaneChanging=0;
  //person.personPath[0]=-1;
}//

void BPMTrafficPerson::randomPerson(int p, BTrafficPeople &people,
                                    uint srcvertex, uint tgtvertex, float startTime, int carType) {
  // Data
  people.init_intersection[p] = srcvertex;
  people.end_intersection[p] = tgtvertex;
  //printf("src %u tgt %u\n", srcvertex, tgtvertex);
  people.time_departure[p] = startTime * 3600.0f; //seconds
  //printf("Person %d: init %u end %u Time %f\n",p,srcvertex,tgtvertex,goToWork);
  // Status
  qsrand(p);
  people.carType[p] = carType;

  if (carType == 0) { //normal car
    people.a[p] = 1.2f + 0.8f * ((float)qrand()) /
                  RAND_MAX; //acceleration 1.2-2m/s2
    people.b[p] = 3.5f + ((float)qrand()) /
                  RAND_MAX;//break 3.5-4.5m/s2 //www.atrf.info/papers/2009/2009_Chaudry_Ranjitkar.pdf
    //1.0f + ((float)qrand()) / RAND_MAX;//break 1-2m/s2
    people.T[p] = 0.8f + 1.2f * (((float)qrand()) / RAND_MAX); //time heading 0.8-2s
  }

  if (carType == 1) { //truck car
    people.a[p] = 0.1f + ((float)qrand()) / RAND_MAX;//acceleration .1-1.1m/s2
    people.b[p] = 0.1f + ((float)qrand()) / RAND_MAX;//break .1-1.1m/s2
    people.T[p] = 2.0f + 3.0f * (((float)qrand()) / RAND_MAX); //time heading 2.0-3s
  }

  //people.a[p] *= (deltaTime*deltaTime/cellSize);//to cells/dTdT
  //people.b[p] *= (deltaTime*deltaTime/cellSize);//to cells/dTdT
  //people.T[p] *= deltaTime;// to dT

  people.a[p] /= cellSize;//to cells/ss
  people.b[p] /= cellSize;//to cells/ss

  people.v[p] = 0;
  people.active[p] = 0;

  //person.num_steps=0;
  //person.gas=0;

  //person.numOfLaneInEdge=0;
  //person.color=p<<8;
  //person.color=(((float)qrand())/RAND_MAX)*0xFFFFFFFF;
  //person.LC_stateofLaneChanging=0;
  //person.personPath[0]=-1;


}//

void BPMTrafficPerson::generateRandomTrafficPeople(
  float start_time, float end_time,
  int numberPeople, PeopleJobInfoLayers &peopleJobInfoLayers,
  RoadGraph::roadBGLGraph_BI &roadGraph,
  BTrafficPeople &people) {  // OUT
  people.clear();

  if (PERSON_DEBUG) {
    printf(">> B generateTrafficPerson\n");
  }

  QTime timer;
  //////////////////////////////////
  // 1. Generate PEOPLE
  timer.start();
  std::vector<QVector2D> peopleDis;
  QString people_dist_path("test/people_dist.png");
  sampleDistribution(numberPeople, peopleJobInfoLayers.layers[1], peopleDis,
                     people_dist_path);

  if (PERSON_DEBUG) {
    printf(">> >> People %d sampled in %d ms\n", numberPeople, timer.elapsed());
  }

  //////////////////////////////////
  // 2. Generate JOBS
  timer.restart();
  std::vector<QVector2D> jobDis;
  QString job_dist_path("test/job_dist.png");
  sampleDistribution(numberPeople, peopleJobInfoLayers.layers[0], jobDis,
                     job_dist_path);

  if (PERSON_DEBUG) {
    printf(">> >> Job %d sampled in %d ms\n", numberPeople, timer.elapsed());
  }

  // 3. Generate PEOPLE TRAFFIC
  timer.restart();
  people.resize(numberPeople);
  qsrand(6541799621);
  //trafficPersonStatusVec.resize(numberPerGen);
  float midTime = (start_time + end_time) / 2.0f;

  for (int p = 0; p < numberPeople; p++) {
    float goToWork = midTime + LC::misctools::genRand(start_time - midTime,
                     end_time - start_time); //6.30-9.30 /// GOOOD ONE
    randomPerson(p, people, peopleDis[p], jobDis[p], goToWork, roadGraph);

  }

  if (PERSON_DEBUG) {
    printf(">> >> PeopleTraffic %d generated in %d ms\n", numberPeople,
           timer.elapsed());
  }

  if (PERSON_DEBUG) {
    printf("<< generateRandomTrafficPeople\n");
  }
}//


void BPMTrafficPerson::resetTrafficPerson(BTrafficPeople &people) {
  if (PERSON_DEBUG) {
    printf("trafficPersonVec %d\n", people.numPeople);
  }

  for (int p = 0; p < people.numPeople; p++) {
    people.active[p] = 0;
  }
}//

void BPMTrafficPerson::generateTrafficPersonFromStream(
  float startTime, float endTime,
  QFile &file,
  RoadGraph::roadBGLGraph_BI &roadGraph,
  BTrafficPeople &people,//OUT
  float factor
) {
  float periodTime = endTime - startTime;
  QString line;
  QStringList fields;
  QTextStream stream(&file);

  if (!stream.atEnd()) {
    line = stream.readLine();  //skip first line
  }

  float densityTotal = 0.0f;
  int densityTotalI = 0;

  while (!stream.atEnd()) {
    line = stream.readLine();
    //qDebug() << line;
    fields = line.split(',', QString::SkipEmptyParts);
    int sourceCentroid = fields[0].toInt();
    int destCentroid = fields[1].toInt();

    float densityC = fields[2].toFloat() * factor ; // !!! REMOVE /10.0f
    densityTotal += densityC;
    densityTotalI += densityC;
    float densityT = fields[4].toFloat() * factor; // !!! REMOVE /10.0f
    densityTotal += densityT;
    densityTotalI += densityT;

    if (sourceCentroid == destCentroid) {
      //printf("ERROR: source and dest centroid (%d) the same %f %f--> skip\n", sourceCentroid, densityC, densityT);
      continue;
    }

    int numC = qRound(densityC * periodTime);
    int numT = qRound(densityT * periodTime);
    int numP = people.numPeople;
    people.resize(numP + numC + numT);

    //printf("source %d dest %d dens %f %f num %d %d\n", sourceCentroid, destCentroid, densityC, densityT, numC, numT);
    //exit(0);
    for (int p = 0; p < numC; p++) {
      float goToWork = startTime + LC::misctools::genRand(0, periodTime);
      uint srcvertex = RoadGraphDynameq::centroidsToVertex[sourceCentroid][qrand() %
                       RoadGraphDynameq::centroidsToVertex[sourceCentroid].size()];
      uint tgtvertex = RoadGraphDynameq::centroidsToVertex[destCentroid][qrand() %
                       RoadGraphDynameq::centroidsToVertex[destCentroid].size()];
      randomPerson(p + numP, people, srcvertex, tgtvertex, goToWork, 0);
    }

    for (int p = 0; p < numT; p++) {
      float goToWork = startTime + LC::misctools::genRand(0, periodTime);
      uint srcvertex = RoadGraphDynameq::centroidsToVertex[sourceCentroid][qrand() %
                       RoadGraphDynameq::centroidsToVertex[sourceCentroid].size()];
      uint tgtvertex = RoadGraphDynameq::centroidsToVertex[destCentroid][qrand() %
                       RoadGraphDynameq::centroidsToVertex[destCentroid].size()];
      randomPerson(p + numP + numC, people, srcvertex, tgtvertex, goToWork, 1);
    }

    if (people.numPeople > 100000) {
      break;  // !!! REMOVE
    }
  }

  printf("<<generateTrafficPerson periodTime %f densityPerH %d %.0f people %d\n",
         periodTime, densityTotalI, densityTotal, people.numPeople);
}//

void BPMTrafficPerson::generateDynameqTrafficPeople(
  RoadGraph::roadBGLGraph_BI &roadGraph,
  BTrafficPeople &people) {  // OUT

  people.clear();
  QTime timer;
  timer.start();

  if (RoadGraphDynameq::centroidsToVertex.size() == 0) {
    printf("ERROR: Imposible to generate dynameq distribution without loading dynameq files first\n");
    return;
  }

  ///////////////////////////////
  // Simple
  bool simpleDemand = true;

  if (simpleDemand == true) {
    QString fileName = "data/Dynameq/SanFranciscoSubArea_2010_EV.csv";
    QFile demandFile(fileName); // Create a file handle for the file named

    if (!demandFile.open(QIODevice::ReadOnly | QIODevice::Text)) { // Open the file
      printf("Can't open file '%s'\n", fileName.toLatin1().constData());
      return;
    }

    generateTrafficPersonFromStream(2.5f, 3.5f, demandFile, roadGraph, people);
    printf("  2.30 to 3.30 people %d\n", people.numPeople);
  }

  ///////////////////////////////
  // Dynameq
  if (simpleDemand == false) {
    int cNumPeople = 0;
    // warm up
    QString fileNameW = "data/Dynameq/SanFranciscoSubArea_2010_MD.csv";
    QFile demandFileW(fileNameW); // Create a file handle for the file named

    if (!demandFileW.open(QIODevice::ReadOnly | QIODevice::Text)) { // Open the file
      printf("Can't open file '%s'\n", fileNameW.toLatin1().constData());
      return;
    }

    generateTrafficPersonFromStream(14.5f, 15.5f, demandFileW, roadGraph, people,
                                    0.13364f);
    printf("  2.30 to 3.30 people %d\n", people.numPeople - cNumPeople);
    cNumPeople = people.numPeople;
    //peak
    QString fileNameP = "data/Dynameq/SanFranciscoSubArea_2010_PM.csv";
    QFile demandFileP(fileNameP); // Create a file handle for the file named

    if (!demandFileP.open(QIODevice::ReadOnly | QIODevice::Text)) { // Open the file
      printf("Can't open file '%s'\n", fileNameP.toLatin1().constData());
      return;
    }

    generateTrafficPersonFromStream(15.5f, 18.5f, demandFileP, roadGraph, people,
                                    1.0f / 3.0f); // /3 to be per hour?
    printf("  3.30 to 6.30 people %d\n", people.numPeople - cNumPeople);
    cNumPeople = people.numPeople;
    //EV
    QString fileNameE = "data/Dynameq/SanFranciscoSubArea_2010_EV.csv";
    QFile demandFileE(fileNameE); // Create a file handle for the file named

    if (!demandFileE.open(QIODevice::ReadOnly | QIODevice::Text)) { // Open the file
      printf("Can't open file '%s'\n", fileNameE.toLatin1().constData());
      return;
    }

    generateTrafficPersonFromStream(18.5f, 19.5f, demandFileE, roadGraph, people,
                                    0.22594f);
    printf("  6.30 to 7.30 people %d\n", people.numPeople - cNumPeople);

    //print histogram
    float binLength = 0.166f;//10min
    float startTime = 14.5f;
    float endTime = 19.5f;
    float numBins = ceil((endTime - startTime) / binLength);
    std::vector<int> bins(numBins);
    std::fill(bins.begin(), bins.end(), 0);

    for (int p = 0; p < people.numPeople; p++) {
      float t = (people.time_departure[p] / 3600.0f) - startTime;
      int binN = t / binLength;
      bins[binN]++;
    }

    printf("\n");

    for (int binN = 0; binN < bins.size(); binN++) {
      printf("%f %d\n", startTime + binN * binLength, bins[binN]);
    }

    printf("\n");
  }
}


void BPMTrafficPerson::generateB2018TrafficPeople(
  float start_time, float end_time,
  RoadGraph::roadBGLGraph_BI &roadGraph,
  BTrafficPeople &people) {  // OUT

  people.clear();
  QTime timer;
  timer.start();

  if (RoadGraphB2018::demandB2018.size() == 0) {
    printf("ERROR: Imposible to generate b2018 without loading b2018 demmand first\n");
    return;
  }

  people.resize(RoadGraphB2018::totalNumPeople);
  qsrand(6541799621);
  float midTime = (start_time + end_time) / 2.0f;

  int numPeople = 0;

  for (int d = 0; d < RoadGraphB2018::demandB2018.size(); d++) {
    int odNumPeople = RoadGraphB2018::demandB2018[d].num_people;
    uint src_vertex = RoadGraphB2018::demandB2018[d].src_vertex;
    uint tgt_vertex = RoadGraphB2018::demandB2018[d].tgt_vertex;

    for (int p = 0; p < odNumPeople; p++) {
      float goToWork = midTime + LC::misctools::genRand(start_time - midTime,
                       end_time - start_time); //6.30-9.30 /// GOOOD ONE
      int car_type = 0; // all normal cars??

      randomPerson(numPeople, people,
                   src_vertex, tgt_vertex, goToWork, car_type);
      numPeople++;
    }

  }

  if (RoadGraphB2018::totalNumPeople != numPeople) {
    printf("ERROR: generateB2018TrafficPeople totalNumPeople != numPeople, this should not happen.");
    exit(-1);
  }


  //print histogram
  float binLength = 0.166f;//10min
  float startTime = 14.5f;
  float endTime = 19.5f;
  float numBins = ceil((endTime - startTime) / binLength);
  std::vector<int> bins(numBins);
  std::fill(bins.begin(), bins.end(), 0);

  for (int p = 0; p < people.numPeople; p++) {
    float t = (people.time_departure[p] / 3600.0f) - startTime;
    int binN = t / binLength;
    bins[binN]++;
  }

  printf("\n");

  for (int binN = 0; binN < bins.size(); binN++) {
    printf("%f %d\n", startTime + binN * binLength, bins[binN]);
  }

  printf("\n");

}


}  // namespace LC


