#include "b18TrafficOD.h"

#include "../roadGraphB2018Loader.h"
#include "../misctools/misctools.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/math/distributions/non_central_t.hpp>

#define ROUTE_DEBUG 0
#define PERSON_DEBUG 0

namespace LC {

B18TrafficOD::B18TrafficOD(const parameters & inputSimParameters) : simParameters(inputSimParameters) {
}//
B18TrafficOD::~B18TrafficOD() {
}//

#ifdef B18_RUN_WITH_GUI
void B18TrafficOD::sampleDistribution(int numberToSample,
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
}
#endif
void B18TrafficOD::randomPerson(int p, B18TrafficPerson &person,
                                uint srcvertex,
                                uint tgtvertex, float startTimeH) {

  // Data
  person.init_intersection = srcvertex;
  person.end_intersection = tgtvertex;
  person.time_departure = startTimeH;// * 3600.0f; //seconds
  // Status
  qsrand(p);
  
  person.a = simParameters.a;
  person.b = simParameters.b;
  person.T = simParameters.T;
  person.v = 0;
  person.num_steps = 0;
  person.co = 0;
  person.active = 0;
  person.numOfLaneInEdge = 0;
  person.color = p << 8;
  person.LC_stateofLaneChanging = 0;
  person.indexPathInit = 0; // 0 should point to -1.

}

void B18TrafficOD::randomPerson(int p, B18TrafficPerson &person,
                                QVector3D housePos3D, QVector3D jobPos3D,
                                float startTimeH,
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

  randomPerson(p, person, srcvertex, tgtvertex, startTimeH);
}

void B18TrafficOD::resetTrafficPersonJob(std::vector<B18TrafficPerson>
    &trafficPersonVec) {
  for (int p = 0; p < trafficPersonVec.size(); p++) {
    trafficPersonVec[p].active = 0;
  }
}//

#ifdef B18_RUN_WITH_GUI
void B18TrafficOD::createRandomPeople(
  int numberPerGen,
  float startTimeH, float endTimeH,
  std::vector<B18TrafficPerson> &trafficPersonVec,
  PeopleJobInfoLayers &simPeopleJobInfoLayers,
  LC::RoadGraph::roadBGLGraph_BI &roadGraph) {
  if (PERSON_DEBUG) {
    printf(">> generateTrafficPersonJob\n");
  }

  QTime timer;
  //////////////////////////////////
  // 1. Generate PEOPLE
  timer.start();
  std::vector<QVector2D> peopleDis;
  QString people_dist_file("test/people_dist.png");
  sampleDistribution(numberPerGen, simPeopleJobInfoLayers.layers[1], peopleDis,
                     people_dist_file);

  if (PERSON_DEBUG) {
    printf(">> >> People %d sampled in %d ms\n", numberPerGen, timer.elapsed());
  }

  //////////////////////////////////
  // 2. Generate JOBS
  timer.restart();
  std::vector<QVector2D> jobDis;
  QString job_dist_file("test/job_dist.png");
  sampleDistribution(numberPerGen, simPeopleJobInfoLayers.layers[0], jobDis,
                     job_dist_file);

  if (PERSON_DEBUG) {
    printf(">> >> Job %d sampled in %d ms\n", numberPerGen, timer.elapsed());
  }

  // 3. Generate PEOPLE TRAFFIC
  timer.restart();
  trafficPersonVec.resize(numberPerGen);
  qsrand(6541799621);

  float midTime = (startTimeH + endTimeH) / 2.0f;

  for (int p = 0; p < numberPerGen; p++) {
    float goToWorkH = midTime + LC::misctools::genRand(startTimeH - midTime,
                      endTimeH - startTimeH);
    randomPerson(p, trafficPersonVec[p], peopleDis[p], jobDis[p], goToWorkH,
                 roadGraph);
  }

  if (PERSON_DEBUG) {
    printf(">> >> PeopleTraffic %d generated in %d ms\n", numberPerGen,
           timer.elapsed());
  }

  if (PERSON_DEBUG) {
    printf("<< generateTrafficPersonJob\n");
  }
}//
#endif

bool fileDistributionInitialized = false;
const float startSamples = 2.5f;
const float endSamples = 14.5f;
const int numBucketsPerHour = 6;
std::vector<float> hToWDistribution;
const bool gaussianDistribution = false; // true = file; false = gaussian.
float sampleFileDistribution() {
  // Initialized.
  if (fileDistributionInitialized == false) {
    QFile inputFile("berkeley_2018/HtoW_trips.csv");

    if (!inputFile.open(QIODevice::ReadOnly)) {
      printf("for fileDistributionInitialized must exist file berkeley_2018/HtoW_trips.csv\n");
      exit(-1);
    }

    int numBuckets = ceil((endSamples - startSamples) * numBucketsPerHour);
    hToWDistribution = std::vector<float>(numBuckets, 0);
    int accPeople = 0;

    QTextStream in(&inputFile);

    while (!in.atEnd()) {
      QString line = in.readLine();
      QStringList fields = line.split(",");

      if (fields.size() >= 3) {
        int numPeople = fields[2].toInt();
        float time = fields[1].toFloat();
        int targetBucket = (time - startSamples) * numBucketsPerHour;

        if (targetBucket < 0 || targetBucket >= hToWDistribution.size()) {
          continue;
        }

        hToWDistribution[targetBucket] += numPeople;
        accPeople += numPeople;
        //printf("time %.2f people %d -> bucket %d of %d\n", time, hToWDistribution[targetBucket], targetBucket, numBuckets);
      }
    }

    inputFile.close();

    // Print and normalize.
    for (int b = 0; b < hToWDistribution.size(); b++) {
      float bucketStartTime = startSamples + b * (1.0f / numBucketsPerHour);
      //printf("htoW,%.2f,%.0f\n", bucketStartTime, hToWDistribution[b]);
      hToWDistribution[b] /= float(accPeople + 1); // distribution
    }

    // Accumulate distribution.
    for (int b = 1; b < hToWDistribution.size(); b++) { // note starts with 1.
      hToWDistribution[b] += hToWDistribution[b - 1];
      float bucketStartTime = startSamples + b * (1.0f / numBucketsPerHour);
      //printf("Acc,%.2f,%f\n", bucketStartTime, hToWDistribution[b]); // acumulate
    }

    fileDistributionInitialized = true;
  }

  // Sample
  // Select bucket
  float randNumBucket = misctools::genRand();
  int b = 0;

  for (; b < hToWDistribution.size(); b++) {
    //printf("Compare %.2f vs %.2f", randNumBucket, hToWDistribution[b]);
    if (hToWDistribution[b] > randNumBucket) {
      break;
    }
  }

  float bucketStartTime = startSamples + b * (1.0f / numBucketsPerHour);
  // Random withinbucket
  float randTimeWithinBucket = misctools::genRand() / numBucketsPerHour;
  //printf("randBNum %.2f --> b %d\n", randNumBucket, b);
  return randTimeWithinBucket + bucketStartTime;
}

void B18TrafficOD::loadB18TrafficPeople(
    float startTimeH, float endTimeH,
    std::vector<B18TrafficPerson> &trafficPersonVec, // out
    RoadGraph::roadBGLGraph_BI &roadGraph, const int limitNumPeople, const bool addRandomPeople) {

  trafficPersonVec.clear();
  QTime timer;
  timer.start();

  if (RoadGraphB2018::demandB2018.size() == 0) {
    printf("ERROR at loadB18TrafficPeople: Imposible to generate b2018 without loading b2018 demmand first\n");
    return;
  }

  const int totalNumPeople = [&limitNumPeople, &addRandomPeople] {
    if (limitNumPeople > 0 && addRandomPeople)
      return limitNumPeople;
    else
      return RoadGraphB2018::totalNumPeople;
  }();
  trafficPersonVec.resize(totalNumPeople);

  boost::mt19937 rng;
  srand(45321654);
  //boost::math::non_central_t_distribution<> td(/*v=*/2.09,/*delta=*/7.51);
  //boost::variate_generator<boost::mt19937&, boost::math::non_central_t_distribution<> > var(rng, td);
  boost::normal_distribution<> nd(7.5, 0.75);
  boost::variate_generator<boost::mt19937 &, boost::normal_distribution<> > var(
    rng, nd);

  int numPeople = 0;
  printf("demandB2018 = %d\n", RoadGraphB2018::demandB2018.size());
  for (int d = 0; (d < RoadGraphB2018::demandB2018.size()) &&
       (numPeople < totalNumPeople); d++) {
    int odNumPeople = std::min<int>(totalNumPeople - numPeople,
                                    RoadGraphB2018::demandB2018[d].num_people);
    //printf("odNumPeople = %d\n", odNumPeople);
    uint src_vertex = RoadGraphB2018::demandB2018[d].src_vertex;
    uint tgt_vertex = RoadGraphB2018::demandB2018[d].tgt_vertex;

    for (int p = 0; p < odNumPeople; p++) {
      float goToWorkH;

      if (gaussianDistribution) {
        goToWorkH  = var();
      } else {
        goToWorkH = sampleFileDistribution();
      }

      randomPerson(numPeople, trafficPersonVec[numPeople], src_vertex, tgt_vertex,
                   goToWorkH);
      // printf("go to work %.2f --> %.2f\n", goToWork, (trafficPersonVec[p].time_departure / 3600.0f));
      numPeople++;
    }
  }
  //printf("trafficPersonVec size = %d", trafficPersonVec.size());

  if (totalNumPeople > numPeople) {
    std::cerr << "Current amount: " << numPeople << std::endl;
    std::cerr << "Total amount: " << totalNumPeople << std::endl;
    printf("No enough on file --> Add random people %d\n",
           (totalNumPeople - numPeople));
    // If this happens, the user ask to generate random people.
    QList<int>  allVertexInd = RoadGraphB2018::indToNodeIndex.keys();

    for (; numPeople < totalNumPeople; numPeople++) {
      uint src_vertex = allVertexInd[rand() % allVertexInd.size()];
      uint tgt_vertex = allVertexInd[rand() % allVertexInd.size()];

      float goToWorkH;

      if (gaussianDistribution) {
        goToWorkH = var();
      } else {
        goToWorkH = sampleFileDistribution();
      }

      randomPerson(numPeople, trafficPersonVec[numPeople], src_vertex, tgt_vertex,
                   goToWorkH);
    }
  }

  if (totalNumPeople != numPeople) {
    printf("ERROR: generateB2018TrafficPeople totalNumPeople != numPeople, this should not happen.");
    exit(-1);
  }

  if (gaussianDistribution) {
    //print histogram
    float binLength = 0.166f;//10min
    float numBins = ceil((endTimeH - startTimeH) / binLength);
    printf("End time %.2f  Start time %.2f --> numBins %f\n", endTimeH, startTimeH,
           numBins);
    std::vector<int> bins(numBins);
    std::fill(bins.begin(), bins.end(), 0);

    for (int p = 0; p < trafficPersonVec.size(); p++) {
      // printf("depart %.2f\n", (trafficPersonVec[p].time_departure / 3600.0f));
      float t = (trafficPersonVec[p].time_departure / 3600.0f) - startTimeH;

      int binN = t / binLength;

      if (binN < 0 || binN >= numBins) {
        printf("ERROR: Bin out of range %d of %f\n", binN, numBins);
        continue;
      }

      bins[binN]++;
    }

    printf("\n");

    for (int binN = 0; binN < bins.size(); binN++) {
      printf("%f %d\n", startTimeH + binN * binLength, bins[binN]);
    }
  } else {
    // Plot histogram of file distribution.
    int numBuckets = ceil((endSamples - startSamples) * numBucketsPerHour);
    std::vector<int> bins(numBuckets, 0);

    for (int p = 0; p < trafficPersonVec.size(); p++) {
      // printf("depart %.2f\n", (trafficPersonVec[p].time_departure / 3600.0f));
      float time = (trafficPersonVec[p].time_departure / 3600.0f);
      int targetBucket = (time - startSamples) * numBucketsPerHour;

      if (targetBucket < 0 || targetBucket >= hToWDistribution.size()) {
        continue;
      }

      bins[targetBucket]++;
    }

    for (int b = 0; b < bins.size(); b++) {
      float bucketStartTime = startSamples + b * (1.0f / numBucketsPerHour);
      //printf("PeopleDist,%.2f,%d\n", bucketStartTime, bins[b]); // acumulate
    }
  }

  printf("loadB18TrafficPeople: People %d\n", numPeople);
}

void B18TrafficOD::loadB18TrafficPeopleSP(
    float startTimeH, float endTimeH,
    std::vector<B18TrafficPerson> &trafficPersonVec, // out
    const std::shared_ptr<abm::Graph>& graph_,
    const int limitNumPeople, const bool addRandomPeople, const std::vector<float> dep_times) {

  trafficPersonVec.clear();
  QTime timer;
  timer.start();

  //printf("demandB2018 size = %d\n", RoadGraphB2018::demandB2018.size());
  if (RoadGraphB2018::demandB2018.size() == 0) {
    printf("ERROR at loadB18TrafficPeopleSP: Imposible to generate b2018 without loading b2018 demmand first\n");
    return;
  }

  const int totalNumPeople = [&limitNumPeople, &addRandomPeople] {
    if (limitNumPeople > 0 && addRandomPeople)
      return limitNumPeople;
    else
      return RoadGraphB2018::totalNumPeople;
  }();
  trafficPersonVec.resize(totalNumPeople);

  /*
  boost::mt19937 rng;
  srand(45321654);
  //boost::math::non_central_t_distribution<> td(v=2.09,delta=7.51);
  //boost::variate_generator<boost::mt19937&, boost::math::non_central_t_distribution<> > var(rng, td);
  boost::normal_distribution<> nd(7.5, 0.75);
  boost::variate_generator<boost::mt19937 &, boost::normal_distribution<> > var(
    rng, nd);
  */
  int numPeople = 0;
  for (int d = 0; (d < RoadGraphB2018::demandB2018.size()) &&
       (numPeople < totalNumPeople); d++) {
    int odNumPeople = std::min<int>(totalNumPeople - numPeople,
                                    RoadGraphB2018::demandB2018[d].num_people);
    //printf("odNumPeople = %d\n", odNumPeople);
    uint src_vertex = RoadGraphB2018::demandB2018[d].src_vertex;
    uint tgt_vertex = RoadGraphB2018::demandB2018[d].tgt_vertex;
    float goToWorkH = dep_times[d];
    //printf("dep time %f\n", goToWorkH);

    for (int p = 0; p < odNumPeople; p++) {

      /*
      if (gaussianDistribution) {
        goToWorkH  = var();
      } else {
        goToWorkH = sampleFileDistribution();
      }
      */

      randomPerson(numPeople, trafficPersonVec[numPeople], src_vertex, tgt_vertex,
                   goToWorkH);
      // printf("go to work %.2f --> %.2f\n", goToWork, (trafficPersonVec[p].time_departure / 3600.0f));
      numPeople++;
    }
  }
  /*
  if (totalNumPeople > numPeople) {
    std::cerr << "Current amount: " << numPeople << std::endl;
    std::cerr << "Total amount: " << totalNumPeople << std::endl;
    printf("No enough on file --> Add random people %d\n",
           (totalNumPeople - numPeople));
    // If this happens, the user ask to generate random people.
    QList<int>  allVertexInd = RoadGraphB2018::indToNodeIndex.keys();

    for (; numPeople < totalNumPeople; numPeople++) {
      uint src_vertex = allVertexInd[rand() % allVertexInd.size()];
      uint tgt_vertex = allVertexInd[rand() % allVertexInd.size()];

      float goToWorkH;

      if (gaussianDistribution) {
        goToWorkH = var();
      } else {
        goToWorkH = sampleFileDistribution();
      }

      randomPerson(numPeople, trafficPersonVec[numPeople], src_vertex, tgt_vertex,
                   goToWorkH);
    }
  }
  */

  if (totalNumPeople != numPeople) {
    printf("ERROR: generateB2018TrafficPeople totalNumPeople != numPeople, this should not happen.");
    exit(-1);
  }

  /*
  if (gaussianDistribution) {
    //print histogram
    float binLength = 0.166f;//10min
    float numBins = ceil((endTimeH - startTimeH) / binLength);
    printf("End time %.2f  Start time %.2f --> numBins %f\n", endTimeH, startTimeH,
           numBins);
    std::vector<int> bins(numBins);
    std::fill(bins.begin(), bins.end(), 0);

    for (int p = 0; p < trafficPersonVec.size(); p++) {
      // printf("depart %.2f\n", (trafficPersonVec[p].time_departure / 3600.0f));
      float t = (trafficPersonVec[p].time_departure / 3600.0f) - startTimeH;

      int binN = t / binLength;

      if (binN < 0 || binN >= numBins) {
        printf("ERROR: Bin out of range %d of %f\n", binN, numBins);
        continue;
      }

      bins[binN]++;
    }

    printf("\n");

    for (int binN = 0; binN < bins.size(); binN++) {
      printf("%f %d\n", startTimeH + binN * binLength, bins[binN]);
    }
  } else {
    // Plot histogram of file distribution.
    int numBuckets = ceil((endSamples - startSamples) * numBucketsPerHour);
    std::vector<int> bins(numBuckets, 0);

    for (int p = 0; p < trafficPersonVec.size(); p++) {
      // printf("depart %.2f\n", (trafficPersonVec[p].time_departure / 3600.0f));
      float time = (trafficPersonVec[p].time_departure / 3600.0f);
      int targetBucket = (time - startSamples) * numBucketsPerHour;

      if (targetBucket < 0 || targetBucket >= hToWDistribution.size()) {
        continue;
      }

      bins[targetBucket]++;
    }

    for (int b = 0; b < bins.size(); b++) {
      float bucketStartTime = startSamples + b * (1.0f / numBucketsPerHour);
      //printf("PeopleDist,%.2f,%d\n", bucketStartTime, bins[b]); // acumulate
    }
  }
  */
  printf("loadB18TrafficPeople: People %d\n", numPeople);
}
}


