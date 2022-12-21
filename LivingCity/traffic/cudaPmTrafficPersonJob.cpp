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

#include "cudaPmTrafficPersonJob.h"

#define ROUTE_DEBUG 0
#define PERSON_DEBUG 0

namespace LC {

CUDAPMTrafficPersonJob::CUDAPMTrafficPersonJob() {
}//
CUDAPMTrafficPersonJob::~CUDAPMTrafficPersonJob() {
}//

void CUDAPMTrafficPersonJob::sampleDistribution(int numberToSample,
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

  /*cv::Mat nomDistAllChannelsJob;//=cv::Mat::zeros(peopleDistribution.rows,peopleDistribution.cols,CV_32FC3);
  matDistribution.convertTo(nomDistAllChannelsJob,CV_32FC3);
  std::vector<cv::Mat> normDistCh;
  cv::split(nomDistAllChannelsJob, normDistCh);
  float acc=0;
  for(int c=0;c<normDistCh[0].cols;c++){
        for(int r=0;r<normDistCh[0].rows;r++){
                acc+=normDistCh[0].at<float>(r,c);
        }
  }
  //2. sample until have enough people
  int cNum=0;
  int rows=normDistCh[0].rows;
  int cols=normDistCh[0].cols;
  cv::Mat peopleSelecJob=cv::Mat::zeros(matDistribution.rows,matDistribution.cols,CV_32FC1);
  int maxNum=INT_MIN;
  while(cNum<numberToSample){
        //if(cNumPer%(numberPerGen/20)==0)printf("cNumPer %d of numberPerGen %d\n",cNumPer,numberPerGen);
        int randR=qrand()%rows;
        int randC=qrand()%cols;
        float prob=1000*normDistCh[0].at<float>(randR,randC);
        float randChose=acc*(((float)qrand()+1)/RAND_MAX);

        //printf("prob %f randChose %f\n",prob,randChose);
        if(prob>randChose){//selected
                //printf("prob %f randChose %f\n",prob,randChose);
                peopleSelecJob.at<float>(randR,randC)+=1.0f;
                samples.push_back(QVector2D(randC,randR));
                if(peopleSelecJob.at<float>(randR,randC)>maxNum)maxNum=peopleSelecJob.at<float>(randR,randC);
                cNum++;
        }
  }
  if(name.length()>0){
        printf(">> >> >> 2.1. normalize image to check and save acc %f maxPeo %d\n",acc,maxNum);
        cv::Mat peopleSelecU=cv::Mat::zeros(matDistribution.rows,matDistribution.cols,CV_8UC1);
        for(int c=0;c<normDistCh[0].cols;c++){
                for(int r=0;r<normDistCh[0].rows;r++){
                        peopleSelecU.at<uchar>(r,c)=(uchar)(255.0f*peopleSelecJob.at<float>(r,c)/maxNum);
                }
        }
        cv::imwrite(name.toLatin1().constData(),peopleSelecU);//"test/img1_job.png"
  }*/
}

void CUDAPMTrafficPersonJob::randomPerson(int p, CUDATrafficPerson &person,
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
  person.init_intersection = srcvertex;
  person.end_intersection = tgtvertex;
  person.time_departure = startTime * 3600.0f; //seconds
  //printf("Person %d: init %u end %u Time %f\n",p,srcvertex,tgtvertex,goToWork);
  // Status
  qsrand(p);
  person.a = 1.0f + ((float)qrand()) / RAND_MAX; //acceleration 1-2m/s2
  person.b = 1.0f + ((float)qrand()) / RAND_MAX; //break 1-2m/s2
  person.T = 0.8f + 1.2f * (((float)qrand()) / RAND_MAX); //time heading 0.8-2s
  person.v = 0;
  person.num_steps = 0;
  person.gas = 0;
  person.active = 0;
  person.numOfLaneInEdge = 0;
  person.color = p << 8;
  //person.color=(((float)qrand())/RAND_MAX)*0xFFFFFFFF;
  person.LC_stateofLaneChanging = 0;
  person.personPath[0] = -1;
}

void CUDAPMTrafficPersonJob::generateTrafficPersonJob(
  int numberPerGen,
  std::vector<CUDATrafficPerson> &trafficPersonVec,
  //cv::Mat& peopleDistribution,
  //cv::Mat& jobDistribution,
  PeopleJobInfoLayers &simPeopleJobInfoLayers,
  LC::RoadGraph::roadBGLGraph_BI &roadGraph) {
  //numberPerGen=10000;
  //this->peopleDistribution=&peopleDistribution;
  //this->jobDistribution=&jobDistribution;
  if (PERSON_DEBUG) {
    printf(">> CUDA generateTrafficPersonJob\n");
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

  //trafficPersonStatusVec.resize(numberPerGen);
  for (int p = 0; p < numberPerGen; p++) {
    float goToWork = 8.0f + LC::misctools::genRand(-1.5f,
                     1.5f); //6.30-9.30 /// GOOOD ONE
    randomPerson(p, trafficPersonVec[p], peopleDis[p], jobDis[p], goToWork,
                 roadGraph);
    /*LC::RoadGraph::roadGraphVertexDesc srcvertex,tgtvertex;
    RoadGraph::roadGraphVertexIter vi, viEnd;


    //3.1 house and job possition

    QVector2D housePos3D=peopleDis[p];
    housePos3D.setX((housePos3D.x()));
    housePos3D.setY((housePos3D.y()));

    QVector2D jobPos3D=jobDis[p];
    jobPos3D.setX((jobPos3D.x()));
    jobPos3D.setY((jobPos3D.y()));
    //3.2 working hours
    //oneP.workingHours=LC::misctools::genRandNormal(38.4f,10.0f);//38.4 mean in USA
    float workingHours=38.4f+LC::misctools::genRand(-1.0f,1.0f);
    //oneP.goToWork=LC::misctools::genRandNormal(9.0f,2.0f);//9.0 mean in USA
    //float goToWork=9.0f+LC::misctools::genRand(-0.5f,0.5f);
    //float goToWork=8.0f+LC::misctools::genRand(-0.9f,1.5f);
    //float goToWork=8.0f+LC::misctools::genRand(-1.5f,1.5f);//6.30-9.30 /// GOOOD ONE
    //float goToWork=8.0f+LC::misctools::genRand(-1.0f,1.0f);//7.00-9.00
    float goToWork=8.0f+LC::misctools::genRand(-1.5f,1.5f);//6.30-9.30 /// GOOOD ONE
    float goToHome=goToWork+(workingHours/7.0f)+LC::misctools::genRand();//go to work + avg work + random var


    // 3.3 find closest nodes
    QVector2D hP=housePos3D;
    QVector2D jP=jobPos3D;
    float minHouseDis=FLT_MAX;
    float minJobDis=FLT_MAX;
    for(boost::tie(vi, viEnd) = boost::vertices(roadGraph);
        vi != viEnd; ++vi){
        QVector2D roadVert(roadGraph[*vi].pt);
        float newHDist=(hP-roadVert).lengthSquared();
        if(newHDist<minHouseDis){
                srcvertex=*vi;
                minHouseDis=newHDist;
        }
        float newJDist=(jP-roadVert).lengthSquared();
        if(newJDist<minJobDis){
                tgtvertex=*vi;
                minJobDis=newJDist;
        }
    }

    // Data
    trafficPersonVec[p].init_intersection=srcvertex;
    trafficPersonVec[p].end_intersection=tgtvertex;
    trafficPersonVec[p].time_departure=goToWork*3600.0f;//seconds
    //printf("Person %d: init %u end %u Time %f\n",p,srcvertex,tgtvertex,goToWork);

    // Status
    qsrand(p);

    trafficPersonVec[p].a=1.0f+((float)qrand())/RAND_MAX;//acceleration 1-2m/s2
    //if(((float)qrand())/RAND_MAX<0.1)trafficPersonVec[p].a=0; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    trafficPersonVec[p].b=1.0f+((float)qrand())/RAND_MAX;//break 1-2m/s2
    trafficPersonVec[p].T=0.8f+1.2f*(((float)qrand())/RAND_MAX);//time heading 0.8-2s
    //printf("Person %d: a %.10f b %.10f T %.10f\n",p,trafficPersonVec[p].a,trafficPersonVec[p].b,trafficPersonVec[p].T);
    //trafficPersonVec[p].a=((trafficPersonVec[p].a)/LC::misctools::Global::global()->cuda_cell_size)*LC::misctools::Global::global()->cuda_delta_time*LC::misctools::Global::global()->cuda_delta_time;
    //trafficPersonVec[p].b=((trafficPersonVec[p].b)/LC::misctools::Global::global()->cuda_cell_size)*LC::misctools::Global::global()->cuda_delta_time*LC::misctools::Global::global()->cuda_delta_time;
    //trafficPersonVec[p].T=((trafficPersonVec[p].T)/LC::misctools::Global::global()->cuda_delta_time)/3600.0f;
    //printf("Person %d:--> a %.10f b %.10f T %.10f\n",p,trafficPersonVec[p].a,trafficPersonVec[p].b,trafficPersonVec[p].T);
    trafficPersonVec[p].v=0;
    trafficPersonVec[p].num_steps=0;
    trafficPersonVec[p].gas=0;
    trafficPersonVec[p].active=0;
    trafficPersonVec[p].numOfLaneInEdge=0;
    trafficPersonVec[p].color=p<<8;
    //trafficPersonVec[p].color=(((float)qrand())/RAND_MAX)*0xFFFFFFFF;
    trafficPersonVec[p].LC_stateofLaneChanging=0;
    trafficPersonVec[p].personPath[0]=-1;*/

  }

  if (PERSON_DEBUG) {
    printf(">> >> PeopleTraffic %d generated in %d ms\n", numberPerGen,
           timer.elapsed());
  }

  if (PERSON_DEBUG) {
    printf("<< CUDA generateTrafficPersonJob\n");
  }

  //generateRoutes(roadGraph);
}//

//save initial edge to be able to reset the people to run again from the begining
/*void CUDAPMTrafficPersonJob::backUpTrafficPersonJob(std::vector<CUDATrafficPerson>& trafficPersonVec){
	printf("backUpTrafficPersonJob\n");
	backUpInitEdge.resize(trafficPersonVec.size());
	for(int p=0;p<trafficPersonVec.size();p++){
		backUpInitEdge[p]=trafficPersonVec[p].nextPathEdge;
	}
}//*/

void CUDAPMTrafficPersonJob::resetTrafficPersonJob(
  std::vector<CUDATrafficPerson> &trafficPersonVec) {
  if (PERSON_DEBUG) {
    printf("trafficPersonVec %d backUpInitEdge %d\n", trafficPersonVec.size(),
           backUpInitEdge.size());
  }

  for (int p = 0; p < trafficPersonVec.size(); p++) {
    /*trafficPersonVec[p].v=0;
    trafficPersonVec[p].num_steps=0;
    trafficPersonVec[p].gas=0;*/
    trafficPersonVec[p].active = 0;
    /*trafficPersonVec[p].numOfLaneInEdge=0;
    if(backUpInitEdge.size()==trafficPersonVec.size()){//back to first edge
        trafficPersonVec[p].nextPathEdge=backUpInitEdge[p];
    }else{
        printf("Error backUpInitEdge\n");
    }*/
  }
}//

}


