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
*		@desc Class that contains the info of a person
*		@author igaciad
************************************************************************************************/

#pragma once

#include <vector>
#include <fstream>
#include <iterator>

struct BTrafficPeopleCUDA {
  unsigned short *init_intersection;
  unsigned short *end_intersection;
  float *time_departure;
  unsigned char *active;
  float *v;
  float *a;
  float *b;
  float *T;
  unsigned char *carType;

  unsigned int *indTo1stEdge;
  unsigned short *nextEdge;

  // SIMULATION
  unsigned int *currIndEdge;
  float *posInLaneC;
  unsigned char *laneInEdge;
};


namespace LC {

struct SimulationSt {
  std::vector<int> threadFinished;
  int numPeopleFinished;
  float currentTime;
  //unsigned int mapToReadShiftL;
  //unsigned int mapToWriteShiftL;
  bool cArray;
};

class BTrafficPeople {
 public:

  BTrafficPeople() {
    numPeople = 0;
  }

  int numPeople;

  std::vector<unsigned short> init_intersection;
  std::vector<unsigned short> end_intersection;
  std::vector<float> time_departure;

  std::vector<unsigned char> active;//0 inactive 1 active 2 finished

  // IDM
  std::vector<float> v;//current velocity
  std::vector<float> a;//acceleration
  std::vector<float> b;//break
  std::vector<float> T;// Time heading
  std::vector<unsigned char> carType;

  // route
  std::vector<unsigned int> indTo1stEdge;//index to first edge in nextEdgeArray
  std::vector<unsigned short> nextEdge;
  std::vector<unsigned char> edgeExitOut;
  std::vector<unsigned char> edgeExitIn;

  ////////////////////////////////////////////
  // SIMULATION: Variables used in simulation
  std::vector<unsigned int> currIndEdge;//index to current edge in nextEdgeArray
  //std::vector<unsigned short> currEdge;

  std::vector<float> posInLaneC;
  std::vector<unsigned char> laneInEdge;

  /*unsigned short active;
  unsigned short numOfLaneInEdge;//number of lane in that edge

  float posInLaneM;

  //////////////////////////
  // current edge (from edgeData)
  unsigned short edgeNumLanes;//total number of lanes in that edge
  unsigned short edgeNextInters;
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


  unsigned short currPathEdge;
  unsigned short personPath[80];//change CUDATrafficPersonShortestPath::calculateSeveralPeopleRoute
  // data
  unsigned short num_steps;
  unsigned int color;
  float gas;


  // lane changing
  unsigned short LC_initOKLanes;
  unsigned short LC_endOKLanes;
  unsigned short LC_stateofLaneChanging;

  int isInIntersection;*/

  //////////////////////////////////////////////////
  // FUNCTIONS

  void clear() {
    numPeople = 0;
    init_intersection.clear();
    end_intersection.clear();
    time_departure.clear();
    active.clear();
    v.clear();
    a.clear();
    b.clear();
    T.clear();
    carType.clear();

    indTo1stEdge.clear();
    //nextEdge.clear();
  }//

  bool checkAllSameSize() {
    bool same = (
                  numPeople == init_intersection.size() &&
                  numPeople == end_intersection.size() &&
                  numPeople == time_departure.size() &&
                  numPeople == active.size() &&
                  numPeople == v.size() &&
                  numPeople == a.size() &&
                  numPeople == b.size() &&
                  numPeople == T.size() &&
                  numPeople == carType.size() &&
                  numPeople == indTo1stEdge.size()
                );//could negate

    if (same == false) {
      printf("checkAllSameSize FALSE: %d | %d %d %d %d %d %d %d %d %d %d\n",
             numPeople, init_intersection.size(), end_intersection.size(),
             time_departure.size(), active.size(), v.size(), a.size(), b.size(), T.size(),
             carType.size(), indTo1stEdge.size());
    }

    return same;
  }//

  void resize(int size) {
    numPeople = size;

    init_intersection.resize(size);
    end_intersection.resize(size);
    time_departure.resize(size);
    active.resize(size);
    v.resize(size);
    a.resize(size);
    b.resize(size);
    T.resize(size);
    carType.resize(size);

    indTo1stEdge.resize(size);
    //nextEdge.resize(size);

  }//
  template <typename T>
  void fileWriteArray(std::vector<T> &vec, std::string fileName) {
    std::ofstream ofs(fileName.c_str(), std::ios::out | std::ofstream::binary);
    //std::copy(vec.begin(), vec.end(), std::ostreambuf_iterator<T>(ofs));
    //std::copy(vec.begin(), vec.end(), std::ostreambuf_iterator<char>(ofs));
    ofs.write(reinterpret_cast<const char *>(&vec[0]), vec.size()*sizeof(T));
    ofs.close();
  }//

  template <typename T>
  void fileReadArray(std::vector<T> &vec, std::string fileName) {
    std::ifstream ifs(fileName.c_str(), std::ios::in | std::ifstream::binary);
    /*std::copy(
        std::istream_iterator<T>(ifs),
        std::istream_iterator<T>(),
        std::back_inserter<std::vector<T>>(vec));*/
    T f;

    while (ifs.read(reinterpret_cast<char *>(&f), sizeof(f))) {
      vec.push_back(f);
    }

    ifs.close();
  }//


  void saveToFile() {

    fileWriteArray<unsigned short>(init_intersection,
                                   "data/people/init_intersection.dat");
    fileWriteArray<unsigned short>(end_intersection,
                                   "data/people/end_intersection.dat");
    fileWriteArray<float>(time_departure, "data/people/time_departure.dat");

    fileWriteArray<unsigned char>(active, "data/people/active.dat");

    fileWriteArray<float>(v, "data/people/v.dat");
    fileWriteArray<float>(a, "data/people/a.dat");
    fileWriteArray<float>(b, "data/people/b.dat");
    fileWriteArray<float>(T, "data/people/T.dat");

    fileWriteArray<unsigned char>(carType, "data/people/carType.dat");

    fileWriteArray<unsigned int>(indTo1stEdge, "data/people/indTo1stEdge.dat");
    fileWriteArray<unsigned short>(nextEdge, "data/people/nextEdge.dat");
    fileWriteArray<unsigned char>(edgeExitOut, "data/people/edgeExitOut.dat");
    fileWriteArray<unsigned char>(edgeExitIn, "data/people/edgeExitIn.dat");
  }//

  void loadFromFile() {
    clear();

    fileReadArray<unsigned short>(init_intersection,
                                  "data/people/init_intersection.dat");
    fileReadArray<unsigned short>(end_intersection,
                                  "data/people/end_intersection.dat");
    fileReadArray<float>(time_departure, "data/people/time_departure.dat");

    fileReadArray<unsigned char>(active, "data/people/active.dat");

    fileReadArray<float>(v, "data/people/v.dat");
    fileReadArray<float>(a, "data/people/a.dat");
    fileReadArray<float>(b, "data/people/b.dat");
    fileReadArray<float>(T, "data/people/T.dat");
    fileReadArray<unsigned char>(carType, "data/people/carType.dat");

    fileReadArray<unsigned int>(indTo1stEdge, "data/people/indTo1stEdge.dat");
    fileReadArray<unsigned short>(nextEdge, "data/people/nextEdge.dat");
    fileReadArray<unsigned char>(edgeExitOut, "data/people/edgeExitOut.dat");
    fileReadArray<unsigned char>(edgeExitIn, "data/people/edgeExitIn.dat");

    numPeople = init_intersection.size();
    printf("Read %d %d %d %d %d %d %d %d %d\n", init_intersection.size(),
           end_intersection.size(), time_departure.size(), active.size(), v.size(),
           a.size(), b.size(), T.size(), carType.size());
  }//
};

}
