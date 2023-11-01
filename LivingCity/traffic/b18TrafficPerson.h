/************************************************************************************************
*
*		LC Project - B18 Traffic Person
*
*
*		@desc Class that contains the info of a person
*		@author igaciad
*
************************************************************************************************/

#ifndef LC_B18_TRAFFIC_PERSON_H
#define LC_B18_TRAFFIC_PERSON_H

namespace LC {

struct B18TrafficPerson {
  unsigned int init_intersection;
  unsigned int end_intersection;
  float time_departure;
  float dist_traveled = 0;
  float last_time_simulated = 0;

  unsigned short active;//0 inactive 1 active 2 finished
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
  unsigned short nextEdgeNumLanes;
  unsigned short nextEdgeNextInters;
  float nextEdgeLength;
  float nextEdgemaxSpeedMperSec;
  ///////////////////////////
  unsigned int indexPathInit;
  unsigned int indexPathCurr;
  unsigned int path_length_cpu; // not including END_OF_PATH value
  unsigned int path_length_gpu; // not including END_OF_PATH value

  //for edge speed calculations
  unsigned int currentEdge;
  unsigned int nextEdge;
  unsigned int prevEdge;
  float start_time_on_prev_edge;
  float end_time_on_prev_edge;
  float manual_v;

  // data
  unsigned short num_steps;
  unsigned int color;
  float co;
  float gas;
  // IDM
  float v;//current velocity
  float a;//acceleration
  float b;//break
  float T;// Time heading
  float cum_v = 0; //Cumulative velocity of each person across all iterations

  // lane changing
  unsigned short LC_initOKLanes;
  unsigned short LC_endOKLanes;
  unsigned short LC_stateofLaneChanging;

  int isInIntersection;
};
struct B18TrafficPersonModify{
  bool ifToCopy;
  bool ifToRemove;
  int gpuIndexToCopy;
};
}

#endif  // LC_B18_TRAFFIC_PERSON_H
