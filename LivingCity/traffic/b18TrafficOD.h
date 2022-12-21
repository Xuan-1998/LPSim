
/************************************************************************************************
*
*		Procedural Machine Traffic Person B2018
*
*		@desc Class that generates procedually a traffic person
*		@author igaciad
*
************************************************************************************************/

#ifndef LC_B18_PM_TRAFFIC_PERSON_H
#define LC_B18_PM_TRAFFIC_PERSON_H

#include "../misctools/misctools.h"

#include <QtGlobal>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "b18TrafficPerson.h"
#include "RoadGraph/roadGraph.h"
#include "sp/graph.h"

#include <boost/graph/dijkstra_shortest_paths.hpp>
#ifdef B18_RUN_WITH_GUI
#include "../VBOPeopleJobInfoLayer.h"
#endif
#include <random>

namespace LC {
class B18TrafficOD {

 public:
  B18TrafficOD(const parameters & inputSimParameters);
  ~B18TrafficOD();

  cv::Mat *peopleDistribution;
  cv::Mat *jobDistribution;
  const parameters simParameters;

  void randomPerson(int p, B18TrafficPerson &person, QVector3D housePos3D,
                    QVector3D jobPos3D, float startTimeH,
                    LC::RoadGraph::roadBGLGraph_BI &roadGraph);
  void randomPerson(int p, B18TrafficPerson &person, uint srcvertex,
                    uint tgtvertex, float startTimeH);

#ifdef B18_RUN_WITH_GUI
  void sampleDistribution(int numberToSample, PeopleJobOneLayer &distribution,
                          std::vector<QVector2D> &samples, QString &name);
  // generate random
  void createRandomPeople(int numberPerGen,
                          float startTimeH, float endTimeH,
                          std::vector<B18TrafficPerson> &trafficPersonVec,
                          PeopleJobInfoLayers &simPeopleJobInfoLayersn,
                          LC::RoadGraph::roadBGLGraph_BI &roadGraph);
#endif

  // generate from b18
  void loadB18TrafficPeople(float startTimeH, float endTimeH,
      std::vector<B18TrafficPerson> &trafficPersonVec, RoadGraph::roadBGLGraph_BI &roadGraph,
      const int limitNumPeople, const bool addRandomPeople);

  void loadB18TrafficPeopleSP(float startTimeH, float endTimeH,
    std::vector<B18TrafficPerson> &trafficPersonVec, // out
    const std::shared_ptr<abm::Graph>& graph_, const int limitNumPeople, const bool addRandomPeople,
    const std::vector<float> dep_times);

  void resetTrafficPersonJob(std::vector<B18TrafficPerson> &trafficPersonVec);
};
}

#endif  // LC_B18_PM_TRAFFIC_PERSON_H
