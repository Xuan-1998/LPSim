/************************************************************************************************
 *
 *		CUDA hearder
 *
 *		@author igarciad
 *
 ************************************************************************************************/

#ifndef B18_TRAFFIC_SIMULATION_H
#define B18_TRAFFIC_SIMULATION_H

#include "b18TrafficPerson.h"
#include "b18EdgeData.h"
#include <vector>


extern void b18InitCUDA_n(bool fistInitialization, // crate buffers
                        std::vector<int>& vertexIdToPar,
                        bool* edgeIfGhost,
                        int edges_num,
                        std::vector<LC::B18TrafficPerson> &trafficPersonVec,
                        std::vector<uint> indexPathVec_n[],
                        std::vector<LC::B18EdgeData> edgesData_n[], std::vector<uchar> laneMap_n[],
                        std::vector<uchar> trafficLights_n[],
                        std::vector<LC::B18IntersectionData> intersections_n[],
                        float startTimeH, float endTimeH,
                        std::vector<float>& accSpeedPerLinePerTimeInterval,
                        std::vector<float>& numVehPerLinePerTimeInterval,
                        float deltaTime);
                        
extern void b18InitCUDA(bool fistInitialization, // crate buffers
                        std::vector<LC::B18TrafficPerson> &trafficPersonVec,
                        std::vector<uint> &indexPathVec,
                        std::vector<LC::B18EdgeData> &edgesData, std::vector<uchar> &laneMap,
                        std::vector<uchar> &trafficLights,
                        std::vector<LC::B18IntersectionData> &intersections,
                        float startTimeH, float endTimeH,
                        std::vector<float>& accSpeedPerLinePerTimeInterval,
                        std::vector<float>& numVehPerLinePerTimeInterval,
                        float deltaTime);
extern void b18updateStructuresCUDA(std::vector<LC::B18TrafficPerson>& trafficPersonVec,
                                    std::vector<uint> &indexPathVec, 
                                    std::vector<LC::B18EdgeData>& edgesData);
extern void b18updateStructuresCUDA_n(std::vector<int>& vertexIdToPar,std::vector<LC::B18TrafficPerson>& trafficPersonVec,
                                    std::vector<uint> &indexPathVec, 
                                    std::vector<LC::B18EdgeData> edgesData[]);
extern void b18GetDataCUDA(std::vector<LC::B18TrafficPerson> &trafficPersonVec, std::vector<LC::B18EdgeData> &edgesData);
extern void b18GetSampleTrafficCUDA(std::vector<float>& accSpeedPerLinePerTimeInterval, 
                                std::vector<float>& numVehPerLinePerTimeInterval);
extern void b18FinishCUDA(void); // free memory
extern void b18ResetPeopleLanesCUDA(uint numPeople); // reset people to inactive
extern void b18SimulateTrafficCUDA(float currentTime, uint numPeople,
                                   uint numIntersections_n[], float deltaTime, const parameters simParameters,
                                   int numBlocks, int threadsPerBlock);

#endif // B18_TRAFFIC_SIMULATION_H

