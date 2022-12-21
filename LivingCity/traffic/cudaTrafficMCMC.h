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
*
*		LC Project - Traffic MCMC
*
*
*		@desc Class that runs a chain of MCMC
*		@author igaciad
*
************************************************************************************************/

#ifndef LC_CUDA_TRAFFIC_MCMC_H
#define LC_CUDA_TRAFFIC_MCMC_H
//
//#include "../misctools/misctools.h"
//#include <QtGlobal>
//
//#include "cudaTrafficPerson.h"
//#include "RoadGraph/roadGraph.h"
//
//#include "PM/pmRoads.h"
//
//#include "cudaTrafficSimulator.h"
//
//
//#include <random>//normal distribution
//
//namespace LC {
//
//	struct CUDADesignOptions{
//		int budget;
//		bool laneDirection;
//		bool addLanes;
//		bool removeLanes;
//		bool jobsDist;
//		bool peoplDist;
//		CUDADesignOptions(int _budget,bool _laneDirection,bool _addLanes,bool _removeLanes,bool _jobsDist,bool _peoplDist){
//			budget=_budget;laneDirection=_laneDirection;addLanes=_addLanes;removeLanes=_removeLanes;jobsDist=_jobsDist,peoplDist=_peoplDist;
//		}
//		CUDADesignOptions(){}
//	};
//
//
//	class CUDATrafficMCMC : public QObject
//	{
//		Q_OBJECT
//	public:
//		CUDATrafficMCMC();
//		~CUDATrafficMCMC();
//
//		typedef boost::geometry::model::d2::point_xy<double> mPoint;
//		typedef boost::geometry::model::polygon<mPoint> polygon;
//
//		// QTHREAD
//		void initChain(int threadNum,int _numChainsToRun,CUDATrafficSimulator* originalSimulator,float temperature,int numSteps,int _numPasses,int indexToEvaluate,CUDADesignOptions _designOptions,std::vector<std::vector<polygon>*>& allVec,PlaceTypesMainClass* currentPlaceTypes);
//		void initMinCost(float scoreGoal);
//		//std::vector<std::vector<std::vector<std::set<LC::RoadGraph::roadGraphEdgeDesc_BI>>>>& edgesPerGroupExpanded,
//		//CUDATrafficSimulator getSimulator();
//
//		int threadNum;
//		int numChainsToRun;
//		float currentScore;
//		float initialScore;
//		bool done;
//
//		// score goal
//		float scoreGoal;
//		float minCost;
//		float currCost;
//		std::vector<float> costWeight;
//
//		//best
//		float bestScore;
//		CUDATrafficSimulator* bestSimulationState;
//
//		CUDATrafficSimulator* originalSimulationState;
//		CUDATrafficSimulator* candidateTrafficSimulator;
//		CUDATrafficSimulator* currentSimState;
//
//	public slots:
//			void process();
//			void processMinCost();
//
//	private:
//		// VARIABLES
//
//		//different regions
//
//		int indexToEvaluate;
//		float temperature;
//		int numSteps;
//		int numPasses;
//		CUDADesignOptions designOptions;
//
//		//keep user selection
//		void calculateEdgePerGroupExpanded();
//		std::vector<std::vector<polygon>*> allVec;
//		std::vector<std::vector<std::vector<std::set<LC::RoadGraph::roadGraphEdgeDesc_BI>>>> edgesPerGroupExpanded;
//
//		//placetype
//		PlaceTypesMainClass* currentPlaceTypes;
//		PlaceTypesMainClass* candidatePlaceTypes;
//
//		// METHODS
//		void evaluateNetwork(std::vector<float>& result,bool diffAmpl=false,bool diffLanes=false);
//		void generateCandidateState();
//		void simulateCandidateState();
//		float computeAcceptanceProbability(float currentScore,float candidateScore,float temperature);
//
//		//modify
//		bool transferLaneInEdge(LC::RoadGraph::roadGraphEdgeDesc_BI edge,int state);
//		void transformDistribution();
//signals:
//		void newResult(int threadNum);
//		void finished();
//		void error(QString err);
//	};
//}

#endif
