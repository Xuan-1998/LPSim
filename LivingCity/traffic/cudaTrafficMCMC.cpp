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

//#include "cudaTrafficMCMC.h"
//#include "client_main.h"
//
//#define MCMC_DEBUG 0
//#define MCMC_GENERATE 0
//
//namespace LC {
//
//	extern ClientMain *clientMain;
//
//	const float maxDepth=10;//same than in change radious size
//
//
//	CUDATrafficMCMC::CUDATrafficMCMC(){
//		bestSimulationState=0;
//		originalSimulationState=0;
//		candidateTrafficSimulator=0;
//		currentSimState=0;
//
//	}//
//	CUDATrafficMCMC::~CUDATrafficMCMC(){
//	}//
//
//	// INIT CHAIN INFO--> Simulator, User Selection, temperature
//	void CUDATrafficMCMC::initChain(
//			int _threadNum,
//			int _numChainsToRun,
//			CUDATrafficSimulator* _originalSimulator,
//			//std::vector<std::vector<std::vector<std::set<LC::RoadGraph::roadGraphEdgeDesc_BI>>>>& _edgesPerGroupExpanded,
//			float _temperature,
//			int _numSteps,
//			int _numPasses,
//			int _indexToEvaluate,
//			CUDADesignOptions _designOptions,
//			std::vector<std::vector<polygon>*>& _allVec,
//			PlaceTypesMainClass* _currentPlaceTypes
//		){
//		threadNum=_threadNum;
//		numChainsToRun=_numChainsToRun;
//		//copy simulator
//		bestSimulationState=new CUDATrafficSimulator(*_originalSimulator);
//		bestSimulationState->simRoadGraph=new RoadGraph(*_originalSimulator->simRoadGraph);
//		bestSimulationState->threadNumber=_threadNum;
//
//		originalSimulationState=new CUDATrafficSimulator(*_originalSimulator);
//		originalSimulationState->simRoadGraph=new RoadGraph(*_originalSimulator->simRoadGraph);
//		originalSimulationState->threadNumber=_threadNum;
//		//printf("MCMC Orig %d Copied %d\n",boost::num_edges(originalSimulator.simRoadGraph->myRoadGraph_BI),boost::num_edges(chainTrafficSimulator->simRoadGraph->myRoadGraph_BI));
//
//		temperature=_temperature;
//		numSteps=_numSteps;
//		numPasses=_numPasses;
//		indexToEvaluate=_indexToEvaluate;
//		designOptions=_designOptions;
//		allVec=_allVec;
//		currentPlaceTypes=new PlaceTypesMainClass(*_currentPlaceTypes);//copy
//		candidatePlaceTypes=0;
//	}//
//
//	void CUDATrafficMCMC::process() {
//		bool optimizeCity=false;
//		bool transformDistr=false;
//		done=false;
//		bestScore=FLT_MAX;
//		initialScore=FLT_MAX;
//		minCost=FLT_MAX;
//		// 1. Calculate Initial Score: set candidate state to be able to calculate score
//		candidateTrafficSimulator=originalSimulationState;
//		calculateEdgePerGroupExpanded();
//		std::vector<float> results;
//		if(optimizeCity==true){
//			initialScore=initialScore=currentScore=FLT_MAX;
//
//		}else{
//			evaluateNetwork(results);//calculate initial score
//			initialScore=results[0];
//			bestScore=initialScore;
//			currentScore=initialScore;
//		}
//		emit newResult(threadNum);//to set up initial value
//		candidateTrafficSimulator=0;
//
//		for(int numChain=0;numChain<numChainsToRun&&done==false;numChain++){
//			// reset score and initial current state
//			currentScore=initialScore;
//			if(currentSimState!=0){//delete if exits
//				delete currentSimState->simRoadGraph;
//				delete currentSimState;
//				currentSimState=0;
//			}
//			currentSimState=new CUDATrafficSimulator(*originalSimulationState);
//			currentSimState->simRoadGraph=new RoadGraph(*originalSimulationState->simRoadGraph);
//
//			if(transformDistr==true){
//				printf("++[T%d][C%d] transformDistribution\n",threadNum,numChain);
//				transformDistribution();
//			}
//
//			//candidateState=0;
//			printf("[T%d][C%d] MCMC initScore %f\n",threadNum,numChain,initialScore);
//
//			// 2. Over all steps
//			for(int step=0;step<numSteps&&done==false;step++){
//
//				// 2.1 generate candidate state
//				if(MCMC_DEBUG)printf("[T%d][C%d] MCMC 2.1 Generate\n",threadNum,numChain);
//				generateCandidateState();
//				// 2.2 simulate
//				if(MCMC_DEBUG)printf("[T%d][C%d] MCMC 2.2 Simulate\n",threadNum,numChain);
//				simulateCandidateState();
//				// 2.3 calculate new score
//				if(MCMC_DEBUG)printf("[T%d][C%d] MCMC 2.3 Evaluate",threadNum,numChain);
//				float candidateScore;
//				if(optimizeCity==false){
//					evaluateNetwork(results,true,true);
//					candidateScore=results[0];
//				}else{
//					candidateScore=candidateTrafficSimulator->avgTravelTime;
//				}
//				//candidateScore=0;///!!!
//
//				printf("[T%d][C%d] Score Old %f Candidate %f\n",threadNum,numChain,currentScore,candidateScore);
//				// 2.4 MCMC
//				printf("[T%d][C%d] MCMC 2.4 MCMC\n",threadNum,numChain);
//				// 2.4.1 check if it is the best state and if it is the case, save it
//				if(bestScore>candidateScore){
//					bestScore=candidateScore;
//
//					if(bestSimulationState!=0){//there were a former best state
//						delete bestSimulationState->simRoadGraph;
//						delete bestSimulationState;
//						bestSimulationState=0;
//					}
//					// copy candidate to best simulation
//					bestSimulationState=new CUDATrafficSimulator(*candidateTrafficSimulator);
//					bestSimulationState->simRoadGraph=new RoadGraph(*candidateTrafficSimulator->simRoadGraph);
//				}
//				float acceptProb = computeAcceptanceProbability( currentScore, candidateScore, this->temperature );
//				if(acceptProb>(((float)qrand())/RAND_MAX)){
//					printf("[T%d][C%d] MCMC 2.4.1 Acepted New %f\n",threadNum,numChain,candidateScore);
//					// score
//					currentScore=candidateScore;//update score
//					//roadgraph
//					if(currentSimState!=0){//delete if exits
//						delete currentSimState->simRoadGraph;
//						delete currentSimState;
//						currentSimState=0;
//					}
//					currentSimState=candidateTrafficSimulator;
//					candidateTrafficSimulator=0;
//					//placetypes
//					if(candidatePlaceTypes!=0){//was change and we should update it
//						delete currentPlaceTypes;
//						currentPlaceTypes=candidatePlaceTypes;
//						candidatePlaceTypes=0;
//					}
//
//					printf("[T%d][C%d] ****Emit new result %f best %f\n",threadNum,numChain,currentScore,bestScore);
//					emit newResult(threadNum);
//					if(currentScore==0)
//						break;// FOUND PERFECT SOLUTION
//				}else{
//					//reject
//					if(candidateTrafficSimulator!=0){//delete if exits
//						delete candidateTrafficSimulator->simRoadGraph;
//						delete candidateTrafficSimulator;
//						candidateTrafficSimulator=0;
//					}
//				}
//			}
//		}
//		done=true;
//		emit newResult(-1);
//		emit finished();
//	}//
//
//	void CUDATrafficMCMC::initMinCost(float _scoreGoal){
//		//costWeight=_costWeight;
//		scoreGoal=_scoreGoal;
//		minCost=FLT_MAX;
//	}//
//
//	void CUDATrafficMCMC::processMinCost() {
//		done=false;
//		bestScore=FLT_MAX;
//		initialScore=FLT_MAX;
//		minCost=FLT_MAX;
//		// 1. Calculate Initial Score: set candidate state to be able to calculate score
//		candidateTrafficSimulator=originalSimulationState;
//		calculateEdgePerGroupExpanded();
//		std::vector<float> results;
//		evaluateNetwork(results);//calculate initial score
//		initialScore=results[0];
//		bestScore=initialScore;
//		currentScore=initialScore;
//		emit newResult(threadNum);//to set up initial value
//		candidateTrafficSimulator=0;
//
//		for(int numChain=0;numChain<numChainsToRun&&done==false;numChain++){
//			// reset score and initial current state
//			currentScore=initialScore;
//			currCost=0;
//			if(currentSimState!=0){//delete if exits
//				delete currentSimState->simRoadGraph;
//				delete currentSimState;
//				currentSimState=0;
//			}
//			currentSimState=new CUDATrafficSimulator(*originalSimulationState);
//			currentSimState->simRoadGraph=new RoadGraph(*originalSimulationState->simRoadGraph);
//
//			//candidateState=0;
//			printf("[T%d][C%d] MCMC initScore S %f\n",threadNum,numChain,initialScore);
//			// 2. Over all steps
//			for(int step=0;step<numSteps&&done==false;step++){
//
//				// 2.1 generate candidate state
//				if(MCMC_DEBUG)printf("[T%d][C%d] MCMC 2.1 Generate\n",threadNum,numChain);
//				generateCandidateState();
//				// 2.2 simulate
//				if(MCMC_DEBUG)printf("[T%d][C%d] MCMC 2.2 Simulate\n",threadNum,numChain);
//				simulateCandidateState();
//				// 2.3 calculate new score
//				if(MCMC_DEBUG)printf("[T%d][C%d] MCMC 2.3 Evaluate",threadNum,numChain);
//				float candidateScore;
//
//					evaluateNetwork(results,true,true);
//					candidateScore=results[0];
//
//
//				// 2.4 MCMC
//				//printf("[T%d][C%d] MCMC 2.4 MCMC\n",threadNum,numChain);
//				// 2.4.1 check if it is the best state and if it is the case, save it
//				float accCost=0;
//				accCost+=results[8];
//				//accCost+=results[8]/boost::num_edges(candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI);//change of lanes
//				//accCost+=results[7];//change of people
//
//				printf("-->>>[T%d][C%d] S %f %f C %f %f\n",threadNum,numChain,currentScore,candidateScore,currCost,accCost);
//				printf("--<><> %f,%f\n",candidateScore,accCost);
//				//best if
//				//best if
//				//		we have not reach the goal score and we are improving it
//				//		we have reach the goal score and the cost is smaller
//				if((bestScore>scoreGoal&&bestScore>candidateScore)||
//					(candidateScore<=scoreGoal&&accCost<minCost)){
//						bestScore=candidateScore;
//						minCost=accCost;
//
//						if(bestSimulationState!=0){//there were a former best state
//							delete bestSimulationState->simRoadGraph;
//							delete bestSimulationState;
//							bestSimulationState=0;
//						}
//						// copy candidate to best simulation
//						bestSimulationState=new CUDATrafficSimulator(*candidateTrafficSimulator);
//						bestSimulationState->simRoadGraph=new RoadGraph(*candidateTrafficSimulator->simRoadGraph);
//
//						printf("[T%d][C%d] BEST ****Emit new result %f best %f COST %f\n",threadNum,numChain,currentScore,bestScore,accCost);
//						emit newResult(threadNum);
//
//				}
//
//				float acceptProb = computeAcceptanceProbability( currentScore, candidateScore, this->temperature );
//				float acceptProbCost = computeAcceptanceProbability( currCost, accCost, this->temperature );
//
//				if((currentScore>scoreGoal&&acceptProb>(((float)qrand())/RAND_MAX))||//like if it were the normal score
//					(currentScore<=scoreGoal&&accCost<=scoreGoal&&acceptProbCost>(((float)qrand())/RAND_MAX))||//try to improve
//					(bestScore>scoreGoal&&candidateScore<=scoreGoal)){//first within score goal
//				//if(accCost<minCost&&acceptProb>(((float)qrand())/RAND_MAX)){
//					printf("[T%d][C%d] MCMC 2.4.1 Acepted OLD S %f C %f --> New S %f C %f\n",threadNum,numChain,currentScore,currCost,candidateScore,accCost);
//					// score
//					currentScore=candidateScore;//update score
//					currCost=accCost;
//					//roadgraph
//					if(currentSimState!=0){//delete if exits
//						delete currentSimState->simRoadGraph;
//						delete currentSimState;
//						currentSimState=0;
//					}
//					currentSimState=candidateTrafficSimulator;
//					candidateTrafficSimulator=0;
//					//placetypes
//					if(candidatePlaceTypes!=0){//was change and we should update it
//						delete currentPlaceTypes;
//						currentPlaceTypes=candidatePlaceTypes;
//						candidatePlaceTypes=0;
//					}
//
//					//printf("[T%d][C%d] ****Emit new result %f best %f\n",threadNum,numChain,currentScore,bestScore);
//					emit newResult(threadNum);
//					if(currentScore==0&&accCost==0)
//						break;// FOUND PERFECT SOLUTION
//				}else{
//					printf("[T%d][C%d] MCMC 2.4.1 Reject OLD S %f C %f <-- New S %f C %f\n",threadNum,numChain,currentScore,currCost,candidateScore,accCost);
//					//reject
//					if(candidateTrafficSimulator!=0){//delete if exits
//						delete candidateTrafficSimulator->simRoadGraph;
//						delete candidateTrafficSimulator;
//						candidateTrafficSimulator=0;
//					}
//				}
//			}
//		}
//		done=true;
//		emit newResult(-1);
//		emit finished();
//	}//
//
//	void bsfFromEdgeHash(RoadGraph& inRoadGraph,LC::RoadGraph::roadGraphEdgeDesc_BI& edge,int design_state,std::set<LC::RoadGraph::roadGraphEdgeDesc_BI>& processed,std::set<LC::RoadGraph::roadGraphEdgeDesc_BI>& nextEdges,int currentDistance){
//
//		// check not block
//		if(inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==0)
//			return;
//		// check not other area
//		if(design_state==1&&(inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==2||inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==3))
//			return;
//		if(design_state==2&&(inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==1||inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==3))
//			return;
//		if(design_state==3&&(inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==1||inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==2))
//			return;
//		// check it does not have a smaller value already
//		if(inRoadGraph.myRoadGraph_BI[edge].cuda_design_state!=0xFFFF&&
//			inRoadGraph.myRoadGraph_BI[edge].cuda_design_state>=10&&(inRoadGraph.myRoadGraph_BI[edge].cuda_design_state%10)>currentDistance)
//			return;
//
//		// 2. expand for both sides
//		LC::RoadGraph::roadGraphVertexDesc_BI ver[2];
//		ver[0]=boost::source(edge, inRoadGraph.myRoadGraph_BI);
//		ver[1]=boost::target(edge, inRoadGraph.myRoadGraph_BI);
//		/*//mark twin edge as well
//		std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e0_pair=boost::edge(ver[1],ver[0],inRoadGraph.myRoadGraph_BI);
//		if(e0_pair.second==false){
//			printf("Error: Twin edge does not exits\n");
//		}else{
//			inRoadGraph.myRoadGraph_BI[e0_pair.first].cuda_design_visited=inRoadGraph.myRoadGraph_BI[edge].cuda_design_visited;//same state than twin
//			inRoadGraph.myRoadGraph_BI[e0_pair.first].cuda_design_state=inRoadGraph.myRoadGraph_BI[edge].cuda_design_state;
//		}
//		//check that we are not too far
//		if(currentDistance>=maxDistance)
//			return;*/
//		RoadGraph::out_roadGraphEdgeIter_BI Oei, Oei_end;
//		for(int verN=0;verN<2;verN++){
//			for(boost::tie(Oei, Oei_end) = boost::out_edges(ver[verN],inRoadGraph.myRoadGraph_BI); Oei != Oei_end; ++Oei){
//				if(processed.find(*Oei) == processed.end()){//not contain
//					if(inRoadGraph.myRoadGraph_BI[*Oei].cuda_design_state<10){//already labeled
//						continue;
//					}
//					if(currentDistance>=inRoadGraph.myRoadGraph_BI[*Oei].cuda_design_state%10){//already with lowerlabeled
//						continue;
//					}
//					nextEdges.insert(*Oei);
//					inRoadGraph.myRoadGraph_BI[*Oei].cuda_design_state=design_state*10+currentDistance;//10|30+distance
//					std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e0_pair=boost::edge(boost::target(*Oei, inRoadGraph.myRoadGraph_BI),boost::source(*Oei, inRoadGraph.myRoadGraph_BI),inRoadGraph.myRoadGraph_BI);
//					if(e0_pair.second==false){
//						//printf("Error: Twin edge does not exits\n");
//					}else{
//						inRoadGraph.myRoadGraph_BI[e0_pair.first].cuda_design_state=(design_state+3)*10+currentDistance;//40|60+distance
//					}
//				}
//			}
//		}
//		// IN those that are in but not out
//		RoadGraph::in_roadGraphEdgeIter_BI Iei, Iei_end;
//		for(int verN=0;verN<2;verN++){
//			for(boost::tie(Iei, Iei_end) = boost::in_edges(ver[verN],inRoadGraph.myRoadGraph_BI); Iei != Iei_end; ++Iei){
//				if(processed.find(*Iei) == processed.end()){//not contain
//					if(inRoadGraph.myRoadGraph_BI[*Iei].cuda_design_state<10){//already labeled
//						continue;
//					}
//					if(currentDistance>=inRoadGraph.myRoadGraph_BI[*Iei].cuda_design_state%10){//already with lowerlabeled
//						continue;
//					}
//
//					//inRoadGraph.myRoadGraph_BI[*Oei].cuda_design_state=design_state*10+currentDistance;//10|30+distance
//
//					std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e0_pair=boost::edge(boost::target(*Iei, inRoadGraph.myRoadGraph_BI),boost::source(*Iei, inRoadGraph.myRoadGraph_BI),inRoadGraph.myRoadGraph_BI);
//					if(e0_pair.second==false){//if there was not out
//						inRoadGraph.myRoadGraph_BI[*Iei].cuda_design_state=(design_state+3)*10+currentDistance;//40|60+distance
//						nextEdges.insert(*Iei);
//						//printf("Error: Twin edge does not exits\n");
//					}else{
//						//inRoadGraph.myRoadGraph_BI[e0_pair.first].cuda_design_state=(design_state+3)*10+currentDistance;//40|60+distance
//					}
//				}
//			}
//		}
//	}//
//
//	void CUDATrafficMCMC::calculateEdgePerGroupExpanded(){
//		if(MCMC_DEBUG)printf(">>calculateEdgePerGroupExpanded\n");
//		QTime timer;
//		timer.start();
//
//
//		////////////////////////////////////
//		/// 1. Label edges
//		if(MCMC_DEBUG)printf("1. Label edges\n");
//		QVector3D p0,p1;
//		int edgeLabel=0;
//
//		RoadGraph::roadGraphEdgeIter_BI ei, eiEnd;
//		std::vector<std::vector<std::set<LC::RoadGraph::roadGraphEdgeDesc_BI>>> edgesPerGroup;
//		edgesPerGroup.resize(allVec.size());
//		for(int sN=0;sN<allVec.size();sN++){
//			edgesPerGroup[sN].resize(allVec[sN]->size());
//		}
//		for(boost::tie(ei, eiEnd) = boost::edges(candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI);
//			ei != eiEnd; ++ei)
//		{
//			p0 = candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[boost::source(*ei, candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI)].pt;
//			p1 = candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[boost::target(*ei, candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI)].pt;
//			candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*ei].cuda_design_state=0xFFFF;
//			candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*ei].cuda_design_visited=0xFFFF;
//			//candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*ei].cuda_design_modified=0x0000;
//			bool found=false;
//			for(int aV=allVec.size()-1;aV>=0&&found==false;aV--){// prioretize block (0)
//				for(int b=0;b<allVec[aV]->size()&&found==false;b++){
//					bool containsP0=boost::geometry::within(mPoint(p0.x(),p0.y()),(allVec[aV])->at(b));
//					bool containsP1=boost::geometry::within(mPoint(p1.x(),p1.y()),(allVec[aV])->at(b));
//
//					if((containsP0==true&&containsP1==true)||
//						(aV==0&&(containsP0||containsP1))){
//							edgeLabel++;
//							found=true;
//							candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*ei].cuda_design_state=aV;
//							edgesPerGroup[aV][b].insert(*ei);
//							break;
//					}
//
//				}
//			}
//		}
//		////////////////////////////////////
//		// 2. Set starting edge (visualize) and expand BSF
//		//int maxDepth=8;//same than in LANE CHANGE
//		//std::vector<std::vector<std::vector<std::set<LC::RoadGraph::roadGraphEdgeDesc_BI>>>> edgesPerGroupExpanded;
//		edgesPerGroupExpanded.clear();
//		edgesPerGroupExpanded.resize(allVec.size());
//		for(int sN=0;sN<allVec.size();sN++){
//			edgesPerGroupExpanded[sN].resize(allVec[sN]->size());
//			for(int gNN=0;gNN<allVec[sN]->size();gNN++){
//				edgesPerGroupExpanded[sN][gNN].resize(maxDepth+1);
//			}
//		}
//		if(MCMC_DEBUG)printf("2. Expand\n");
//		for(int state=1;state<allVec.size();state++){//state=1--> skip block
//			for(int lT=0;lT<edgesPerGroup[state].size();lT++){
//				std::set<LC::RoadGraph::roadGraphEdgeDesc_BI> currentEdges= edgesPerGroup[state][lT];//initial set of edges
//				edgesPerGroupExpanded[state][lT][0]=edgesPerGroup[state][lT];//depth 0 is the inner edges
//				for(int bsfDepth=0;bsfDepth<maxDepth;bsfDepth++){
//					int edgeN=0;
//					for(std::set<LC::RoadGraph::roadGraphEdgeDesc_BI>::iterator it = edgesPerGroupExpanded[state][lT][bsfDepth].begin();
//						it != edgesPerGroupExpanded[state][lT][bsfDepth].end();) { // last set is the new one to loop
//							edgeN++;
//							LC::RoadGraph::roadGraphEdgeDesc_BI newEdge=*it;
//							// use edgesPerGroup to put all the edges
//							bsfFromEdgeHash(*candidateTrafficSimulator->simRoadGraph,newEdge,state,edgesPerGroup[state][lT],edgesPerGroupExpanded[state][lT][bsfDepth+1],bsfDepth);
//							++it;
//					}
//				}
//			}
//		}
//		if(MCMC_DEBUG)printf("<<calculateEdgePerGroupExpanded Time labeling: %d ms NumEdgeIndeGroups %d\n",timer.elapsed(),edgeLabel);
//	}//
//
//	float CUDATrafficMCMC::computeAcceptanceProbability(float currentScore,float candidateScore,float temperature){
//		float acceptanceProbability = 0.0f;
//
//		float curFun = exp( -temperature*currentScore );
//		float candidateFun = exp( -temperature*candidateScore );
//
//		acceptanceProbability = std::min( 1.0f, candidateFun/curFun );
//		return acceptanceProbability;
//	}//
//
//
//	bool CUDATrafficMCMC::transferLaneInEdge(LC::RoadGraph::roadGraphEdgeDesc_BI edge,int state){
//		// find twin
//		std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e_dir=boost::edge(boost::source(edge, candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI),boost::target(edge, candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI),candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI);
//		std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e_twi=boost::edge(boost::target(edge, candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI),boost::source(edge, candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI),candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI);
//		if(e_dir.second==false){
//			printf("Error transferLaneInEdge: Direct edge does not exits\n");
//			return false;
//		}
//		/*if(e_dir.second==false||e_twi.second==false){
//			printf("Error transferLaneInEdge: Twin edge does not exits\n");
//			return false;
//		}else{*/
//			bool transferForward;
//			if(e_twi.second==false){
//				transferForward=true;
//			}else{
//				float randN=((float)qrand())/RAND_MAX;
//				switch (state){
//				case 1:
//					transferForward=randN<0.9f;//90% forward/10% inv
//					break;
//				case 2:
//					transferForward=randN<0.5f;//50% forward/50% inv
//					break;
//				case 3:
//					transferForward=randN<0.1f;//10% forward/90% inv
//					break;
//				}
//			}
//			if(transferForward==true/*&&candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[e_twi.first].numberOfLanes>0*/){//note twi>0
//
//				candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[e_dir.first].numberOfLanes++;//note ++
//				candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[e_dir.first].cuda_design_modified=1;
//				if(e_twi.second==true&&candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[e_twi.first].numberOfLanes>0){
//					candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[e_twi.first].numberOfLanes--;
//					candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[e_twi.first].cuda_design_modified=1;
//				}
//				return true;
//			}
//			if(transferForward==false&&candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[e_dir.first].numberOfLanes>0){//note dir>0
//				candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[e_dir.first].numberOfLanes--;//note --
//				candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[e_dir.first].cuda_design_modified=1;
//				if(e_twi.second==true){
//					candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[e_twi.first].numberOfLanes++;
//					candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[e_twi.first].cuda_design_modified=1;
//				}
//				return true;
//			}
//		//}
//		return false;
//	}//
//
//	void CUDATrafficMCMC::generateCandidateState(){
//		// copy current state
//		if(MCMC_GENERATE)printf("copy current state currentSimState %d\n",currentSimState);
//		candidateTrafficSimulator=new CUDATrafficSimulator(*currentSimState);
//		candidateTrafficSimulator->simRoadGraph=new RoadGraph(*currentSimState->simRoadGraph);
//		//modifiedPlaceTypes=false;
//		if(MCMC_GENERATE)printf("createLaneMap\n");
//		candidateTrafficSimulator->createLaneMap();// to set the new edge number
//		calculateEdgePerGroupExpanded();
//
//		if(MCMC_GENERATE)printf("MCMC G candidateState %d\n",boost::num_edges(candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI));
//		///////////////////////////////////////////////////
//		// 1. DEFINE THE TRANFORMATION IS GOING TO TAKE PLACR
//		bool laneChangeB=false;//designOptions.laneDirection;
//		bool jobPeopleB=true;//designOptions.jobsDist|designOptions.peoplDist;
//		bool changePlacetypes=false;
//
//		if(laneChangeB==true&&jobPeopleB==true){
//			float ranN=((float)qrand())/RAND_MAX;
//			if(ranN<0.1){
//				laneChangeB=false;
//				jobPeopleB=true;
//			}else{
//				laneChangeB=true;
//				jobPeopleB=false;
//			}
//
//		}
//
//		bool design=true;
//
//		///////////////////////////////////////////////////
//		// 2. LANE DIR CHANGE
//		if(laneChangeB==true){
//			printf("**Lane Change\n");
//			//
//			std::random_device rd;
//			std::normal_distribution<float> dist(0,(maxDepth+1)/2.0f);
//			std::mt19937 rand_engine(rd());
//
//			for(int state=1;state<edgesPerGroupExpanded.size();state++){
//				for(int lT=0;lT<edgesPerGroupExpanded[state].size();lT++){
//					// Per each group
//					//int numEdgesToChange=5+qrand()%10;
//					int numEdgesToChange=10+qrand()%15;
//					for(int nEC=0;nEC<numEdgesToChange;nEC++){//50 attempts
//						int depth=abs(dist(rand_engine))+1.0f;//distrib 1->maxDpeth+1
//						if(MCMC_GENERATE)printf("depth %d states %d\n",depth,edgesPerGroupExpanded[state][lT].size());
//						if(depth>=edgesPerGroupExpanded[state][lT].size())//overflow goes to zero
//							depth=0;//edgesPerGroupExpanded[state][lT][0].size()-1;
//						if(edgesPerGroupExpanded[state][lT][depth].size()<=0){//not posible to select if there is none
//							continue;
//						}
//						int numElChange=qrand()%edgesPerGroupExpanded[state][lT][depth].size();
//						std::set<LC::RoadGraph::roadGraphEdgeDesc_BI>::iterator it = edgesPerGroupExpanded[state][lT][depth].begin();
//						std::advance(it,numElChange);
//						if(transferLaneInEdge(*it,state)){
//
//							//nEC++;
//						}
//					}
//				}
//			}
//		}
//		///////////////////////////////////////////////////
//		// 3. JOBS PEOPLE
//		if(jobPeopleB==true){
//			printf("**Jobs People\n");
//			//3. Create per segment car utilization
//			if(MCMC_GENERATE)printf("3. Create per segment car utilization: People vec %d\n",candidateTrafficSimulator->trafficPersonVec.size());
//			std::vector<QSet<int>> edgeToPeople;
//			edgeToPeople.resize(candidateTrafficSimulator->trafficLights.size());
//			for(int p=0;p<candidateTrafficSimulator->trafficPersonVec.size();p++){
//				int rN=0;
//				while(true){
//					ushort edge=candidateTrafficSimulator->trafficPersonVec[p].personPath[rN];
//					if(edge==0xFFFF){
//						break;
//					}
//					edgeToPeople[edge].insert(p);
//					rN++;
//					//printf("rN %d\n",rN);
//				}
//			}
//			if(MCMC_GENERATE)printf("edgeToPeople Size %d %d edgesPerGroupExpanded %d\n",edgeToPeople.size(),candidateTrafficSimulator->trafficLights.size(),edgesPerGroupExpanded.size());
//			////////////////
//			// 3. JOBS PEOPLE DESIGN
//			{
//				// 3.1 Find the pairs of distributions that are more likely to use those roads
//				if(MCMC_GENERATE)printf("3.1 Find the pairs of distributions that are more likely to use those roads\n");
//				std::vector<std::vector<QMap<int,std::pair<int,int>> >> topPairsPerGroup;
//				topPairsPerGroup.resize(4);
//				for(int state=1;state<edgesPerGroupExpanded.size();state++){
//					topPairsPerGroup[state].resize(edgesPerGroupExpanded[state].size());
//					if(MCMC_GENERATE)printf("  state %d NumGr %d\n",state,edgesPerGroupExpanded[state].size());
//					for(int lT=0;lT<edgesPerGroupExpanded[state].size();lT++){
//
//						std::map<std::pair<int,int>,int> pairStartEndToNumber;//kind of histogram
//						//for each edge in the interior ([0])
//						if(MCMC_GENERATE)printf("for each edge in the interior state %d group %d\n",state,lT);
//						for (std::set<LC::RoadGraph::roadGraphEdgeDesc_BI>::iterator it=edgesPerGroupExpanded[state][lT][0].begin();
//							it!=edgesPerGroupExpanded[state][lT][0].end(); ++it){
//								// for each person in that edge
//								std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e_dir=boost::edge(boost::source(*it, candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI),boost::target(*it, candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI),candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI);
//								if(e_dir.second==false)
//									printf("NO exists\n");
//								if(candidateTrafficSimulator->edgeDescToLaneMapNum.find(e_dir.first)==candidateTrafficSimulator->edgeDescToLaneMapNum.end()){
//									printf("NO in map\n");
//								}
//								uint laneNumber=candidateTrafficSimulator->edgeDescToLaneMapNum[e_dir.first];
//								//printf("laneN %u chainTrafficSimulator->edgeDescToLaneMapNum %d\n",laneNumber,chainTrafficSimulator->edgeDescToLaneMapNum.size());
//								QSet<int>::const_iterator iPeople = edgeToPeople[laneNumber].constBegin();
//								while (iPeople != edgeToPeople[laneNumber].constEnd()) {
//									int cPerson=*iPeople;
//									ushort initInter=candidateTrafficSimulator->trafficPersonVec[cPerson].init_intersection;
//									ushort endInter=candidateTrafficSimulator->trafficPersonVec[cPerson].end_intersection;
//									QVector3D initInterPt=candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[initInter].pt;
//									QVector3D endInterPt=candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[endInter].pt;
//									uint initInterIndex=candidateTrafficSimulator->simPeopleJobInfoLayers.getIndexClosestToPoint(initInterPt);
//									uint endInterIndex=candidateTrafficSimulator->simPeopleJobInfoLayers.getIndexClosestToPoint(endInterPt);
//									std::pair<int,int> pair=std::make_pair(initInterIndex,endInterIndex);
//									if(pairStartEndToNumber.find(pair)!=pairStartEndToNumber.end()){//exists key
//										pairStartEndToNumber[pair]++;
//									}else{
//										pairStartEndToNumber[pair]=1;
//									}
//									++iPeople;
//								}
//						}
//
//
//						//QMap<int,std::pair<int,int>> numberPairStartEnd;
//						for (std::map<std::pair<int,int>,int>::iterator riI=pairStartEndToNumber.begin(); riI!=pairStartEndToNumber.end(); ++riI){
//							topPairsPerGroup[state][lT].insert(riI->second,riI->first);
//							//if(numD<5)
//							//printf("Acc %d Pair %d %d\n",riI->second,riI->first.first,riI->first.second);
//
//						}
//					}
//				}
//				// 3.2 print top 5
//				if(MCMC_GENERATE)printf("3.2 Print TOP\n");
//				for(int state=1;state<edgesPerGroupExpanded.size();state++){
//					for(int lT=0;lT<edgesPerGroupExpanded[state].size();lT++){
//						if(MCMC_GENERATE)printf("Print TOP state %d group %d\n",state,lT);
//						QMap<int,std::pair<int,int>>::const_iterator iG=topPairsPerGroup[state][lT].constEnd();
//						int numD=0;
//						while(iG!=topPairsPerGroup[state][lT].constBegin()){
//							--iG;
//							if(MCMC_GENERATE)printf("Acc %d Pair %d %d\n",iG.key(),iG.value().first,iG.value().second);
//							numD++;
//							if(numD>5)
//								break;
//						}
//					}
//				}
//
//				// 3.3 with all the groups try to interexchange the densities
//				if(MCMC_GENERATE)printf("3.3 with all the groups try to interexchange the densities\n");
//				int state1GN=0;
//				int state3GN=0;
//				while(true){
//					if(state1GN>=topPairsPerGroup[1].size()&&state3GN>=topPairsPerGroup[3].size()){//check if all processed
//						break;
//					}
//					// 3.2.1 exchange 1-3
//					//int numTopToExchange=1+qrand()%1;
//					int numTopToExchange=3+qrand()%9;
//					QMap<int,std::pair<int,int>>::const_iterator iG1,iG3;
//					//QMapIterator<int,std::pair<int,int>> iG1(topPairsPerGroup[1][state1GN]);
//					//QMapIterator<int,std::pair<int,int>> iG3(topPairsPerGroup[3][state3GN]);
//					//QMap<QString, int>::const_rever i = topPairsPerGroup[1][state1GN].constBegin();
//					if(state1GN<topPairsPerGroup[1].size()){
//						if(MCMC_GENERATE)printf("__S group 1\n");
//						numTopToExchange=std::min<int>(numTopToExchange,topPairsPerGroup[1][state1GN].size());
//						iG1 = topPairsPerGroup[1][state1GN].constEnd();
//						//iG1.toBack();
//						//iG1.previous();
//					}
//					if(state1GN<topPairsPerGroup[3].size()){
//						if(MCMC_GENERATE)printf("__D group 3\n");
//						numTopToExchange=std::min<int>(numTopToExchange,topPairsPerGroup[3][state3GN].size());
//						iG3 = topPairsPerGroup[3][state3GN].constEnd();
//						//iG3.toBack();
//						//iG3.previous();
//					}
//					printf("__Number to Exchange--> %d Size Group %d %d CountGroup %d %d\n",numTopToExchange,
//						topPairsPerGroup[1].size(),topPairsPerGroup[3].size(),state1GN,state3GN);
//					for(int iN=0;iN<numTopToExchange;iN++){
//						if(state1GN<topPairsPerGroup[1].size()){
//							--iG1;
//							//if(qrand()<(RAND_MAX/2.0f))
//							//	--iG1;
//						}
//						if(state1GN<topPairsPerGroup[3].size()){
//							--iG3;
//							//if(qrand()<(RAND_MAX/2.0f))
//							//	--iG3;
//						}
//						// amplitured
//						int indexJobSour,indexPeopleSour,indexJobDest,indexPeopleDest;
//						// SOURCE
//						if(state1GN<topPairsPerGroup[1].size()){
//							if(qrand()<(0.75f*RAND_MAX)){// 75% most frequent
//								indexJobSour=iG1.value().second;
//								indexPeopleSour=iG1.value().first;
//							}else{// 25% random
//								indexJobSour=qrand()%candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].amplitudes.size();
//								indexPeopleSour=qrand()%candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].amplitudes.size();
//							}
//						}else{
//							//random
//							if(qrand()<(RAND_MAX/2.0f)){ // 50% from other index with amplitued
//								indexJobSour=candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].getRandomIndexWithAmpl();
//								indexPeopleSour=candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].getRandomIndexWithAmpl();
//							}else{ // 50% random
//								indexJobSour=candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].getRandomIndexWithin();
//								indexPeopleSour=candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].getRandomIndexWithin();
//							}
//						}
//						// DESTINATION
//						if(state1GN<topPairsPerGroup[3].size()){
//							indexJobDest=iG3.value().second;
//							indexPeopleDest=iG3.value().first;
//						}else{
//							//random
//							if(qrand()<(RAND_MAX/2.0f)){
//								indexJobDest=candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].getRandomIndexWithAmpl();
//								indexPeopleDest=candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].getRandomIndexWithAmpl();
//							}else{
//								indexJobDest=candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].getRandomIndexWithin();
//								indexPeopleDest=candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].getRandomIndexWithin();
//							}
//						}
//
//						float amJob1=candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].amplitudes[indexJobSour];
//						float amPeopl1=candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].amplitudes[indexPeopleSour];
//
//						float amJob3=candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].amplitudes[indexJobDest];
//						float amPeopl3=candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].amplitudes[indexPeopleDest];
//						// exchange
//						//float exchange=10+60.0f*((float)qrand())/RAND_MAX;//100.0f
//						float exchange=100.0f;
//						exchange=std::min<float>(exchange,amPeopl1);
//						exchange=std::min<float>(exchange,amJob1);
//						exchange=std::min<float>(exchange,amPeopl3);
//						exchange=std::min<float>(exchange,amJob3);
//						if(MCMC_GENERATE)printf("Old distribution SOURCE [%d] %f [%d] %f DEST [%d] %f [%d] %f\n",
//							indexJobSour,candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].amplitudes[indexJobSour],
//							indexPeopleSour,candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].amplitudes[indexPeopleSour],
//							indexJobDest,candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].amplitudes[indexJobDest],
//							indexPeopleDest,candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].amplitudes[indexPeopleDest]);
//						candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].amplitudes[indexJobSour]-=exchange;
//						candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].amplitudes[indexPeopleSour]-=exchange;
//
//						candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].amplitudes[indexJobDest]+=exchange;
//						candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].amplitudes[indexPeopleDest]+=exchange;
//						if(MCMC_GENERATE)printf("New distribution SOURCE [%d] %f [%d] %f DEST [%d] %f [%d] %f\n",
//							indexJobSour,candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].amplitudes[indexJobSour],
//							indexPeopleSour,candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].amplitudes[indexPeopleSour],
//							indexJobDest,candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].amplitudes[indexJobDest],
//							indexPeopleDest,candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].amplitudes[indexPeopleDest]);
//						/*if(state1GN<topPairsPerGroup[1].size())
//							//iG1.previous();
//						if(state1GN<topPairsPerGroup[3].size()){
//							//iG3.previous();
//						}*/
//						state1GN++;
//						state3GN++;
//					}
//				}
//			}
//		}
//		///////////////////////////////////////////////////
//		// 4. Change placetype
//		if(changePlacetypes==true){
//
//			// copy current placetype
//			candidatePlaceTypes=new PlaceTypesMainClass(*currentPlaceTypes);//copy
//			// create new parameters
//			//!!!
//			printf("ptNum %d\n",candidatePlaceTypes->myPlaceTypes.size());
//			//candidatePlaceTypes->myPlaceTypes[0].myAttributes.pt_num_departing=3;
//			//candidatePlaceTypes->myPlaceTypes[0].myAttributes.pt_edges_curvature=candidatePlaceTypes->myPlaceTypes[0].myAttributes.pt_edges_curvature*1.1f;
//			/*printf("pt_edges_curvature %f\n",candidatePlaceTypes->myPlaceTypes[0].myAttributes.pt_edges_curvature);
//			printf("pt_loc_edges_irregularity %f\n",candidatePlaceTypes->myPlaceTypes[0].myAttributes.pt_loc_edges_irregularity);
//			printf("pt_edges_lengthU %f\n",candidatePlaceTypes->myPlaceTypes[0].myAttributes.pt_edges_lengthU);
//			printf("pt_edges_lengthU %f\n",candidatePlaceTypes->myPlaceTypes[0].myAttributes.pt_edges_lengthV);*/
//			for(int i=0;i<10;i++){
//				//candidatePlaceTypes->myPlaceTypes[0].myAttributes.pt_edges_curvature=(qrand()%5)/100.0f;
//				//candidatePlaceTypes->myPlaceTypes[0].myAttributes.pt_loc_edges_irregularity=(qrand()%5)/100.0f;
//
//				//candidatePlaceTypes->myPlaceTypes[0].myAttributes.pt_loc_edges_lengthV=(1+qrand()%99)/100.0f;
//				//candidatePlaceTypes->myPlaceTypes[0].myAttributes.pt_loc_edges_lengthU=(1+qrand()%99)/100.0f;
//				candidatePlaceTypes->myPlaceTypes[0].myAttributes.pt_edges_lengthU=(1000+qrand()%10000)/100.0f;
//				candidatePlaceTypes->myPlaceTypes[0].myAttributes.pt_edges_lengthV=(1000+qrand()%10000)/100.0f;
//
//				/*int parSpaceSz = candidatePlaceTypes->ptInParamSpace.size();
//				if( parSpaceSz <= 0 ){
//				//printf("a1.\n");
//				return 1;
//				}*/
//
//				if(ProceduralRoadsMachine::generateRoadNetwork(*candidatePlaceTypes, *candidateTrafficSimulator->simRoadGraph)==true){
//					break;
//				}
//			}
//
//
//			/*return;
//			// remove selected placetype myRoadGraph AND myRoadGraph_BI
//			RoadGraph::roadGraphVertexIter vi, viEnd;
//			int placeTypeToRemove=0;
//			for(boost::tie(vi, viEnd) = boost::vertices(candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI);
//				vi != viEnd; ++vi)
//			{
//				if(candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*vi].myPlaceTypeIdx==placeTypeToRemove){
//					RoadGraph::out_roadGraphEdgeIter_BI Oei, Oei_end;
//					for(boost::tie(Oei, Oei_end) = boost::out_edges(*vi,candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI); Oei != Oei_end; ++Oei){
//
//					}
//
//				}
//			}*/
//			// generate new roadgraph
//			//ProceduralRoadsMachine::generateRoadNetwork(*candidatePlaceTypes, *chainTrafficSimulator->simRoadGraph);
//
//
//		}//place types
//
//	}//
//
//	void CUDATrafficMCMC::simulateCandidateState(){
//		//printf("MCMC S Copy to temp\n");
//		//RoadGraph* temp=chainTrafficSimulator->originalRoadGraph;
//		//printf("MCMC S Simulate\n");
//		//chainTrafficSimulator->simRoadGraph=candidateState;
//		//printf("MCMC S Cand %d\n",boost::num_edges(chainTrafficSimulator->simRoadGraph->myRoadGraph_BI));
//		//printf("MCMC S tempSimulatorRoad %d\n",boost::num_edges(chainTrafficSimulator->simRoadGraph->myRoadGraph_BI));
//		candidateTrafficSimulator->simulateInCPU_MultiPass(numPasses);
//		//printf("MCMC S Restore temp\n");
//		//chainTrafficSimulator->originalRoadGraph=temp;
//	}//
//
//	void CUDATrafficMCMC::evaluateNetwork(std::vector<float>& results,bool diffAmpl,bool diffLanes){
//
//		calculateEdgePerGroupExpanded();//!!
//		if(MCMC_DEBUG)printf("MCMC evaluateNetwork CState %d edgesPerGroupExpanded %d\n",boost::num_edges(candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI),edgesPerGroupExpanded.size());
//		float errorToZero=0;int numToZero=0;
//		float errorToMid=0;int numToMid=0;
//		float errorToOne=0;int numToOne=0;
//		bool temporalCalculation=false;
//		if(temporalCalculation==false){
//			for(int state=1;state<edgesPerGroupExpanded.size();state++){
//				for(int lT=0;lT<edgesPerGroupExpanded[state].size();lT++){
//					//edgesPerGroupExpanded[state][lT][0]
//					if(MCMC_DEBUG)printf("state %d group %d group0 size %d\n",state,lT,edgesPerGroupExpanded[state][lT][0].size());
//					for(std::set<LC::RoadGraph::roadGraphEdgeDesc_BI>::iterator it = edgesPerGroupExpanded[state][lT][0].begin();
//						it != edgesPerGroupExpanded[state][lT][0].end();) { // user define by the user
//							//LC::RoadGraph::roadGraphEdgeDesc_BI currEdge=*it;//edge to evaluate
//							//std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e_dir=boost::edge(boost::source(currEdge, chainTrafficSimulator->simRoadGraph->myRoadGraph_BI),boost::target(currEdge, chainTrafficSimulator->simRoadGraph->myRoadGraph_BI),chainTrafficSimulator->simRoadGraph->myRoadGraph_BI);
//
//							if(candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*it/*e_dir.first*/].numberOfLanes>0){
//								if(state==1){//distance to 0
//									errorToZero=errorToZero+abs(candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*it/*e_dir.first*/].averageUtilization[indexToEvaluate]);
//									numToZero++;
//								}
//								if(state==2){//distance to 0.5
//									errorToMid=errorToMid+abs(0.5f-candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*it/*e_dir.first*/].averageUtilization[indexToEvaluate]);
//									numToMid++;
//								}
//								if(state==3){//distance to 1
//									errorToOne=errorToOne+abs(1.0f-candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*it/*e_dir.first*/].averageUtilization[indexToEvaluate]);
//									numToOne++;
//								}
//							}
//							++it;
//							//printf("error %f size %d\n",error,chainTrafficSimulator->simRoadGraph->myRoadGraph_BI[e_dir.first].averageUtilization.size());
//					}
//				}
//			}
//		}else{
//			int state=1;
//			int indexToEv[]={6,16,26};
//			for(int inEv=0;inEv<3;inEv++){
//				indexToEvaluate=indexToEv[inEv];
//				for(int lT=0;lT<edgesPerGroupExpanded[state].size();lT++){
//					//edgesPerGroupExpanded[state][lT][0]
//					if(MCMC_DEBUG)printf("state %d group %d group0 size %d\n",state,lT,edgesPerGroupExpanded[state][lT][0].size());
//					for(std::set<LC::RoadGraph::roadGraphEdgeDesc_BI>::iterator it = edgesPerGroupExpanded[state][lT][0].begin();
//						it != edgesPerGroupExpanded[state][lT][0].end();) { // user define by the user
//							//LC::RoadGraph::roadGraphEdgeDesc_BI currEdge=*it;//edge to evaluate
//							//std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e_dir=boost::edge(boost::source(currEdge, chainTrafficSimulator->simRoadGraph->myRoadGraph_BI),boost::target(currEdge, chainTrafficSimulator->simRoadGraph->myRoadGraph_BI),chainTrafficSimulator->simRoadGraph->myRoadGraph_BI);
//
//							if(candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*it/*e_dir.first*/].numberOfLanes>0){
//								if(inEv==1){//distance to 0
//									errorToZero=errorToZero+abs(candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*it/*e_dir.first*/].averageUtilization[indexToEvaluate]);
//									numToZero++;
//								}
//
//								if(inEv==0||inEv==2){//distance to 1
//									errorToOne=errorToOne+abs(1.0f-candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*it/*e_dir.first*/].averageUtilization[indexToEvaluate]);
//									numToOne++;
//								}
//							}
//							++it;
//							//printf("error %f size %d\n",error,chainTrafficSimulator->simRoadGraph->myRoadGraph_BI[e_dir.first].averageUtilization.size());
//					}
//				}
//			}
//		}
//		results.resize(9);
//		results[0]=errorToZero+errorToMid+errorToOne;
//		results[1]=errorToZero;
//		results[2]=errorToMid;
//		results[3]=errorToOne;
//		results[4]=numToZero;
//		results[5]=numToMid;
//		results[6]=numToOne;
//		if(diffAmpl==true){
//			results[7]=0;
//			float accDiffAmplitudes=0;
//			float accAmplitudes=0;
//			for(int nSample=0;nSample<candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].amplitudes.size();nSample++){
//				accDiffAmplitudes+=abs(candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].amplitudes[nSample]-originalSimulationState->simPeopleJobInfoLayers.layers[0].amplitudes[nSample]);
//				accDiffAmplitudes+=abs(candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].amplitudes[nSample]-originalSimulationState->simPeopleJobInfoLayers.layers[1].amplitudes[nSample]);
//				accAmplitudes+=candidateTrafficSimulator->simPeopleJobInfoLayers.layers[0].amplitudes[nSample];
//				accAmplitudes+=candidateTrafficSimulator->simPeopleJobInfoLayers.layers[1].amplitudes[nSample];
//			}
//			results[7]=accDiffAmplitudes/accAmplitudes;//normalized to the total amplitudes
//			printf("------ diffAmpl %f\n",results[7]);
//		}
//		if(diffLanes==true){
//			//printf("------ diffLanes\n");
//			results[8]=0;
//			RoadGraph::roadGraphEdgeIter_BI ei, eiEnd;
//			for(boost::tie(ei, eiEnd) = boost::edges(candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI);
//				ei != eiEnd; ++ei)
//			{
//				std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e0_pair=boost::edge(
//					boost::source(*ei, originalSimulationState->simRoadGraph->myRoadGraph_BI),
//					boost::target(*ei, originalSimulationState->simRoadGraph->myRoadGraph_BI),
//					originalSimulationState->simRoadGraph->myRoadGraph_BI);
//				if(e0_pair.second==false){
//					results[8]+=candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*ei].numberOfLanes;//not even exist
//					//printf("------ No pair %d\n",candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*ei].numberOfLanes);
//				}else{
//					int numCurrLan=candidateTrafficSimulator->simRoadGraph->myRoadGraph_BI[*ei].numberOfLanes;
//					int numorigLan=originalSimulationState->simRoadGraph->myRoadGraph_BI[e0_pair.first].numberOfLanes;
//					results[8]+=abs((float)(numCurrLan-numorigLan));//different number of lanes
//					//printf("------ Dif %f\n",abs((float)(numCurrLan-numorigLan)));
//				}
//
//			}
//			if(MCMC_DEBUG)printf("------ diffLanes %f\n",results[8]);
//		}
//
//	}//
//
//	void CUDATrafficMCMC::transformDistribution(){
//		//1. reset amplitudes
//		for(int aN=0;aN<currentSimState->simPeopleJobInfoLayers.layers[0].amplitudes.size();aN++){
//			currentSimState->simPeopleJobInfoLayers.layers[0].amplitudes[aN]=0;
//			currentSimState->simPeopleJobInfoLayers.layers[1].amplitudes[aN]=0;
//		}
//		//2. put distribution surrounding red areas
//		//std::vector<std::vector<polygon>*> allVec;
//		QSet<int> closestVertexSet;
//		for(int gN=0;gN<allVec[3]->size();gN++){
//			printf("+++++++++ grp3 %d\n",allVec[3]->at(gN).outer().size());
//			for(size_t i=0; i<(allVec[3]->at(gN).outer()).size(); i++){
//
//				QVector3D cont((allVec[3]->at(gN).outer())[i].x(),(allVec[3]->at(gN).outer())[i].y(),0);
//				int closestIndex=currentSimState->simPeopleJobInfoLayers.getIndexClosestToPoint(cont);
//				closestVertexSet.insert(closestIndex);
//			}
//		}
//		printf("+++++++++ closestVertexSet %d\n",closestVertexSet.size());
//		//3. set jobs and works around that
//		QSet<int>::const_iterator i = closestVertexSet.constBegin();
//		while (i != closestVertexSet.constEnd()) {
//			//if(qrand()<(RAND_MAX/2.0f)){
//				currentSimState->simPeopleJobInfoLayers.layers[0].amplitudes[*i]=100.0f;
//			//}else{
//				currentSimState->simPeopleJobInfoLayers.layers[1].amplitudes[*i]=100.0f;
//			//}
//			//qDebug() << *i;
//			++i;
//		}
//
//
//
//	}//
//
//
//}


