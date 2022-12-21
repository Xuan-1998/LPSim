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
*		Procedural Roads Machine
*
*
*		@desc Main
*		@author cvanegas
*
************************************************************************************************/
#ifndef Q_MOC_RUN
#include <boost/graph/planar_face_traversal.hpp>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#endif
#include "pmRoads.h"
#include "../global.h"

//include "client_global_variables.h"

namespace LC {


	//Function declarations
	void attemptExpansion(RoadGraph::roadGraphVertexDesc &srcVertexDesc, RoadGraph &inRoadGraph,
		std::list<RoadGraph::roadGraphVertexDesc> &newSeeds, float inRefAngle);

	void removeDeadEnds(RoadGraph &inRoadGraph);

	//generate initial edges
	//    this is mainly used to add edges in the bounding polygon
	void generateInitialEdges(std::vector<QVector3D> &polygon, RoadGraph &inRoadGraph)
	{
		RoadGraph::roadGraphVertexDesc vDescFirst;
		RoadGraph::roadGraphVertexDesc vDesc0;
		RoadGraph::roadGraphVertexDesc vDesc1;

		int iNext;
		int pgonSz = polygon.size();

		if(pgonSz < 3) return;

		vDesc0 = boost::add_vertex(inRoadGraph.myRoadGraph);
		inRoadGraph.myRoadGraph[vDesc0].pt = polygon[0];
		inRoadGraph.myRoadGraph[vDesc0].isSeed = false;
		inRoadGraph.myRoadGraph[vDesc0].isBoundingPgonVertex = true;	
		inRoadGraph.myRoadGraph[vDesc0].myPlaceTypeIdx = -1;
		inRoadGraph.myRoadGraph[vDesc0].width = G::global().getFloat("arterial_edges_width");

		vDescFirst = vDesc0;

		for(int i=1; i<pgonSz; ++i){

			vDesc1 = boost::add_vertex(inRoadGraph.myRoadGraph);
			inRoadGraph.myRoadGraph[vDesc1].pt = polygon[i];
			inRoadGraph.myRoadGraph[vDesc1].isSeed = false;
			inRoadGraph.myRoadGraph[vDesc1].isBoundingPgonVertex = true;
			inRoadGraph.myRoadGraph[vDesc1].width = G::global().getFloat("arterial_edges_width");


			//add new edge
			std::pair<RoadGraph::roadGraphEdgeDesc, bool> edge_pair = 
				boost::add_edge( vDesc0, vDesc1, inRoadGraph.myRoadGraph );		
			inRoadGraph.myRoadGraph[edge_pair.first].roadSegmentWidth = inRoadGraph.myRoadGraph[vDesc0].width;
			//inRoadGraph.myRoadGraph[edge_pair.first].maxSpeed = LC::misctools::Global::global()->arterial_edges_speed;

			vDesc0 = vDesc1;
		}
		std::pair<RoadGraph::roadGraphEdgeDesc, bool> edge_pair = 
			boost::add_edge( vDesc0, vDescFirst, inRoadGraph.myRoadGraph );
		inRoadGraph.myRoadGraph[edge_pair.first].roadSegmentWidth = inRoadGraph.myRoadGraph[vDesc0].width;
		//inRoadGraph.myRoadGraph[edge_pair.first].maxSpeed = LC::misctools::Global::global()->arterial_edges_speed;
	}

	//Create initial seeds
	//   generate one seed per place type
	void generateInitialSeeds(PlaceTypesMainClass &placeTypesIn,
		std::list<RoadGraph::roadGraphVertexDesc> &seeds,
		RoadGraph &inRoadGraph)
	{
		seeds.clear();

		unsigned long ranSeed = 1;
		srand(ranSeed);

		std::vector<QVector3D> seedsPositions;
		for(int i=0; i<G::global().getInt("num_place_types")/*placeTypesIn.myPlaceTypes.size()*/; ++i){
			seedsPositions.push_back(placeTypesIn.myPlaceTypes.at(i).getQVector3D("pt_pt"));
		}
		
		//for(int i=0; i<seedsPositions.size(); ++i){
		for(int i=0; i<G::global().getInt("num_place_types"); ++i){

			//======== populate initial seeds
			RoadGraph::roadGraphVertexDesc tmpSeedDesc = boost::add_vertex(inRoadGraph.myRoadGraph);

			//unsigned long ranSeed = LC::misctools::mix(clock(), time(NULL), _getpid());
			int newRandSeed = (rand() + 188179)%714025;

			float deltaTheta = 0.0f;

			float compensatedCurvature = placeTypesIn.myPlaceTypes[i].getFloat("pt_edges_curvature");

			RoadGraphVertex tmpSeed(seedsPositions[i],
				placeTypesIn.myPlaceTypes[i].getFloat("pt_edges_lengthU"),
				placeTypesIn.myPlaceTypes[i].getFloat("pt_edges_lengthV"),
				placeTypesIn.myPlaceTypes[i].getFloat("pt_orientation"),
				placeTypesIn.myPlaceTypes[i].getInt("pt_num_departing"),
				newRandSeed,
				deltaTheta,
				true,
				false,
				placeTypesIn.myPlaceTypes[i].getFloat("pt_edges_irregularity"),
				compensatedCurvature,
				G::global().getFloat("arterial_edges_width"),
				i,
				G::global().getFloat("arterial_edges_speed"));
			{
				inRoadGraph.myRoadGraph[tmpSeedDesc].pt = tmpSeed.pt;
				inRoadGraph.myRoadGraph[tmpSeedDesc].departingDirections = tmpSeed.departingDirections;
				inRoadGraph.myRoadGraph[tmpSeedDesc].distU = tmpSeed.distU;
				inRoadGraph.myRoadGraph[tmpSeedDesc].distV = tmpSeed.distV;
				inRoadGraph.myRoadGraph[tmpSeedDesc].randSeed = tmpSeed.randSeed;
				inRoadGraph.myRoadGraph[tmpSeedDesc].deltaTheta = tmpSeed.deltaTheta;
				inRoadGraph.myRoadGraph[tmpSeedDesc].isSeed = tmpSeed.isSeed;
				inRoadGraph.myRoadGraph[tmpSeedDesc].isBoundingPgonVertex = tmpSeed.isBoundingPgonVertex;		
				inRoadGraph.myRoadGraph[tmpSeedDesc].irregularity = tmpSeed.irregularity;
				inRoadGraph.myRoadGraph[tmpSeedDesc].curvature = tmpSeed.curvature;
				inRoadGraph.myRoadGraph[tmpSeedDesc].width = tmpSeed.width;
				inRoadGraph.myRoadGraph[tmpSeedDesc].myPlaceTypeIdx = i;
			}

			printf("%d %f %f\n",placeTypesIn.myPlaceTypes[i].getFloat("pt_num_departing"),placeTypesIn.myPlaceTypes[i].getFloat("pt_edges_lengthV"),placeTypesIn.myPlaceTypes[i].getFloat("pt_orientation"));
			seeds.push_back(tmpSeedDesc);
		}
	}

	//===================================================
	// These functions are for the routine that extracts loops from the planar graph

	struct RoadGraphLoop
	{
		LC::misctools::Loop3D myLoop3D;
		int myRandSeed;
		int myPlaceTypeIdx;
	};

	RoadGraphLoop graphLoopGlobal;

	bool roads_isFirstVertexVisited;
	std::map<int, int> roads_placeTypeFrequencyMap;
	std::vector<RoadGraphLoop> *loopsPtr;
	RoadGraph *roads_roadGraphPtr;
	int roads_curRandSeed;
	int roads_curPlaceTypeIdx;
	int roads_maxPlaceTypeIdx;

	struct road_output_visitor : public boost::planar_face_traversal_visitor
	{
		void begin_face()
		{	
			graphLoopGlobal.myLoop3D.clear();
			graphLoopGlobal.myRandSeed = 0;
			graphLoopGlobal.myPlaceTypeIdx = -1;
			roads_isFirstVertexVisited = true;		
			roads_placeTypeFrequencyMap.clear();
		}

		void end_face()
		{
			if(graphLoopGlobal.myLoop3D.size() > 2){			

				graphLoopGlobal.myRandSeed = roads_curRandSeed;

				//assign place type to graph loop ------------			
				std::map<int,int>::iterator it;
				int tmpMaxKey = -1;
				int tmpMaxVal = 0;
				for ( it=roads_placeTypeFrequencyMap.begin() ; it != roads_placeTypeFrequencyMap.end(); it++ ){
					if ( (*it).second > tmpMaxVal ){
						tmpMaxKey = (*it).first;
						tmpMaxVal = (*it).second;					
					}
				}

				if(tmpMaxKey != -1){
					graphLoopGlobal.myPlaceTypeIdx = tmpMaxKey;
				} else {
					graphLoopGlobal.myPlaceTypeIdx = 0;
				}
				//assign place type to graph loop ------------			

				loopsPtr->push_back(graphLoopGlobal);			
			}			
		}
	};

	//Vertex visitor
	struct road_vertex_output_visitor : public road_output_visitor
	{
		template <typename Vertex> 
		void next_vertex(Vertex v) 
		{ 	
			if(  v >= 0 && v < boost::num_vertices(roads_roadGraphPtr->myRoadGraph) ){
				graphLoopGlobal.myLoop3D.push_back( (roads_roadGraphPtr->myRoadGraph)[v].pt );

				//initialize block random seed from first street node random seed
				if(roads_isFirstVertexVisited){
					roads_isFirstVertexVisited = false;				
					roads_curRandSeed = ( (roads_roadGraphPtr->myRoadGraph)[v].randSeed*4096 + 150889) % 714025;
				}

				roads_curPlaceTypeIdx = (roads_roadGraphPtr->myRoadGraph)[v].myPlaceTypeIdx;
				//std::cout << " "<< roads_curPlaceTypeIdx << " "; std::fflush(stdout);
				//if(roads_curPlaceTypeIdx >= 0 && roads_curPlaceTypeIdx < roads_maxPlaceTypeIdx)
				{
					//if curPlaceTypeIdx is not a key in the map, initialize it to zero
					if(roads_placeTypeFrequencyMap.find(roads_curPlaceTypeIdx) == roads_placeTypeFrequencyMap.end()){
						roads_placeTypeFrequencyMap[roads_curPlaceTypeIdx] = 0;
					} else { //else, increase by one
						(roads_placeTypeFrequencyMap[roads_curPlaceTypeIdx])++;
					}				
				}
				//std::cout << "done ";
			}
		}

		template <typename Edge> 
		void next_edge(Edge e) 
		{ 
			int sIdx, tIdx;
			sIdx = boost::source(e, roads_roadGraphPtr->myRoadGraph);
			tIdx = boost::target(e, roads_roadGraphPtr->myRoadGraph);

			if(  sIdx >= 0 && sIdx < boost::num_vertices(roads_roadGraphPtr->myRoadGraph) &&
				tIdx >= 0 && tIdx < boost::num_vertices(roads_roadGraphPtr->myRoadGraph) )
			{			
				//std::cout << (roadGraphPtr->myRoadGraph)[e].roadSegmentWidth << " ";			
				//blockContourWidths.push_back( (roadGraphPtr->myRoadGraph)[e].roadSegmentWidth);				
				//blockContourWidths.push_back(5.0f);

			}
		}
	};

	bool findLoopsInGraph( RoadGraph &inRoadGraph, std::vector<RoadGraphLoop> &outLoops )
	{	
		roads_roadGraphPtr = &inRoadGraph;
		loopsPtr = &outLoops;
		loopsPtr->clear();

		bool isPlanar = false;	

		//Make sure graph is planar
		typedef std::vector< RoadGraph::roadGraphEdgeDesc > tEdgeDescriptorVector;
		std::vector<tEdgeDescriptorVector> embedding(boost::num_vertices(inRoadGraph.myRoadGraph));

		// Test for planarity		
		if (boost::boyer_myrvold_planarity_test(boost::boyer_myrvold_params::graph = inRoadGraph.myRoadGraph,
			boost::boyer_myrvold_params::embedding = &embedding[0]) )
		{
			//std::cout << "Input graph is planar" << std::endl;
			isPlanar = true;
		} else {
			std::cout << "ERROR: Graph could not be planarized\n";
			return false;
		}

		//Create edge index property map?	
		typedef std::map<RoadGraph::roadGraphEdgeDesc, size_t> EdgeIndexMap;
		EdgeIndexMap mapEdgeIdx;
		boost::associative_property_map<EdgeIndexMap> pmEdgeIndex(mapEdgeIdx);		
		RoadGraph::roadGraphEdgeIter ei, ei_end;	
		int edge_count = 0;
		for(boost::tie(ei, ei_end) = boost::edges(inRoadGraph.myRoadGraph); ei != ei_end; ++ei){
			mapEdgeIdx.insert(std::make_pair(*ei, edge_count++));	
		}

		//Extract blocks from road graph using boost graph planar_face_traversal	
		road_vertex_output_visitor v_vis;
		boost::planar_face_traversal(inRoadGraph.myRoadGraph, &embedding[0], v_vis, pmEdgeIndex);

		//Clean loops, remove largest, reorient
		float curArea;
		float maxArea = -FLT_MAX;
		int maxAreaIdx = -1;
		for(int i=0; i<outLoops.size(); ++i){
			LC::misctools::Loop3D cleanOutLoop;
			LC::misctools::Polygon3D::reorientFace(outLoops.at(i).myLoop3D);
			LC::misctools::Polygon3D::cleanLoop(outLoops.at(i).myLoop3D, cleanOutLoop, 5.0f);		
			outLoops.at(i).myLoop3D = cleanOutLoop;

			curArea = LC::misctools::Polygon3D::computeLoopArea(outLoops.at(i).myLoop3D, true);
			if(curArea > maxArea){
				maxArea = curArea;
				maxAreaIdx = i;
			}
		}

		if(maxAreaIdx != -1){
			outLoops.erase(outLoops.begin() + maxAreaIdx);
		}

		return true;
		//std::cout << loopsPtr->size() << " road loops extracted\n";
	}

	//===================================================

	QVector3D getMeanLoopPoint(LC::misctools::Loop3D &loopIn)
	{
		int loopInSz = loopIn.size();
		QVector3D meanPt;
		for(int i=0; i<loopInSz; ++i){
			meanPt = meanPt + loopIn.at(i);
		}
		meanPt = meanPt / (float)loopInSz;
		return meanPt;
	}

	//Create initial street seeds
	bool generateInitialStreetSeeds(PlaceTypesMainClass &placeTypesIn,
		std::list<RoadGraph::roadGraphVertexDesc> &seeds,
		RoadGraph &inRoadGraph)
	{
		std::vector<RoadGraphLoop> outLoops;

		//extract loops from current graph
		if(!findLoopsInGraph(inRoadGraph, outLoops)){
			return false;
		}

		seeds.clear();

		unsigned long ranSeed = 1;
		srand(ranSeed);

		std::vector<QVector3D> seedsPositions;

		int placeTypeIdx;

		//======== populate initial seeds

		QVector2D newPtA;
		QVector2D newPtB;

		float locEdgesU, locEdgesV;
		float locEdgesIrregularity, locEdgesCurvature;
		float locEdgesWidth;

		for(int i=0; i<outLoops.size(); ++i){

			//get obb of loop
			QVector3D obbSize;
			QMatrix4x4 obbMat;
			LC::misctools::Polygon3D::getLoopOBB(outLoops.at(i).myLoop3D, obbSize, obbMat);

			QVector3D dirVectorInit, dirVector;
			QVector3D midPt(0.0f, 0.0f, 0.0f);
			QVector3D auxPt(1.0f, 0.0f, 0.0f);

			midPt = midPt*obbMat;

			dirVectorInit = (auxPt*obbMat - midPt);
			dirVectorInit.normalize();
			if(obbSize.y() > obbSize.x()){
				dirVector.setX( -dirVectorInit.y() );
				dirVector.setY(  dirVectorInit.x() );
			} else {
				dirVector = dirVectorInit;
			}

			//make position of seed be equal to center point of OBB of loop
			//	newPtA = QVector2D(getMeanLoopPoint(outLoops.at(i).myLoop3D));
			newPtA = QVector2D(midPt);

			//make orientation be equal to main direction of OBB of loop
			float tmpOrient;
			float tmpDeltaTheta = 0.0f;
			tmpOrient = atan2(dirVector.y(), dirVector.x());

			newPtA = newPtA + QVector2D(LC::misctools::genRand(-2.0f, 2.0f), LC::misctools::genRand(-2.0f, 2.0f));

			//use as seed only if within a place type instance
			bool isWithinPlaceType = false;
			float distToValidClosestPlaceType = FLT_MAX;
			int validClosestPlaceTypeIdx = -1;

			for(int j=0; j<G::global().getInt("num_place_types"); ++j){
				float distToPTCenter = (placeTypesIn.myPlaceTypes.at(j).getQVector3D("pt_pt") - newPtA).length();			
				if( distToPTCenter < placeTypesIn.myPlaceTypes.at(j).getFloat("pt_radius")){
					isWithinPlaceType = true;
					if( distToPTCenter < distToValidClosestPlaceType ){
						distToValidClosestPlaceType = distToPTCenter;
						validClosestPlaceTypeIdx = j;
					}
				}
			}
			//std::cout << "\n";

			if(!isWithinPlaceType){
				continue;
			}

			RoadGraph::roadGraphVertexDesc vDescA = boost::add_vertex(inRoadGraph.myRoadGraph);

			//srand(outLoops.at(i).myRandSeed);

			//placeTypeIdx = outLoops.at(i).myPlaceTypeIdx;
			placeTypeIdx = validClosestPlaceTypeIdx;
			if(placeTypeIdx < 0 || placeTypeIdx >= G::global().getInt("num_place_types")){
				placeTypeIdx = 0;
			}
			outLoops.at(i).myPlaceTypeIdx = placeTypeIdx;

			//std::cout << " " << i << " out of " << outLoops.size()<< "with pti: " << placeTypeIdx << "\n"; std::fflush(stdout);

			int newRandSeed = (rand() + 188179)%714025;


			//---------compute U and V
			locEdgesU = placeTypesIn.myPlaceTypes[placeTypeIdx].getFloat("pt_loc_edges_lengthU")*
				placeTypesIn.myPlaceTypes[placeTypeIdx].getFloat("pt_edges_lengthU");

			locEdgesV = placeTypesIn.myPlaceTypes[placeTypeIdx].getFloat("pt_loc_edges_lengthV")*
				placeTypesIn.myPlaceTypes[placeTypeIdx].getFloat("pt_edges_lengthV");

			float ang = tmpOrient;
			float inRef = 0.0f;
			bool isU;
			if( ( fabs(fabs(ang - inRef)            ) < 0.25f*M_PI ) ||
				( fabs(fabs(ang - inRef) -      M_PI) < 0.25f*M_PI ) ||
				( fabs(fabs(ang - inRef) - 2.0f*M_PI) < 0.25f*M_PI ) )			
			{
				isU = true;
			} else {
				isU = false;
			}		

			//if(obbSize.x() > obbSize.y()){
			if(isU){
				float cpy = locEdgesU;
				locEdgesU = locEdgesV;
				locEdgesV = cpy;
				//tmpOrient = tmpOrient+0.5*M_PI;
			}
			//----------------------------

			locEdgesIrregularity = placeTypesIn.myPlaceTypes[placeTypeIdx].getFloat("pt_loc_edges_irregularity");

			locEdgesCurvature = placeTypesIn.myPlaceTypes[placeTypeIdx].getFloat("pt_loc_edges_curvature");

			locEdgesWidth = 0.5f*G::global().getFloat("arterial_edges_width");

			RoadGraphVertex tmpSeed(newPtA,
				locEdgesU,
				locEdgesV,
				tmpOrient,
				4,//placeTypesIn.myPlaceTypes[placeTypeIdx].myAttributes.pt_num_departing,
				newRandSeed,
				tmpDeltaTheta,
				false,
				false,
				locEdgesIrregularity,
				locEdgesCurvature,
				locEdgesWidth,
				placeTypeIdx,
				G::global().getFloat("arterial_edges_speed"));
			{
				inRoadGraph.myRoadGraph[vDescA].pt = tmpSeed.pt;
				inRoadGraph.myRoadGraph[vDescA].departingDirections = tmpSeed.departingDirections;
				inRoadGraph.myRoadGraph[vDescA].distU = locEdgesU;
				inRoadGraph.myRoadGraph[vDescA].distV = locEdgesV;
				inRoadGraph.myRoadGraph[vDescA].randSeed = newRandSeed;
				inRoadGraph.myRoadGraph[vDescA].deltaTheta = tmpDeltaTheta;
				inRoadGraph.myRoadGraph[vDescA].isSeed = tmpSeed.isSeed;
				inRoadGraph.myRoadGraph[vDescA].isBoundingPgonVertex = tmpSeed.isBoundingPgonVertex;		
				inRoadGraph.myRoadGraph[vDescA].irregularity = locEdgesIrregularity;
				inRoadGraph.myRoadGraph[vDescA].curvature = locEdgesCurvature;
				inRoadGraph.myRoadGraph[vDescA].width = locEdgesWidth;
				inRoadGraph.myRoadGraph[vDescA].myPlaceTypeIdx = tmpSeed.myPlaceTypeIdx;
			}

			seeds.push_back(vDescA);
		}

		return true;
		//std::cout << seeds.size() << " street seeds created\n";
	}


	//Generate Roads
	bool ProceduralRoadsMachine::generateRoadNetwork(/*Terrain &terrain,*/
		PlaceTypesMainClass &placeTypesIn,
		RoadGraph &inRoadGraph)
	{
		//std::cout << time(NULL) << " " <<rand() << " ";
		//printf("***>>>> placeTypesIn %d\n",placeTypesIn.myPlaceTypes.size());
		inRoadGraph.clear();

		/*if(G::global().getInt("num_place_types")==1){/// !!! REMOVE
			placeTypesIn.myPlaceTypes.push_back(placeTypesIn.myPlaceTypes[0]);/// !!! REMOVE
			placeTypesIn.myPlaceTypes[1].myAttributes.pt_pt=QVector3D(300,300,placeTypesIn.myPlaceTypes.at(0).myAttributes.pt_pt.z()); /// !!! REMOVE
			placeTypesIn.myPlaceTypes[1].myAttributes.pt_building_height_mean=40;
			G::global().getInt("num_place_types")++;
		}*/
		

		//======== create a list of initial seeds
		std::list<RoadGraph::roadGraphVertexDesc> initSeeds;
		std::list<RoadGraph::roadGraphVertexDesc> newSeeds;

		generateInitialSeeds(placeTypesIn, initSeeds, inRoadGraph);

		//======== create initial edges
		generateInitialEdges(G::boundingPolygon, inRoadGraph);
		std::cout << "num edges: " << boost::num_edges(inRoadGraph.myRoadGraph) << "\n";
		std::cout << "num vertices: " << boost::num_vertices(inRoadGraph.myRoadGraph) << "\n";
		//======== start growth
		int iteCount = 0;
		int tmpPtIdx;
		bool newSeedsCreated;
		RoadGraph::roadGraphVertexDesc tmpSeedDesc;

		for(int i=0; i<G::global().getInt("num_place_types")/*placeTypesIn.myPlaceTypes.size()*/; ++i){
			placeTypesIn.myPlaceTypes[i]["pt_cur_edges_count"] = 0;
		}

		////////////////////////////////
		// force some edges

		/*{// REMOVE
			RoadGraph::roadGraphVertexDesc tgtVertexDesc;
			RoadGraph::roadGraphVertexDesc tgtVertexDesc2;
			RoadGraph::roadGraphVertexDesc tgtVertexDesc3;

			//tgtVertexDesc = boost::add_vertex(inRoadGraph.myRoadGraph);

			//{
			//	inRoadGraph.myRoadGraph[tgtVertexDesc].pt = QVector3D(0,0,0);
			//	inRoadGraph.myRoadGraph[tgtVertexDesc].isSeed = true;
			//	inRoadGraph.myRoadGraph[tgtVertexDesc].isBoundingPgonVertex =false;
			//		//inRoadGraph.myRoadGraph[initSeeds.front()].isBoundingPgonVertex;			
			//	inRoadGraph.myRoadGraph[tgtVertexDesc].myPlaceTypeIdx =
			//		inRoadGraph.myRoadGraph[initSeeds.front()].myPlaceTypeIdx;
			//	inRoadGraph.myRoadGraph[tgtVertexDesc].width =
			//		inRoadGraph.myRoadGraph[initSeeds.front()].width;
			//}
			
			tgtVertexDesc2 = boost::add_vertex(inRoadGraph.myRoadGraph);

			{
				inRoadGraph.myRoadGraph[tgtVertexDesc2].pt = QVector3D(400,400,0);
				inRoadGraph.myRoadGraph[tgtVertexDesc2].isSeed = true;
				inRoadGraph.myRoadGraph[tgtVertexDesc2].isBoundingPgonVertex =false;
					//inRoadGraph.myRoadGraph[initSeeds.front()].isBoundingPgonVertex;			
				inRoadGraph.myRoadGraph[tgtVertexDesc2].myPlaceTypeIdx =
					inRoadGraph.myRoadGraph[initSeeds.front()].myPlaceTypeIdx;
				inRoadGraph.myRoadGraph[tgtVertexDesc2].width =
					inRoadGraph.myRoadGraph[initSeeds.front()].width;
			}

			tgtVertexDesc3 = boost::add_vertex(inRoadGraph.myRoadGraph);

			{
				inRoadGraph.myRoadGraph[tgtVertexDesc3].pt = QVector3D(400,50,0);
				inRoadGraph.myRoadGraph[tgtVertexDesc3].isSeed = true;
				inRoadGraph.myRoadGraph[tgtVertexDesc3].isBoundingPgonVertex =false;
					//inRoadGraph.myRoadGraph[initSeeds.front()].isBoundingPgonVertex;			
				inRoadGraph.myRoadGraph[tgtVertexDesc3].myPlaceTypeIdx =
					inRoadGraph.myRoadGraph[initSeeds.front()].myPlaceTypeIdx;
				inRoadGraph.myRoadGraph[tgtVertexDesc3].width =
					inRoadGraph.myRoadGraph[initSeeds.front()].width;
			}

			//initSeeds.push_back(tgtVertexDesc);
			initSeeds.push_back(tgtVertexDesc2);
			initSeeds.push_back(tgtVertexDesc3);

			{
				//add two new edges from old edge endpoints to new vertex (split edge)
				std::pair<RoadGraph::roadGraphEdgeDesc, bool> e0_pair
					= boost::add_edge(  initSeeds.front(), tgtVertexDesc2, inRoadGraph.myRoadGraph );
				inRoadGraph.myRoadGraph[e0_pair.first].roadSegmentWidth = inRoadGraph.myRoadGraph[initSeeds.front()].width;

			}
			{
				//add two new edges from old edge endpoints to new vertex (split edge)
				std::pair<RoadGraph::roadGraphEdgeDesc, bool> e0_pair
					= boost::add_edge(  tgtVertexDesc2, tgtVertexDesc3, inRoadGraph.myRoadGraph );
				inRoadGraph.myRoadGraph[e0_pair.first].roadSegmentWidth = inRoadGraph.myRoadGraph[initSeeds.front()].width;

			}
			{
				//add two new edges from old edge endpoints to new vertex (split edge)
				std::pair<RoadGraph::roadGraphEdgeDesc, bool> e0_pair
					= boost::add_edge(  initSeeds.front(), tgtVertexDesc3, inRoadGraph.myRoadGraph );
				inRoadGraph.myRoadGraph[e0_pair.first].roadSegmentWidth = inRoadGraph.myRoadGraph[initSeeds.front()].width;

			}

		}//*/


	

		//======= grow arterials
		while(!initSeeds.empty()
			//&& iteCount < 10*(int)(LC::misctools::Global::global()->arterial_edges_count)
			//&& iteCount < max(50000, 10*(int)(LC::misctools::Global::global()->arterial_edges_count))
			&& iteCount < 1000
			)
		{
			//std::cout << "ga ";
			tmpSeedDesc = initSeeds.front();

			tmpPtIdx = inRoadGraph.myRoadGraph[tmpSeedDesc].myPlaceTypeIdx;

			//std::cout << "\n cur " <<placeTypesIn.at(tmpPtIdx).myAttributes.pt_cur_edges_count
			//	<< "max " << placeTypesIn.at(tmpPtIdx).myAttributes.pt_max_edges_count;

			//check that idx is within range
			if(tmpPtIdx < 0 || tmpPtIdx >= /*placeTypesIn.myPlaceTypes.size()*/G::global().getInt("num_place_types")){
				newSeedsCreated = false;
				//if the place type associated to this seed
				//		has already too many road segments, exit
			} else {
				//if(placeTypesIn.myPlaceTypes.at(tmpPtIdx).myAttributes.pt_cur_edges_count >=
				//placeTypesIn.myPlaceTypes.at(tmpPtIdx).myAttributes.pt_max_edges_count)
				float distToPTCenter = (placeTypesIn.myPlaceTypes.at(tmpPtIdx).getQVector3D("pt_pt") -
					inRoadGraph.myRoadGraph[tmpSeedDesc].pt).length();
				//if( distToPTCenter > placeTypesIn.myPlaceTypes.at(tmpPtIdx).myAttributes.pt_radius){
				if( false ){
					newSeedsCreated = false;
				}			
				//otherwise
				else
				{
					//try to expand from seed
					float refAng = placeTypesIn.myPlaceTypes.at(tmpPtIdx).getFloat("pt_orientation");
					attemptExpansion(tmpSeedDesc, inRoadGraph, newSeeds, refAng);
					newSeedsCreated = !(newSeeds.empty());
				}
			}

			if(newSeedsCreated){
				//add new seeds to this type
				placeTypesIn.myPlaceTypes.at(tmpPtIdx)["pt_cur_edges_count"] =
					placeTypesIn.myPlaceTypes.at(tmpPtIdx).getInt("pt_cur_edges_count")+newSeeds.size();

				//append seeds in newSeeds to initSeeds
				initSeeds.splice(initSeeds.end(), newSeeds);
			}

			//remove seed from initSeeds
			initSeeds.pop_front();

			iteCount++;

			//printf("Num graph vertices: %d\n", roadGraph.myRoadGraph.m_vertices.size());
			//printf("Queue sz: %d\n", initSeeds.size());
		}

		//Remove dead ends

		removeDeadEnds(inRoadGraph);


		//==============================================
		//======= grow streets
		//==============================================

		if(true/*LC::misctools::Global::global()->streets_compute*/){

			std::list<RoadGraph::roadGraphVertexDesc> initStreetSeeds;

			if(!generateInitialStreetSeeds(placeTypesIn, initStreetSeeds, inRoadGraph)){
				return false;
			}

			iteCount = 0;
			while(!initStreetSeeds.empty()
				//&& iteCount < 10*(int)(LC::misctools::Global::global()->arterial_edges_count)
				&& iteCount < 1000
				)
			{


				tmpSeedDesc = initStreetSeeds.front();
				tmpPtIdx = inRoadGraph.myRoadGraph[tmpSeedDesc].myPlaceTypeIdx;

				if(tmpPtIdx < 0 || tmpPtIdx >= /*placeTypesIn.myPlaceTypes.size()*/G::global().getInt("num_place_types")){
					newSeedsCreated = false;				
				} else {		

					bool isPointWithinLimits = false;

					//float distToPTCenter = (placeTypesIn.myPlaceTypes.at(tmpPtIdx).myAttributes.pt_pt -
					//	inRoadGraph.myRoadGraph[tmpSeedDesc].pt).length();								

					//if(distToPTCenter < placeTypesIn.myPlaceTypes.at(tmpPtIdx).myAttributes.pt_radius){

					if(placeTypesIn.myPlaceTypes.at(tmpPtIdx).containsPoint(inRoadGraph.myRoadGraph[tmpSeedDesc].pt)){

						if( LC::misctools::isPointWithinLoop(
							G::boundingPolygon,
							inRoadGraph.myRoadGraph[tmpSeedDesc].pt) )
						{
							isPointWithinLimits = true;
						}
					}

					if(!isPointWithinLimits){
						newSeedsCreated = false;
					} else {							
						float refAng = 0.0f;
						attemptExpansion(tmpSeedDesc, inRoadGraph, newSeeds, refAng);
						newSeedsCreated = !(newSeeds.empty());	
					}
				}


				if(newSeedsCreated){
					//append seeds in newSeeds to initSeeds
					initStreetSeeds.splice(initStreetSeeds.end(), newSeeds);
				}

				//remove seed from initSeeds
				initStreetSeeds.pop_front();

				iteCount++;
			}

			//Remove dead ends
			removeDeadEnds(inRoadGraph);

		}

		////////////////////////////////////////
		// N !!!!!!
		////////////////////////////////////
		// Create BI network
		inRoadGraph.myRoadGraph_BI.clear();
		RoadGraph::roadGraphVertexIter vi, viEnd;
		for(boost::tie(vi, viEnd) = boost::vertices(inRoadGraph.myRoadGraph);
				vi != viEnd; ++vi)
		{
					RoadGraph::roadGraphVertexDesc_BI tgtVertexDesc = boost::add_vertex(inRoadGraph.myRoadGraph_BI);

				{
					inRoadGraph.myRoadGraph_BI[tgtVertexDesc].pt = inRoadGraph.myRoadGraph[tgtVertexDesc].pt;
					inRoadGraph.myRoadGraph_BI[tgtVertexDesc].isSeed = inRoadGraph.myRoadGraph[tgtVertexDesc].isSeed;
					inRoadGraph.myRoadGraph_BI[tgtVertexDesc].isBoundingPgonVertex =inRoadGraph.myRoadGraph[tgtVertexDesc].isBoundingPgonVertex;			
					inRoadGraph.myRoadGraph_BI[tgtVertexDesc].myPlaceTypeIdx = inRoadGraph.myRoadGraph[tgtVertexDesc].myPlaceTypeIdx;
					inRoadGraph.myRoadGraph_BI[tgtVertexDesc].width =inRoadGraph.myRoadGraph[tgtVertexDesc].width;
				}
		}

		// 2. Copy edges duplicated
		RoadGraph::roadGraphEdgeIter ei, eiEnd;
		float speedArterialStreet=0.5f;/// REMOVE !!! 0.5f to not use streets
		for(boost::tie(ei, eiEnd) = boost::edges(inRoadGraph.myRoadGraph);
			ei != eiEnd; ++ei)
		{	
			//std::pair<RoadGraph::roadGraphEdgeDesc, bool> e0_pair
			//			= boost::edge(  vDesc, vDesc1, inRoadGraph.myRoadGraph );
			RoadGraph::roadGraphVertexDesc_BI srcVertexDesc=boost::source(*ei,inRoadGraph.myRoadGraph);
			RoadGraph::roadGraphVertexDesc_BI tgtVertexDesc=boost::target(*ei,inRoadGraph.myRoadGraph);
			int numLanes=0;
			std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e0_pair
						= boost::add_edge(  srcVertexDesc, tgtVertexDesc, inRoadGraph.myRoadGraph_BI );
					inRoadGraph.myRoadGraph_BI[e0_pair.first].roadSegmentWidth = inRoadGraph.myRoadGraph[tgtVertexDesc].width;

					if(inRoadGraph.myRoadGraph[*ei].roadSegmentWidth==G::global().getFloat("arterial_edges_width")){
						inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes =G::global().getInt("cuda_arterial_numLanes");
						numLanes+=inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes;
						inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeedMperSec =G::global().getFloat("cuda_arterial_edges_speed_ms");
					}else{
						inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes = G::global().getInt("cuda_road_numLanes");
						numLanes+=inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes;
						inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeedMperSec =G::global().getFloat("cuda_arterial_edges_speed_ms")*speedArterialStreet;
						//printf("mS %f\n",inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeed);
					}
					
					
					//inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeedCellsPerDelta = inRoadGraph.myRoadGraph[*ei].maxSpeedCellsPerDelta;
			e0_pair
						= boost::add_edge(  tgtVertexDesc,srcVertexDesc, inRoadGraph.myRoadGraph_BI );
					inRoadGraph.myRoadGraph_BI[e0_pair.first].roadSegmentWidth = inRoadGraph.myRoadGraph[tgtVertexDesc].width;
					if(inRoadGraph.myRoadGraph[*ei].roadSegmentWidth==G::global().getFloat("arterial_edges_width")){
						inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes = G::global().getInt("cuda_arterial_numLanes");
						numLanes+=inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes;
						inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeedMperSec =G::global().getFloat("cuda_arterial_edges_speed_ms");
					}else{
						inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes =G::global().getInt("cuda_road_numLanes");
						numLanes+=inRoadGraph.myRoadGraph_BI[e0_pair.first].numberOfLanes;
						inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeedMperSec =G::global().getFloat("cuda_arterial_edges_speed_ms")*speedArterialStreet;
					}
					//inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeed = inRoadGraph.myRoadGraph[*ei].maxSpeed;
					//inRoadGraph.myRoadGraph_BI[e0_pair.first].maxSpeedCellsPerDelta = inRoadGraph.myRoadGraph[*ei].maxSpeedCellsPerDelta;
			inRoadGraph.myRoadGraph[*ei].numberOfLanes =numLanes;
		}

		////////////////////////////////////
		// N update distances !!!!!
		QVector3D p0;
		QVector3D p1;		

		// 1. Update length normal
		//RoadGraph::roadGraphEdgeIter ei, eiEnd;
		for(boost::tie(ei, eiEnd) = boost::edges(inRoadGraph.myRoadGraph);
			ei != eiEnd; ++ei)
		{		
			p0 = inRoadGraph.myRoadGraph[boost::source(*ei, inRoadGraph.myRoadGraph)].pt;
			p1 = inRoadGraph.myRoadGraph[boost::target(*ei, inRoadGraph.myRoadGraph)].pt;
			inRoadGraph.myRoadGraph[*ei].edgeLength=(p0-p1).length();
			//inRoadGraph.myRoadGraph[*ei].edgeLengthCells=inRoadGraph.myRoadGraph[*ei].edgeLength/LC::misctools::Global::global()->cuda_cell_size;
			//inRoadGraph.myRoadGraph[*ei].maxSpeedCellsPerDelta=inRoadGraph.myRoadGraph[*ei].maxSpeed*LC::misctools::Global::global()->cuda_delta_time/LC::misctools::Global::global()->cuda_cell_size;
			//inRoadGraph.myRoadGraph[*ei].edgeLengthCells=inRoadGraph.myRoadGraph[*ei].edgeLength/LC::misctools::Global::global()->cuda_cell_size;
		}
		// 1. Update length BI
		RoadGraph::roadGraphEdgeIter_BI ei2, eiEnd2;
		float totalRoadLength=0;
		for(boost::tie(ei2, eiEnd2) = boost::edges(inRoadGraph.myRoadGraph_BI);
			ei2 != eiEnd2; ++ei2)
		{		
			p0 = inRoadGraph.myRoadGraph_BI[boost::source(*ei2, inRoadGraph.myRoadGraph_BI)].pt;
			p1 = inRoadGraph.myRoadGraph_BI[boost::target(*ei2, inRoadGraph.myRoadGraph_BI)].pt;
			inRoadGraph.myRoadGraph_BI[*ei2].edgeLength=(p0-p1).length();
			totalRoadLength+=(inRoadGraph.myRoadGraph_BI[*ei2].edgeLength*inRoadGraph.myRoadGraph_BI[*ei2].numberOfLanes);
			//inRoadGraph.myRoadGraph_BI[*ei2].edgeLengthCells=inRoadGraph.myRoadGraph_BI[*ei2].edgeLength/LC::misctools::Global::global()->cuda_cell_size;
			//inRoadGraph.myRoadGraph_BI[*ei2].maxSpeedCellsPerDelta=inRoadGraph.myRoadGraph_BI[*ei2].maxSpeed*LC::misctools::Global::global()->cuda_delta_time/LC::misctools::Global::global()->cuda_cell_size;
			//printf("max %f maxPeC %f\n",inRoadGraph.myRoadGraph_BI[*ei2].maxSpeed,inRoadGraph.myRoadGraph_BI[*ei2].maxSpeedCellsPerDelta);
			//inRoadGraph.myRoadGraph_BI[*ei2].edgeLengthCells=inRoadGraph.myRoadGraph_BI[*ei2].edgeLength/LC::misctools::Global::global()->cuda_cell_size;
		}

		printf("Num streets %d Num Nodes %d Total Length %f\n",boost::num_edges(inRoadGraph.myRoadGraph),boost::num_vertices(inRoadGraph.myRoadGraph),totalRoadLength);
		return true;
		//std::cout << "done growing streets\n";

	}

	/**
	* Check if segment AB is redundant, i.e., if the angle between segment AB and all the edges incoming to the vertex
	* with center in B is too small
	* This function DOES NOT modify the graph
	**/
	bool isSegmentRedundant (QVector3D &a, QVector3D &b, RoadGraph::roadGraphVertexDesc &vtxDesc,
		RoadGraph &inRoadGraph, float angleThreshold)
	{
		RoadGraph::roadGraphAdjIter ai, ai_end;
		float ang;

		//iterate all incident edges to vtxDesc			
		for (boost::tie(ai, ai_end) = boost::adjacent_vertices(vtxDesc, inRoadGraph.myRoadGraph);
			ai != ai_end; ++ai)
		{
			//compute angle between new potential segment and all other segments incident to vi
			ang = LC::misctools::angleThreePoints(a, b, inRoadGraph.myRoadGraph[*ai].pt);

			//if angle is too small, segment is redundant
			if( ang < angleThreshold )
			{
				return true;
			}
		}
		return false;
	}

	/**
	* Attempts snapping targetVtx to the graphs 
	* This function DOES NOT modify the graph
	**/
	bool snapsToGraphVertices(QVector3D &sourceVtxPt, QVector3D &targetVtxPt, RoadGraph &inRoadGraph,
		RoadGraph::roadGraphVertexDesc &tgtVertexDesc, float threshold = 1.0f)
	{
		float ang;
		float distToClosest = FLT_MAX;
		float curDist;

		RoadGraph::roadGraphVertexIter vi, viEnd;

		for(boost::tie(vi, viEnd) = boost::vertices(inRoadGraph.myRoadGraph); vi != viEnd; ++vi)
		{
			//if distance between points is small
			curDist = (inRoadGraph.myRoadGraph[*vi].pt - targetVtxPt).lengthSquared();
			if( curDist < distToClosest ){			
				distToClosest = curDist;
				tgtVertexDesc = boost::vertex(*vi, inRoadGraph.myRoadGraph);
			}
		}

		if(distToClosest < threshold*threshold){
			targetVtxPt = inRoadGraph.myRoadGraph[tgtVertexDesc].pt;
			return true;		
		}

		return false;
	}


	/**
	* Checks if new edge will intersect an existing edge
	* This function DOES NOT modify the graph
	**/
	bool intersectsOtherGraphEdge(QVector3D &p0, QVector3D &p1, RoadGraph &inRoadGraph,
		RoadGraph::roadGraphVertexDesc &closest_ei_source_desc,
		RoadGraph::roadGraphVertexDesc &closest_ei_target_desc,
		RoadGraph::roadGraphEdgeIter &eiClosest,
		QVector3D &closestIntPt)
	{
		QVector2D p0_2D(p0.toVector2D());
		QVector2D p1_2D(p1.toVector2D());
		QVector2D intPt;
		float distToInt;
		float distToClosestInt = FLT_MAX;
		float tab, tcd;
		RoadGraph::roadGraphVertexDesc ei_source_desc, ei_target_desc;	
		bool intersect = false;

		RoadGraph::roadGraphEdgeIter ei, eiEnd;

		//Adapt edges ===============
		for(boost::tie(ei, eiEnd) = boost::edges(inRoadGraph.myRoadGraph);
			ei != eiEnd; ++ei)
		{	
			ei_source_desc = boost::source(*ei, inRoadGraph.myRoadGraph); 
			ei_target_desc = boost::target(*ei, inRoadGraph.myRoadGraph);

            QVector2D src_vector = inRoadGraph.myRoadGraph[ei_source_desc].pt.toVector2D();
            QVector2D tgt_vector = inRoadGraph.myRoadGraph[ei_target_desc].pt.toVector2D();

			//if new segment intersects other segment
            if (LC::misctools::segmentSegmentIntersectXY(p0_2D, p1_2D,
                                                         src_vector, tgt_vector,
                                                         &tab, &tcd, true, intPt))
			{
				distToInt = (p0 - intPt).lengthSquared();

				//make sure we get only closest segment
				if( distToInt < distToClosestInt ){
					distToClosestInt = distToInt;
					eiClosest = ei;
					closest_ei_source_desc = ei_source_desc;
					closest_ei_target_desc = ei_target_desc;
					closestIntPt = intPt;
					intersect = true;
				}
			}
		}	

		return intersect;
	}

	/**
	* Attempts expansion of a segment in all possible directions and adds new edges to roadGraph.
	* If new seeds are found, they are added to newSeeds.
	* This function DOES modify the graph
	**/
	void attemptExpansion(RoadGraph::roadGraphVertexDesc &srcVertexDesc, RoadGraph &inRoadGraph,
		std::list<RoadGraph::roadGraphVertexDesc> &newSeeds, float inRefAngle)
	{
		newSeeds.clear();

		QVector3D dirVec;
		QVector3D targetVtxPt;
		QVector3D closestIntPt;
		float theta;
		float deltaDist, deltaDistNoNoise;
		bool snap;
		bool intersects;	

		RoadGraph::roadGraphVertexDesc tgtVertexDesc;
		RoadGraph::roadGraphVertexDesc closest_ei_source_desc, closest_ei_target_desc;
		RoadGraph::roadGraphEdgeIter eiClosest;

		srand(inRoadGraph.myRoadGraph[srcVertexDesc].randSeed);

		int numDepDir = inRoadGraph.myRoadGraph[srcVertexDesc].departingDirections.size();

		//attempt expansion along each direction
		for(int i=0; i<numDepDir; ++i)
		{
			float curIrregularity =  inRoadGraph.myRoadGraph[srcVertexDesc].irregularity;

			//if( LC::misctools::genRand(0.0f, 1.0f) < 0.5f*curIrregularity){
			//	continue;
			//}
			//curIrregularity = 0.0f;

			theta = inRoadGraph.myRoadGraph[srcVertexDesc].departingDirections[i];
			dirVec.setX(cos(theta));
			dirVec.setY(sin(theta));
			dirVec.setZ(0.0f);

			snap = false;
			intersects = false;

			bool isU;

			deltaDistNoNoise = inRoadGraph.myRoadGraph[srcVertexDesc].getDistFromDirAngle(theta, inRefAngle, isU);
			deltaDist = deltaDistNoNoise + 
				LC::misctools::genRand(-0.3f*deltaDistNoNoise*curIrregularity,
				0.3f*deltaDistNoNoise*curIrregularity);

			//std::cout << deltaDist << " ";

			targetVtxPt = inRoadGraph.myRoadGraph[srcVertexDesc].pt + dirVec*deltaDist;		

			//check if target vertex is within bounding polygon
			/*if( !LC::misctools::isPointWithinLoop(
			G::boundingPolygon,
			targetVtxPt) )
			{
			continue;
			}*/

			// INTERSECTS -- If edge intersects other edge
			intersects = intersectsOtherGraphEdge(inRoadGraph.myRoadGraph[srcVertexDesc].pt, targetVtxPt, inRoadGraph,
				closest_ei_source_desc, closest_ei_target_desc, eiClosest, closestIntPt);
			if(intersects){
				targetVtxPt = closestIntPt;
			}		

			//the threshold should be the max between e.g. 1/4 of the length and e.g. 10m
			float threshold = std::max<float>(0.25f*deltaDistNoNoise, 20.0f);
			//float threshold = 10.0f;

			if(!intersects){
				// SNAPS -- attempt snapping to any other vertex
				snap = snapsToGraphVertices(inRoadGraph.myRoadGraph[srcVertexDesc].pt, targetVtxPt, inRoadGraph, tgtVertexDesc, threshold);
			} else {
				// SNAPS -- attemp snapping to the eiClosest vertices
				float distToEdgeSource = (inRoadGraph.myRoadGraph[boost::source(*eiClosest, inRoadGraph.myRoadGraph)].pt
					- targetVtxPt).length();
				float distToEdgeTarget = (inRoadGraph.myRoadGraph[boost::target(*eiClosest, inRoadGraph.myRoadGraph)].pt
					- targetVtxPt).length();

				if( distToEdgeSource < threshold || distToEdgeTarget < threshold)
				{
					if(distToEdgeSource < distToEdgeTarget){
						targetVtxPt = inRoadGraph.myRoadGraph[closest_ei_source_desc].pt;
						tgtVertexDesc = closest_ei_source_desc;					
					} else {
						targetVtxPt = inRoadGraph.myRoadGraph[closest_ei_target_desc].pt;
						tgtVertexDesc = closest_ei_target_desc;
					}
					snap = true;
				}			
			}

			// ANGLE REDUNDANCY -- if departing segment is redundant
			if( isSegmentRedundant(targetVtxPt, inRoadGraph.myRoadGraph[srcVertexDesc].pt, srcVertexDesc, inRoadGraph, 0.2f*M_PI) ){
				continue;
			}
			if(snap){
				if(isSegmentRedundant(inRoadGraph.myRoadGraph[srcVertexDesc].pt, targetVtxPt, tgtVertexDesc, inRoadGraph, 0.2f*M_PI)){
					continue;
				}
			}

			if(intersects && !snap){
				tgtVertexDesc = boost::add_vertex(inRoadGraph.myRoadGraph);

				{
					inRoadGraph.myRoadGraph[tgtVertexDesc].pt = targetVtxPt;
					inRoadGraph.myRoadGraph[tgtVertexDesc].isSeed = false;
					inRoadGraph.myRoadGraph[tgtVertexDesc].isBoundingPgonVertex =
						inRoadGraph.myRoadGraph[closest_ei_source_desc].isBoundingPgonVertex;			
					inRoadGraph.myRoadGraph[tgtVertexDesc].myPlaceTypeIdx =
						inRoadGraph.myRoadGraph[closest_ei_source_desc].myPlaceTypeIdx;
					inRoadGraph.myRoadGraph[tgtVertexDesc].width =
						inRoadGraph.myRoadGraph[closest_ei_source_desc].width;
				}

				{
					//add two new edges from old edge endpoints to new vertex (split edge)
					std::pair<RoadGraph::roadGraphEdgeDesc, bool> e0_pair
						= boost::add_edge(  closest_ei_source_desc, tgtVertexDesc, inRoadGraph.myRoadGraph );
					inRoadGraph.myRoadGraph[e0_pair.first].roadSegmentWidth = inRoadGraph.myRoadGraph[closest_ei_source_desc].width;

					std::pair<RoadGraph::roadGraphEdgeDesc, bool> e1_pair
						= boost::add_edge(  closest_ei_target_desc, tgtVertexDesc, inRoadGraph.myRoadGraph );				
					inRoadGraph.myRoadGraph[e1_pair.first].roadSegmentWidth = inRoadGraph.myRoadGraph[closest_ei_source_desc].width;

					//remove edge
					boost::remove_edge(*eiClosest, inRoadGraph.myRoadGraph);
				}

			}

			//int newRandSeed = (i*(inRoadGraph.myRoadGraph[srcVertexDesc].randSeed*4096 + 150889))%714025;
			//int newRandSeed = inRoadGraph.myRoadGraph[srcVertexDesc].randSeed + 1;
			int newRandSeed = (rand() + 188179)%714025;

			//std::cout << "ors: " << inRoadGraph.myRoadGraph[srcVertexDesc].randSeed << " nrs: " << newRandSeed <<  "      ";

			//float curCurvature =  LC::misctools::Global::global()->arterial_edges_curvature;
			float curCurvature =  0.5f*inRoadGraph.myRoadGraph[srcVertexDesc].curvature;

			//float newDeltaTheta = curCurvature*inRoadGraph.myRoadGraph[srcVertexDesc].deltaTheta
			//	+ curCurvature*LC::misctools::genRand(-1.0f, 1.0f);

			float newDeltaTheta = inRoadGraph.myRoadGraph[srcVertexDesc].deltaTheta + 
				+ curCurvature*LC::misctools::genRand(-1.0f, 1.0f);

			float refAngle = theta + newDeltaTheta;

			//Determine new deltaU and deltaV
			float newDistU;
			float newDistV;
			bool newIsSeed = false;
			bool newIsBoundingPgonVertex = false;
			//if(numDepDir != 4){

			newDistU = isU? deltaDist : inRoadGraph.myRoadGraph[srcVertexDesc].distU;
			newDistV = isU? inRoadGraph.myRoadGraph[srcVertexDesc].distV : deltaDist;


			RoadGraphVertex targetVtx(targetVtxPt,
				//isU? deltaDist : inRoadGraph.myRoadGraph[srcVertexDesc].distU,
				newDistU,
				//isU? inRoadGraph.myRoadGraph[srcVertexDesc].distV : deltaDist,			
				newDistV,
				refAngle,
				4,
				newRandSeed,
				newDeltaTheta,
				newIsSeed,
				newIsBoundingPgonVertex,
				inRoadGraph.myRoadGraph[srcVertexDesc].irregularity,
				inRoadGraph.myRoadGraph[srcVertexDesc].curvature,
				inRoadGraph.myRoadGraph[srcVertexDesc].width,
				inRoadGraph.myRoadGraph[srcVertexDesc].myPlaceTypeIdx,
				G::global().getFloat("arterial_edges_speed"));
			//(rand()*4096 + 150889)%714025);

			if(!snap && !intersects){
				tgtVertexDesc = boost::add_vertex(inRoadGraph.myRoadGraph);
			}

			//need to add descriptor of vertex snapped to

			//add edge to graph
			std::pair<RoadGraph::roadGraphEdgeDesc, bool> tmpRGED_pair
				= boost::add_edge(srcVertexDesc, tgtVertexDesc, inRoadGraph.myRoadGraph);

			//std::cout << inRoadGraph.myRoadGraph[srcVertexDesc].width << " ";

			//printf("Added edge %d - %d\n", srcVertexDesc, tgtVertexDesc);


			if(!snap && !intersects){ //if not snap, vertex must be initialized
				{
					inRoadGraph.myRoadGraph[tgtVertexDesc].pt = targetVtx.pt;
					inRoadGraph.myRoadGraph[tgtVertexDesc].departingDirections = targetVtx.departingDirections;
					inRoadGraph.myRoadGraph[tgtVertexDesc].distU = targetVtx.distU;
					inRoadGraph.myRoadGraph[tgtVertexDesc].distV = targetVtx.distV;			
					inRoadGraph.myRoadGraph[tgtVertexDesc].randSeed = newRandSeed;
					inRoadGraph.myRoadGraph[tgtVertexDesc].deltaTheta = newDeltaTheta;
					inRoadGraph.myRoadGraph[tgtVertexDesc].isSeed = newIsSeed;
					inRoadGraph.myRoadGraph[tgtVertexDesc].isBoundingPgonVertex = newIsBoundingPgonVertex;			
					inRoadGraph.myRoadGraph[tgtVertexDesc].irregularity = targetVtx.irregularity;
					inRoadGraph.myRoadGraph[tgtVertexDesc].curvature = targetVtx.curvature;
					inRoadGraph.myRoadGraph[tgtVertexDesc].width = targetVtx.width;
					inRoadGraph.myRoadGraph[tgtVertexDesc].myPlaceTypeIdx = targetVtx.myPlaceTypeIdx;
				}

				//add target vertex to list of seeds
				newSeeds.push_back(tgtVertexDesc);
			}		

			inRoadGraph.myRoadGraph[tmpRGED_pair.first].roadSegmentWidth = inRoadGraph.myRoadGraph[srcVertexDesc].width;
		}

		/*inRoadGraph.myRoadGraph[srcVertexDesc].pt = inRoadGraph.myRoadGraph[srcVertexDesc].pt;
		inRoadGraph.myRoadGraph[srcVertexDesc].departingDirections = inRoadGraph.myRoadGraph[srcVertexDesc].departingDirections;
		inRoadGraph.myRoadGraph[srcVertexDesc].distU = inRoadGraph.myRoadGraph[srcVertexDesc].distU;
		inRoadGraph.myRoadGraph[srcVertexDesc].distV = inRoadGraph.myRoadGraph[srcVertexDesc].distV;
		inRoadGraph.myRoadGraph[srcVertexDesc].randSeed = inRoadGraph.myRoadGraph[srcVertexDesc].randSeed;
		inRoadGraph.myRoadGraph[srcVertexDesc].deltaTheta = inRoadGraph.myRoadGraph[srcVertexDesc].deltaTheta;*/
	}


	/**
	* Remove from graph vertices of degree one and the edge they are connected to
	**/

	void removeDeadEnds(RoadGraph &inRoadGraph)
	{
		RoadGraph::roadGraphVertexIter vi, viEnd;			
		RoadGraph::roadGraphEdgeIter ei, eiEnd;
		RoadGraph::roadGraphVertexDesc vDesc;

		//std::vector<RoadGraph::roadGraphVertexDesc> verticesToDelete;

		bool deadEndsLeft = true;

		for(int i=0; i<5 && deadEndsLeft; ++i){

			deadEndsLeft = false;

			//remove edges connecting to those vertices
			for(boost::tie(vi, viEnd) = boost::vertices(inRoadGraph.myRoadGraph);
				vi != viEnd; ++vi)
			{

				vDesc = boost::vertex(*vi, inRoadGraph.myRoadGraph);

				int outDeg = boost::out_degree( vDesc, inRoadGraph.myRoadGraph);
				//if( boost::out_degree( vDesc, inRoadGraph.myRoadGraph ) < 2){
				if( outDeg < 2){

					boost::clear_vertex(vDesc, inRoadGraph.myRoadGraph);
					//verticesToDelete.push_back(vDesc);
					deadEndsLeft = true;
				} else if ( outDeg == 2 && !inRoadGraph.myRoadGraph[vDesc].isBoundingPgonVertex){
					//printf("1.5\n");
					//if the degree is two (and the vertex is not in the bounding polygon)
					//		it means that it's just a change in geometry of the edge,
					//		and then we want to simplify the edge
					//so first, we create a new edge connecting the two vertices that are adjacent to vDesc

					RoadGraph::roadGraphVertexDesc vDesc0, vDesc1;
					RoadGraph::roadGraphAdjIter ai, ai_end;				
					int count = 0;

					for (boost::tie(ai, ai_end) = boost::adjacent_vertices(vDesc, inRoadGraph.myRoadGraph);
						ai != ai_end; ++ai)
					{	
						if(count == 0){
							vDesc0 = *ai;
						} else if(count == 1){
							vDesc1 = *ai;
						}
						count++;
					}

					std::pair<RoadGraph::roadGraphEdgeDesc, bool> edge_pair = 
						boost::add_edge( vDesc0, vDesc1, inRoadGraph.myRoadGraph );

					inRoadGraph.myRoadGraph[edge_pair.first].roadSegmentWidth = inRoadGraph.myRoadGraph[vDesc0].width;
					//printf("1.8\n");
					//and then we remove the vertex vDesc
					//boost::remove_vertex(vDesc, inRoadGraph.myRoadGraph);
					//boost::clear_vertex(vDesc, inRoadGraph.myRoadGraph);
					//printf("1.9\n");
				}
			}
		}
		//printf("1.10\n");
		//remove actual vertices
		//for(int i=0; i<verticesToDelete.size(); ++i)
		//{
		//	boost::remove_vertex(verticesToDelete[i], inRoadGraph.myRoadGraph);
		//}
	}


}
