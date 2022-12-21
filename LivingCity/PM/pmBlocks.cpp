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
#include "pmBlocks.h"
#include "../global.h"

namespace LC {

	RoadGraph * roadGraphPtr;
	std::vector< Block > * blocksPtr;
	LC::misctools::Loop3D blockContourTmp;
	std::vector< float > blockContourWidths;
	bool isFirstVertexVisited;

	bool goodVertexPlaceTypeFound;
	int maxPlaceTypeIdx;

	int curRandSeed;
	int curPlaceTypeIdx;

	struct output_visitor : public boost::planar_face_traversal_visitor
	{
		void begin_face()
		{
			blockContourTmp.clear();
			blockContourWidths.clear();
			isFirstVertexVisited = true;
			curRandSeed = 0;
		}

		void end_face()
		{
			if(blockContourTmp.size() > 2){
				Block newBlock;
				newBlock.blockContour.contour = blockContourTmp;		
				newBlock.blockContourRoadsWidths = blockContourWidths;
				newBlock.randSeed = curRandSeed;	
				blocksPtr->push_back(newBlock);
			}
		}
	};

	//Vertex visitor
	struct vertex_output_visitor : public output_visitor
	{
		template <typename Vertex> 
		void next_vertex(Vertex v) 
		{ 	
			//std::cout << v << " 
			if(  v >= 0 && v < boost::num_vertices(roadGraphPtr->myRoadGraph) ){
				blockContourTmp.push_back( (roadGraphPtr->myRoadGraph)[v].pt );

				//initialize block random seed from first street node random seed
				if(isFirstVertexVisited){
					isFirstVertexVisited = false;
					curRandSeed = ( (roadGraphPtr->myRoadGraph)[v].randSeed*4096 + 150889) % 714025;
				}
			}
		}

		template <typename Edge> 
		void next_edge(Edge e) 
		{ 
			int sIdx, tIdx;
			sIdx = boost::source(e, roadGraphPtr->myRoadGraph);
			tIdx = boost::target(e, roadGraphPtr->myRoadGraph);

			if(  sIdx >= 0 && sIdx < boost::num_vertices(roadGraphPtr->myRoadGraph) &&
				tIdx >= 0 && tIdx < boost::num_vertices(roadGraphPtr->myRoadGraph) ){			

				//blockContourWidths.push_back( 0.5f*(((roadGraphPtr->myRoadGraph)[e].numberOfLanes+1)*LC::misctools::Global::global()->roadLaneWidth));	//half of the width	
				blockContourWidths.push_back(3.5f);
			}
		}
	};

	/**
	* Remove intersecting edges of a graph
	**/
	bool removeIntersectingEdges(RoadGraph &roadGraph)
	{
		//QSet<RoadGraph::roadGraphEdgeIter*> edgesToRemove2;
		std::vector<RoadGraph::roadGraphEdgeIter> edgesToRemove;

		QVector2D a0, a1, b0, b1;
		QVector2D intPt;
		RoadGraph::roadGraphEdgeIter a_ei, a_ei_end;
		RoadGraph::roadGraphEdgeIter b_ei, b_ei_end;
		float ta0a1, tb0b1;

		for(boost::tie(a_ei, a_ei_end) = boost::edges(roadGraph.myRoadGraph); a_ei != a_ei_end; ++a_ei){
			a0 = QVector2D(roadGraph.myRoadGraph[boost::source(*a_ei, roadGraph.myRoadGraph)].pt);
			a1 = QVector2D(roadGraph.myRoadGraph[boost::target(*a_ei, roadGraph.myRoadGraph)].pt);

			//for(tie(b_ei, b_ei_end) = boost::edges(roadGraph.myRoadGraph); b_ei != b_ei_end; ++b_ei){
			for(b_ei = a_ei; b_ei != a_ei_end; ++b_ei){			

				if(b_ei != a_ei){
					b0 = QVector2D(roadGraph.myRoadGraph[boost::source(*b_ei, roadGraph.myRoadGraph)].pt);
					b1 = QVector2D(roadGraph.myRoadGraph[boost::target(*b_ei, roadGraph.myRoadGraph)].pt);

					if(LC::misctools::segmentSegmentIntersectXY(a0, a1, b0, b1, &ta0a1, &tb0b1, true, intPt) ){
						bool addEd=true;
						for(int eN=0;eN<edgesToRemove.size();eN++){
							if(edgesToRemove[eN]==b_ei){
								addEd=false;
								break;
							}
						}
						if(addEd)
							edgesToRemove.push_back(b_ei);
						//edgesToRemove2.insert(&b_ei);
					}
				}
			}		
		}

		for(int i=0; i<edgesToRemove.size(); ++i){	
			boost::remove_edge(*(edgesToRemove[i]), roadGraph.myRoadGraph);
		}

		if(edgesToRemove.size()>0){
			return true;
		} else {
			return false;
		}
	}//

	/**
	* Given a road network, this function extracts the blocks
	**/
	bool ProceduralBlocksMachine::generateBlocks(/*MTC::geometry::Terrain &terrain,*/
		PlaceTypesMainClass &placeTypesIn,
		RoadGraph &roadGraph,
		std::vector< Block > &blocks)
	{
		//printf("b1.1\n");
		roadGraphPtr = &roadGraph;
		blocksPtr = &blocks;
		blocksPtr->clear();
		//printf("b1.2\n");
		//std::cout << "Init num blocks is: " << blocksPtr->size() << std::endl;

		bool isPlanar = false;
		bool converges = true;

		maxPlaceTypeIdx = G::getInt("num_place_types");//placeTypesIn.myPlaceTypes.size();

		//Make sure graph is planar
		typedef std::vector< RoadGraph::roadGraphEdgeDesc > tEdgeDescriptorVector;
		std::vector<tEdgeDescriptorVector> embedding(boost::num_vertices(roadGraph.myRoadGraph));
		//printf("b1.3\n");
		int cont=0;
		while(!isPlanar && converges)
		{	
			if(cont>2){
				return false;
			}
			// Test for planarity		
			if (boost::boyer_myrvold_planarity_test(boost::boyer_myrvold_params::graph = roadGraph.myRoadGraph,
				boost::boyer_myrvold_params::embedding = &embedding[0]) ){
					//std::cout << "Input graph is planar" << std::endl;
					isPlanar = true;
			}
			else {			
				//std::cout << "Input graph is not planar" << std::endl;
				//Remove intersecting edges
				if(!removeIntersectingEdges(roadGraph) ){
					converges = false;
				}
			}
			cont++;
		}

		if(!isPlanar){
			std::cout << "ERROR: Graph could not be planarized (generateBlocks)\n";
			return false;
		}

		//Create edge index property map?	
		typedef std::map<RoadGraph::roadGraphEdgeDesc, size_t> EdgeIndexMap;
		EdgeIndexMap mapEdgeIdx;
		boost::associative_property_map<EdgeIndexMap> pmEdgeIndex(mapEdgeIdx);		
		RoadGraph::roadGraphEdgeIter ei, ei_end;	
		int edge_count = 0;
		for(boost::tie(ei, ei_end) = boost::edges(roadGraph.myRoadGraph); ei != ei_end; ++ei){
			mapEdgeIdx.insert(std::make_pair(*ei, edge_count++));	
		}

		//std::cout << "1..\n"; fflush(stdout);

		//Extract blocks from road graph using boost graph planar_face_traversal
		//std::cout << std::endl << "Vertices on the faces: " << std::endl;
		vertex_output_visitor v_vis;	
		boost::planar_face_traversal(roadGraph.myRoadGraph, &embedding[0], v_vis, pmEdgeIndex);

		//std::cout << "Num blocks after face traversal is: " << blocksPtr->size() << std::endl;

		//std::cout << "2..\n"; fflush(stdout);

		//Misc postprocessing operations on blocks =======

		int maxVtxCount = 0;
		int maxVtxCountIdx = -1; 
		float avgInsetArea = 0.0f;

		int numBadBlocks = 0;

		std::vector<float> blockAreas;

		LC::misctools::Loop3D blockContourInset;
		for(int i=0; i<blocks.size(); ++i){

			//assign default place type
			blocks[i].setMyPlaceTypeIdx(-1);

			//Reorient faces
			if(LC::misctools::Polygon3D::reorientFace(blocks[i].blockContour.contour)){
				std::reverse(blocks[i].blockContourRoadsWidths.begin(),
					blocks[i].blockContourRoadsWidths.end() - 1);
				//std::cout << "reorient\n";
			}

			//fix block geometry before calling function...
			LC::misctools::Loop3D cleanPgon;
			LC::misctools::Polygon3D::cleanLoop(blocks[i].blockContour.contour,
				cleanPgon, 5.0f);		

			//update widths			
			if(blocks[i].blockContour.contour.size() != cleanPgon.size()){

				//std::cout << "clean\n";

				int cleanPgonSz = cleanPgon.size();
				std::vector<float> cleanWidths(cleanPgonSz);

				for(int j=0; j<cleanPgonSz; ++j){
					//find element j in from clean polygon in original polygon
					//if element IS there, add to clean width array
					for(int k=0; k<blocks[i].blockContour.contour.size(); ++k){
						if( cleanPgon[j] == blocks[i].blockContour.contour[k] )
						{
							cleanWidths[(j-1+cleanPgonSz)%cleanPgonSz] = blocks[i].blockContourRoadsWidths[k];
							//std::cout << blocks[i].blockContourRoadsWidths[k] << " ";
							break;
						}			
					}
				}

				blocks[i].blockContour.contour = cleanPgon;
				blocks[i].blockContourRoadsWidths = cleanWidths;
				//std::cout << cleanPgon.size() << " " << cleanWidths.size() << "\n";

				blocks[i].myColor = QVector3D(0.5f, 0.7f, 0.8f);		

			}


			if( blocks[i].blockContour.contour.size() != blocks[i].blockContourRoadsWidths.size() ){
				std::cout << "Error: " << blocks[i].blockContour.contour.size() << " " << blocks[i].blockContourRoadsWidths.size() << "\n";
				blocks[i].blockContour.contour.clear();
				blockAreas.push_back(0.0f);
				numBadBlocks++;
				continue;
			}

			if(blocks[i].blockContour.contour.size() < 3){
				blockAreas.push_back(0.0f);
				numBadBlocks++;
				continue;
			}

			//Compute block offset	
			/*for(int wN=0;wN<blocks[i].blockContourRoadsWidths.size();wN++){// REMOVE
				printf("inset %d %f\n",i,blocks[i].blockContourRoadsWidths[wN]);
				blocks[i].blockContourRoadsWidths[wN]=3.5f;
			}*/
			float insetArea = blocks[i].blockContour.computeInset(
				blocks[i].blockContourRoadsWidths,
				blockContourInset);
			

			blocks[i].blockContour.contour = blockContourInset;


			blocks[i].blockContour.getBBox(blocks[i].bbox.minPt, blocks[i].bbox.maxPt);


			avgInsetArea += insetArea;
			blockAreas.push_back(insetArea);

			//assign place type to block ------------
			int validClosestPlaceTypeIdx = -1;

			//if(blocks.size() > 5)

			float distToValidClosestPlaceType = FLT_MAX;
			QVector3D testPt;
			testPt = blocks.at(i).bbox.midPt();


			//NEW WAY!
			for(int k=G::global().getInt("num_place_types")/*placeTypesIn.myPlaceTypes.size()*/-1; k>=0; --k){			
				if( placeTypesIn.myPlaceTypes.at(k).containsPoint(testPt) ){				
					validClosestPlaceTypeIdx = k;
				}			
			}

			blocks[i].setMyPlaceTypeIdx( validClosestPlaceTypeIdx );

			//---------------------------------------
		}
		avgInsetArea = avgInsetArea/ ( (float)(blockAreas.size() - numBadBlocks));

		//Remove the largest block
		float maxArea = -FLT_MAX;
		int maxAreaIdx = -1;
		for(int i=0; i<blocks.size(); ++i){
			if(blocks[i].blockContour.contour.size() < 3){
				continue;
			}
			//std::cout << "area: " << blockAreas[i] << "\n";
			if(blockAreas[i] > maxArea)
			{
				maxArea = blockAreas[i];
				maxAreaIdx = i;
			}
		}

		if(maxAreaIdx != -1){
			blocks.erase(blocks.begin()+maxAreaIdx);
			blockAreas.erase(blockAreas.begin()+maxAreaIdx);
		}

		return true;

	}//

}
