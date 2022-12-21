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
*		Procedural Machine Main
*
*
*		@desc Main
*		@author cvanegas
*
************************************************************************************************/

#include <qtimer.h>

#include "pmMain.h"
#include "../misctools/misctools.h"

namespace LC {

	/**
	* Generates new road network and calls function to recompute blocks, parcels, buildings
	**/

	bool ProceduralMachine::generateRoadNetwork(/*MTC::geometry::Terrain &terrain,*/
		PlaceTypesMainClass &placeTypesIn,
		RoadGraph &roadGraph,
		std::vector< Block > &blocks,
		ClientGeometry* clientGeoPtr,
		//QGLWidget* glWidget,
		int flag)
	{
		std::cout << "Generating roads...";

		bool res;

		QTime tim;
		tim.start();	
		//computes new road network
		res = ProceduralRoadsMachine::generateRoadNetwork(/*terrain, */placeTypesIn, roadGraph);
		std::cout << "\t" << tim.elapsed() << " ms\n";

		if(!res){
			std::cout << "Error generating roads. Procedural generation cancelled.\n";
			return false;
		}

		if(flag==ClientGeometry::kJustRoads){
			printf("JUST ROADS\n");
			return res;
		}

		//return true;//!!!!
		//given road network, compute new blocks
		res = generateBlocks(placeTypesIn, roadGraph, blocks, clientGeoPtr);// , glWidget);//ProceduralBlocksMachine::generateBlocks(/*terrain,*/ placeTypesIn, roadGraph, blocks);
		if(!res){
			return false;
		}
	
		return true;
	}
	
	/**
	* Generates new blocks given a road network
	**/
	bool ProceduralMachine::generateBlocks(/*MTC::geometry::Terrain &terrain,*/
		PlaceTypesMainClass &placeTypesIn,
		RoadGraph &roadGraph,
		std::vector< Block > &blocks,
		ClientGeometry* clientGeoPtr//,
		//QGLWidget* glWidget
		)
	{
		
		std::cout << "Generating blocks...";

		bool res;

		QTime tim;
		tim.start();	
		//printf("***************** edges curvature %f\n",clientGeoPtr->geoPlaceTypes.myPlaceTypes.at(0).myAttributes.pt_edges_curvature);
		//extracts new blocks from road network
		//printf("b1\n");
		res = ProceduralBlocksMachine::generateBlocks(/*terrain,*/ placeTypesIn, roadGraph, blocks);
		//printf("b2\n");
		std::cout << "\t" << tim.elapsed() << " ms\n";

		if(!res){
			std::cout << "Error generating blocks. Procedural generation cancelled.\n";
			return false;
		}
		
		//given blocks, compute new parcels
		res = generateParcels(placeTypesIn, blocks, clientGeoPtr);// , glWidget);
		if(!res){
			return false;
		}
		
		return true;
	}

	/**
	* Given a set of blocks, compute subdivision for the blocks
	**/
	bool ProceduralMachine::generateParcels(//MTC::geometry::Terrain &terrain,
		PlaceTypesMainClass &placeTypesIn,
		std::vector< Block > &blocks,
		ClientGeometry* clientGeoPtr//,
		//QGLWidget* glWidget
		)
	{
		if(true){//LC::misctools::Global::global()->parcel_compute){

			std::cout << "Generating parcels...";

			bool res;

			QTime tim;
			tim.start();

			//given blocks, compute new parcels
			res = ProceduralParcelsMachine::generateParcels(/*terrain,*/ placeTypesIn, blocks);
			std::cout << "\t" << tim.elapsed() << " ms\n";

			if(!res){
				std::cout << "Error generating parcels. Procedural generation cancelled.\n";
				return false;
			}

			res = generateBuildings(/*terrain,*/ placeTypesIn, blocks);// , glWidget);
			if(!res){
				return false;
			}

			return true;
		}
		return false;
	}


	/**
	* Compute buildable area of each parcel and generate buildings within
	**/
	bool ProceduralMachine::generateBuildings(
		PlaceTypesMainClass &placeTypesIn,
		std::vector< Block > &blocks//,
		//QGLWidget* glWidget
		)
	{
		std::cout << "Generating buildings...";

		QTime tim;
		tim.start();

		//given blocks with parcels, compute buildings
		ProceduralBuildingsMachine::generateBuildings( placeTypesIn, blocks);
		std::cout << "\t" << tim.elapsed() << " ms\n";
		return true;
	}
	
}
