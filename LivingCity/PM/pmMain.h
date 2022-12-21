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

#ifndef MTC_CLIENT_MAIN_PROCEURAL_MACHINE_H
#define MTC_CLIENT_MAIN_PROCEURAL_MACHINE_H

#include "../misctools/misctools.h"
//
#include "../Geometry/client_geometry.h"
#include "../Geometry/placeTypeInstances.h"

#include "pmRoads.h"
#include "pmBlocks.h"
#include "pmParcels.h"
#include "pmBuildings.h"

namespace LC {

	class ProceduralMachine
	{
	public:
		ProceduralMachine(){
		}

		~ProceduralMachine(){
		}

		//Generate Roads
		static bool generateRoadNetwork(/*MTC::geometry::Terrain &terrain,*/
			PlaceTypesMainClass &placeTypes,
			RoadGraph &roadGraph,
			std::vector< Block > &blocks,
			ClientGeometry* clientGeoPtr,
			//QGLWidget* glWidget,
			int flag);


		//Generate Blocks
		static bool generateBlocks(/*MTC::geometry::Terrain &terrain,*/
			PlaceTypesMainClass &placeTypesIn,
			RoadGraph &roadGraph,
			std::vector< Block > &blocks,
			ClientGeometry* clientGeoPtr//,
			//QGLWidget* glWidget
			);

		//Generate Parcels
		static bool generateParcels(//MTC::geometry::Terrain &terrain,
			PlaceTypesMainClass &placeTypesIn,
			std::vector< Block > &blocks,
			ClientGeometry* clientGeoPtr//,
			//QGLWidget* glWidget
			);

		//Generate Buildings
		static bool generateBuildings(//MTC::geometry::Terrain &terrain,
			PlaceTypesMainClass &placeTypesIn,
			std::vector< Block > &blocks//,
			//QGLWidget* glWidget
			);

	private:



	private:

	};



}



#endif
