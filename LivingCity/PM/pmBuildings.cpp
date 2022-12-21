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
*		Procedural Buildings Machine
*
*
*		@desc Main
*		@author cvanegas
*
************************************************************************************************/

#include "pmBuildings.h"

namespace LC {

	bool generateBlockBuildings(Block &inBlock, PlaceTypesMainClass &placeTypesIn);

	bool ProceduralBuildingsMachine::generateBuildings(
		PlaceTypesMainClass &placeTypesIn,
		std::vector< Block > &blocks)
	{

		//For each block
		for(int i=0; i<blocks.size(); ++i){
			srand(blocks[i].randSeed);
			generateBlockBuildings(blocks[i], placeTypesIn);
		}

		return true;
	}

	/**
	* Compute Building Footprint Polygon
	**/
	bool computeBuildingFootprintPolygon(float maxFrontage, float maxDepth,
		std::vector<int> &frontEdges,
		std::vector<int> &rearEdges, 
		std::vector<int> &sideEdges,
		misctools::Loop3D &buildableAreaCont,
		misctools::Loop3D &buildingFootprint)
	{

		if( (maxFrontage < 1.0f) || (maxDepth < 1.0f) ){
			buildingFootprint = buildableAreaCont;
			return true;
		}

		buildingFootprint.clear();

		int frontageIdx = -1;
		int frontageIdxNext;

		int baSz = buildableAreaCont.size();
		if(baSz < 3){
			return false;
		}

		float curLength;
		float maxBALength = -FLT_MAX;

		int thisIdx;
		int nextIdx;

		bool orientedCW = misctools::Polygon3D::reorientFace(buildableAreaCont, true);

		for(int i=0; i<frontEdges.size(); ++i){

			//std::cout << "i: " << i << "    FESz: " << frontEdges.size() <<  "\n"; std::fflush(stdout);
			thisIdx = frontEdges.at(i);
			if(orientedCW){			
				nextIdx = (thisIdx-1+baSz)%baSz;
			} else {
				nextIdx = (thisIdx+1)%baSz;		
			}
			curLength = (buildableAreaCont.at(thisIdx)-buildableAreaCont.at(nextIdx)).lengthSquared();		
			if(curLength > maxBALength){
				maxBALength = curLength;
				frontageIdx = thisIdx;
				frontageIdxNext = nextIdx;
			}
		}

		maxBALength = sqrt(maxBALength);

		if(frontageIdx == -1){
			return false;
		}

		//std::cout << "f: " << frontageIdx << "   n: " << frontageIdxNext << "    s: " << buildableAreaCont.size() << "\n";
		std::fflush(stdout);

		QVector3D frontPtA, frontPtB;
		QVector3D rearPtA,  rearPtB;

		QVector3D frontageCenter = 0.5f*(buildableAreaCont.at(frontageIdx) + buildableAreaCont.at(frontageIdxNext));
		QVector3D frontageVector = (buildableAreaCont.at(frontageIdx)-
			buildableAreaCont.at(frontageIdxNext)).normalized();
		QVector3D depthVector(frontageVector.y(), -frontageVector.x(), frontageVector.z());

		float actualFrontage = std::min<float>(maxFrontage, maxBALength);
		float actualDepth = maxDepth + misctools::genRand(-0.05, 0.05)*maxDepth;	

		frontPtA = frontageCenter - 0.5*actualFrontage*frontageVector;
		frontPtB = frontageCenter + 0.5*actualFrontage*frontageVector;
		rearPtA  = frontPtA + depthVector*actualDepth;
		rearPtB  = frontPtB + depthVector*actualDepth;

		buildingFootprint.push_back(rearPtA);
		buildingFootprint.push_back(rearPtB);
		buildingFootprint.push_back(frontPtB);
		buildingFootprint.push_back(frontPtA);	
		printf("buildingFootprint %d\n",buildingFootprint.size());
		return true;
	}

	bool generateParcelBuildings(Block &inBlock, Parcel &inParcel, PlaceTypesMainClass &placeTypesIn)
	{
		float probEmptyParcel = 0.0f;
		LC::misctools::Loop3D pContourCpy;

		if(inParcel.getMyPlaceTypeIdx() == -1){
			return false;
		}

		//if parcel is park, process
		if(inParcel.parcelType==PAR_PARK){//park
			//printf("PARK\n");
			return false;
		}

		//Compute parcel frontage
		std::vector<int> frontEdges;
		std::vector<int> rearEdges;
		std::vector<int> sideEdges;

		pContourCpy.clear();
		pContourCpy = inParcel.parcelContour.contour;
		inParcel.parcelContour.contour.clear();


		LC::misctools::Polygon3D::cleanLoop(pContourCpy, inParcel.parcelContour.contour, 1.0f);
		LC::misctools::Polygon3D::reorientFace(inParcel.parcelContour.contour);

		inBlock.findParcelFrontAndBackEdges(inBlock, inParcel, frontEdges, rearEdges, sideEdges);

		//Compute buildable area polygon
		float bldgFootprintArea = inParcel.computeBuildableArea(
			placeTypesIn.myPlaceTypes.at(inParcel.getMyPlaceTypeIdx()).getFloat("pt_parcel_setback_front"),
			placeTypesIn.myPlaceTypes.at(inParcel.getMyPlaceTypeIdx()).getFloat("pt_parcel_setback_rear"),
			placeTypesIn.myPlaceTypes.at(inParcel.getMyPlaceTypeIdx()).getFloat("pt_parcel_setback_sides"),
			frontEdges, rearEdges, sideEdges,
			inParcel.parcelBuildableAreaContour.contour);
		//printf("parcelBuilsable %d\n",inParcel.parcelBuildableAreaContour.contour.size());
		if(inParcel.parcelBuildableAreaContour.isSelfIntersecting()){
			inParcel.parcelBuildableAreaContour.contour.clear();
			return false;
		}		

		//compute building footprint polygon
		if(!computeBuildingFootprintPolygon(
			placeTypesIn.myPlaceTypes.at(inParcel.getMyPlaceTypeIdx()).getFloat("pt_building_max_frontage"),
			placeTypesIn.myPlaceTypes.at(inParcel.getMyPlaceTypeIdx()).getFloat("pt_building_max_depth"),
			frontEdges, rearEdges, sideEdges,
			inParcel.parcelBuildableAreaContour.contour,
			inParcel.myBuilding.buildingFootprint.contour))
		{
			printf("!computeBuildingFootprintPolygon\n");
			return false;
		}

		// stoties
		float heightDev = 
			(placeTypesIn.myPlaceTypes.at(inParcel.getMyPlaceTypeIdx()).getFloat("pt_building_height_deviation")/100.0f)*
			LC::misctools::genRand(-1.0f,1.0f)*
			placeTypesIn.myPlaceTypes.at(inParcel.getMyPlaceTypeIdx()).getFloat("pt_building_height_mean");

		int bldgNumStories = (int)
			placeTypesIn.myPlaceTypes.at(inParcel.getMyPlaceTypeIdx()).getFloat("pt_building_height_mean") +
			heightDev;

		//Set building
		inParcel.myBuilding.buildingFootprint=inParcel.myBuilding.buildingFootprint;
		inParcel.myBuilding.numStories=bldgNumStories;
		inParcel.myBuilding.bldType=BLDG_WITH_BLDG;

		return true;
	}


	bool generateBlockBuildings(Block &inBlock, PlaceTypesMainClass &placeTypesIn)
	{
		Block::parcelGraphVertexIter vi, viEnd;	

		//=== First compute parcel frontage and buildable area
		//For each parcel
		for(boost::tie(vi, viEnd) = boost::vertices(inBlock.myParcels); vi != viEnd; ++vi){					
			if(!generateParcelBuildings(inBlock, inBlock.myParcels[*vi], placeTypesIn)){
				continue;
			}
		}
		return true;
	}


	//}
}
