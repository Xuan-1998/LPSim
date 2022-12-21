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
*		Procedural Parcels Machine
*
*
*		@desc Main
*		@author cvanegas
*
************************************************************************************************/

#include "pmParcels.h"

namespace LC {


	void subdivideBlockIntoParcels(Block &block, PlaceTypesMainClass &placeTypesIn);

	bool subdivideParcel(Block &block, Parcel parcel, float areaMean, float areaVar, float splitIrregularity,
		std::vector<Parcel> &outParcels); 


	void setParcelsAsParks(PlaceTypesMainClass &placeTypesIn, std::vector< Block > &blocks);


	void assignPlaceTypeToParcels(PlaceTypesMainClass &placeTypesIn, std::vector< Block > &blocks);

	bool ProceduralParcelsMachine::generateParcels(//Terrain &terrain,
		PlaceTypesMainClass &placeTypesIn,
		std::vector< Block > &blocks)
	{

		//std::cout << "\n";
		std::cout << "start #"<<blocks.size()<<"...";
		for(int i=0; i<blocks.size(); ++i)
		{	
			//std::cout << i << " out of " << blocks.size() << "\n"; fflush(stdout);
			subdivideBlockIntoParcels(blocks[i], placeTypesIn);
		}
		std::cout << "end...";

		//once all parcels have been generated, randomly select a percentage of them
		//	and set their land use to parks
		if(G::global().getInt("num_place_types")/*placeTypesIn.myPlaceTypes.size()*/>0){
			//std::cout << "\n Park percentage:" << parkPercentage << "\n";

			//assign parcels place type index
			assignPlaceTypeToParcels(placeTypesIn, blocks);

			//parks
			setParcelsAsParks(placeTypesIn, blocks);
			//std::cout << "\n Landmark\n";

		}
		return true;
	}

	void subdivideBlockIntoParcels(Block &block, PlaceTypesMainClass &placeTypesIn)
	{	
		//std::cout << "h0 ";	std::fflush(stdout);

		srand(block.randSeed);

		std::vector<Parcel> tmpParcels;

		//Empty parcels in block
		block.myParcels.clear();

		//Make the initial parcel of the block be the block itself
		//Parcel
		Parcel tmpParcel;
		tmpParcel.setContour(block.blockContour);

		//std::cout << block.myPlaceTypeIdx << " "; std::fflush(stdout);

		if( block.getMyPlaceTypeIdx() == -1){
			tmpParcels.push_back(tmpParcel);
		} else {
			//start recursive subdivision
			subdivideParcel(block, tmpParcel,
				placeTypesIn.myPlaceTypes.at(block.getMyPlaceTypeIdx()).getFloat("pt_parcel_area_mean"),
				placeTypesIn.myPlaceTypes.at(block.getMyPlaceTypeIdx()).getFloat("pt_parcel_area_deviation")/100.0f,
				placeTypesIn.myPlaceTypes.at(block.getMyPlaceTypeIdx()).getFloat("pt_parcel_split_deviation"),
				tmpParcels);
		}

		//printf("Block subdivided into %d parcels\n", tmpParcels.size());

		//Add parcels to block graph and compute adjacencies
		Block::parcelGraphVertexDesc tmpPGVD;
		//std::cout <<tmpParcels.size() <<"\n";
		for(int i=0; i<tmpParcels.size(); ++i){

			//std::cout << "h1a ";	std::fflush(stdout);

			//assign index of place type to parcel
			tmpParcels[i].setMyPlaceTypeIdx( block.getMyPlaceTypeIdx() );

			//add parcel to block parcels graph
			tmpPGVD = boost::add_vertex(block.myParcels);
			block.myParcels[tmpPGVD] = tmpParcels[i];

			//std::cout << "h2 ";	std::fflush(stdout);
		}
	}

	/**
	* Parcel subdivision
	* @desc: Recursive subdivision of a parcel using OBB technique
	* @return: true if parcel was successfully subdivided, false otherwise
	* @areaMean: mean target area of parcels after subdivision
	* @areaVar: variance of parcels area after subdivision (normalized in 0-1)
	* @splitIrregularity: A normalized value 0-1 indicating how far
	*					from the middle point the split line should be
	**/
	bool subdivideParcel(Block &block, Parcel parcel, float areaMean, float areaStd,
		float splitIrregularity, std::vector<Parcel> &outParcels)
	{
		//printf("subdivideParcel\n");
		//check if parcel is subdividable
		float thresholdArea = areaMean + LC::misctools::genRand(-1.0f, 1.0f)*areaStd*areaMean;
		//float thresholdArea = areaMean + LC::misctools::genRand(0.0f, 1.0f)*areaStd;
		if( (fabs(boost::geometry::area(parcel.bg_parcelContour))) < thresholdArea ){
			//printf("a: %.3f %.3f", boost::geometry::area(parcel.bg_parcelContour));
			//boost::geometry::correct(parcel.bg_parcelContour);
			//printf("a: %.3f %.3f", boost::geometry::area(parcel.bg_parcelContour));
			outParcels.push_back(parcel);
			return false;
		}

		//compute OBB
		QVector3D obbSize;
		QMatrix4x4 obbMat;
		parcel.parcelContour.getMyOBB(obbSize, obbMat);

		//compute split line passing through center of OBB TODO (+/- irregularity)
		//		and with direction parallel/perpendicular to OBB main axis
		QVector3D slEndPoint;
		QVector3D dirVectorInit, dirVector, dirVectorOrthogonal;
		QVector3D midPt(0.0f, 0.0f, 0.0f);
		QVector3D auxPt(1.0f, 0.0f, 0.0f);
		QVector3D midPtNoise(0.0f, 0.0f, 0.0f);
		std::vector<QVector3D> splitLine;	

		midPt = midPt*obbMat;

		dirVectorInit = (auxPt*obbMat - midPt);
		dirVectorInit.normalize();
		if(obbSize.x() > obbSize.y()){
			dirVector.setX( -dirVectorInit.y() );
			dirVector.setY(  dirVectorInit.x() );
		} else {
			dirVector = dirVectorInit;
		}

		midPtNoise.setX( splitIrregularity*LC::misctools::genRand(-10.0f, 10.0f) );
		midPtNoise.setY( splitIrregularity*LC::misctools::genRand(-10.0f, 10.0f) );
		midPt = midPt + midPtNoise;

		slEndPoint = midPt + 10000.0f*dirVector;
		splitLine.push_back(slEndPoint);
		slEndPoint = midPt - 10000.0f*dirVector;
		splitLine.push_back(slEndPoint);

		//split parcel with line and obtain two new parcels
		LC::misctools::Polygon3D pgon1, pgon2;

		float kDistTol = 0.01f;

		if( parcel.parcelContour.splitMeWithPolyline(splitLine, pgon1.contour, pgon2.contour) ){

			if(true ){//|| LC::misctools::Global::global()->force_street_access == true){
				//CHECK FOR STREET ACCESS
				//check if parcels have street access
				if(
					//check if new contours of pgon1 and pgon2 "touch" the boundary of the block (
					LC::misctools::Polygon3D::distanceXYfromContourAVerticesToContourB ( pgon1.contour,
					block.blockContour.contour ) > kDistTol 
					||
					LC::misctools::Polygon3D::distanceXYfromContourAVerticesToContourB ( pgon2.contour,
					block.blockContour.contour ) > kDistTol  )
				{
					splitLine.clear();
					pgon1.contour.clear();
					pgon2.contour.clear();			

					//if they don't have street access, rotate split line by 90 degrees and recompute
					dirVectorOrthogonal.setX( -dirVector.y() );
					dirVectorOrthogonal.setY(  dirVector.x() );

					slEndPoint = midPt + 10000.0f*dirVectorOrthogonal;
					splitLine.push_back(slEndPoint);
					slEndPoint = midPt - 10000.0f*dirVectorOrthogonal;
					splitLine.push_back(slEndPoint);			

					parcel.parcelContour.splitMeWithPolyline(splitLine, pgon1.contour, pgon2.contour);
				}
			}

			Parcel parcel1;
			Parcel parcel2;

			parcel1.setContour(pgon1);
			parcel2.setContour(pgon2);

			//call recursive function for both parcels
			subdivideParcel(block, parcel1, areaMean, areaStd, splitIrregularity, outParcels);
			subdivideParcel(block, parcel2, areaMean, areaStd, splitIrregularity, outParcels);


		} else {
			return false;
		}

		return true;
	}

	bool compareFirstPartTuple (const std::pair<float,Parcel*> &i, const std::pair<float,Parcel*> &j) {
		return (i.first<j.first);
	}


	void assignPlaceTypeToParcels(PlaceTypesMainClass &placeTypesIn, std::vector< Block > &blocks)
	{
		bool useSamePlaceTypeForEntireBlock = false;

		Block::parcelGraphVertexIter vi, viEnd;
		for(int j=0; j<blocks.size(); ++j){		
			for(boost::tie(vi, viEnd) = boost::vertices(blocks.at(j).myParcels); vi != viEnd; ++vi){
				blocks.at(j).myParcels[*vi].setMyPlaceTypeIdx(-1);
			}
		}

		//New way
		for(int k=G::global().getInt("num_place_types")/*placeTypesIn.myPlaceTypes.size()*/-1; k>=0; --k){		
			for(int j=0; j<blocks.size(); ++j){				
				for(boost::tie(vi, viEnd) = boost::vertices(blocks.at(j).myParcels); vi != viEnd; ++vi)
				{
					if(useSamePlaceTypeForEntireBlock){
						blocks.at(j).myParcels[*vi].setMyPlaceTypeIdx( blocks.at(j).getMyPlaceTypeIdx() );
					} else {					
						QVector3D testPt;
						testPt = blocks.at(j).myParcels[*vi].bbox.midPt();

						if( placeTypesIn.myPlaceTypes.at(k).containsPoint(testPt) ){						
							blocks.at(j).myParcels[*vi].setMyPlaceTypeIdx( k );
						}					
					}
				}			
			}
		}
	}

	void setParcelsAsParks(PlaceTypesMainClass &placeTypesIn, std::vector< Block > &blocks)
	{
		for(int k=0; k<G::global().getInt("num_place_types")/*placeTypesIn.myPlaceTypes.size()*/; ++k){

			std::vector<Parcel*> parcelPtrs;

			bool isFirst = true;
			int seedOfFirstBlock = 0;

			//get all the parcels of that place type in an array
			for(int j=0; j<blocks.size(); ++j){			
				Block::parcelGraphVertexIter vi, viEnd;
				for(boost::tie(vi, viEnd) = boost::vertices(blocks.at(j).myParcels); vi != viEnd; ++vi)
				{
					if( blocks.at(j).myParcels[*vi].getMyPlaceTypeIdx() == k ){


						if(isFirst){
							seedOfFirstBlock = blocks.at(j).randSeed;
							isFirst = false;
						}

						blocks.at(j).myParcels[*vi].parcelType= PAR_WITH_BLDG;
						parcelPtrs.push_back( &(blocks.at(j).myParcels[*vi]) );

					}
				}
			}

			srand(seedOfFirstBlock);

			float parkPercentage = placeTypesIn.myPlaceTypes.at(k).getFloat("pt_park_percentage");
			//std::cout << "\n Park Percentage " << parkPercentage << "\n";
			//parkPercentage = 0.20f;

			//shuffle and select first parkPercentage %
			int numToSetAsParks = (int)(parkPercentage*( (float)(parcelPtrs.size()) ));
			std::random_shuffle( parcelPtrs.begin(), parcelPtrs.end() );

			int countMax = std::min<float>( parcelPtrs.size(), numToSetAsParks );
			for(int i=0; i < countMax ; ++i){
				(parcelPtrs.at(i))->parcelType=PAR_PARK;
			}
		}
	}

}
