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
*		MTC Project - Geometry Project - Parcel Class
*
*
*		@desc Class containing the parcel information
*		@author cvanegas
*
************************************************************************************************/
#ifndef GEOMETRY_PARCEL_H
#define GEOMETRY_PARCEL_H

#include <QSettings>
#include "building.h"

namespace LC {


		const int PAR_PARK=0;
		const int PAR_WITH_BLDG=1;
	
		/**
		* Parcel.
		**/
		class Parcel{

		public:

			/**
			* Interface
			**/
			LC::misctools::Polygon3D parcelContour;
			LC::misctools::Polygon3D parcelBuildableAreaContour;
		
			int parcelType;

			Building myBuilding;
			
			/**
			* Methods
			**/
			
			Parcel();
			~Parcel();

			Parcel(const Parcel &ref){					
				parcelContour = ref.parcelContour;
				parcelBuildableAreaContour = ref.parcelBuildableAreaContour;
				myBuilding = ref.myBuilding;
				bbox = ref.bbox;
				myPlaceTypeIdx = ref.myPlaceTypeIdx;
				initializeParcel();
			}


			inline Parcel &operator=(const Parcel &ref){	
				parcelContour = ref.parcelContour;
				parcelBuildableAreaContour = ref.parcelBuildableAreaContour;
				myBuilding = ref.myBuilding;				

				bbox = ref.bbox;
				initializeParcel();
				myPlaceTypeIdx = ref.myPlaceTypeIdx;
				initializeParcel();
				return (*this);
			}

			/**
			* Set Contour
			**/
			inline void setContour(LC::misctools::Polygon3D &inContour)
			{
				this->parcelContour = inContour;
				if(parcelContour.contour.size()>0){
					boost::geometry::assign(bg_parcelContour, parcelContour.contour);
				}
				initializeParcel();
			}

			/**
			* Initialize parcel
			**/
			void initializeParcel()
			{
				if(parcelContour.contour.size()>0){
					boost::geometry::assign(bg_parcelContour, parcelContour.contour);
					boost::geometry::correct(parcelContour.contour);
				}
				parcelContour.getBBox(bbox.minPt, bbox.maxPt);
			}

			
			/**
			* @brief: Returns true if parcels are adjacent			
			**/
			bool intersectsParcel(Parcel &other);

			/**			
			* @brief: Compute union of this parcel with a given parcel. The contour of the current parcel is modified. The other parcel is left unchanged.
			* @param[in] other: Parcel to union this parcel with.
			* @param[out] bool: True if union was computed. False if union fails. Union fails if parcels are not adjacent.
			**/
			int unionWithParcel(Parcel &other);
					
			/**
			* Export parcel contour to OBJ
			**/
			void writeParcelContourToOBJ(QTextStream &objStream);

			/**
			* Export parcel building to OBJ
			**/
			void writeParcelBuildingToOBJ(QTextStream &objStream);

			
			/**
			* Compute Parcel Buildable Area
			**/
			float computeBuildableArea(float frontSetback, float rearSetback, float sideSetback,
				std::vector<int> &frontEdges, 
				std::vector<int> &rearEdges, 
				std::vector<int> &sideEdges,
				LC::misctools::Loop3D &pgonInset);


			inline void setMyPlaceTypeIdx(int inIdx){
				myPlaceTypeIdx = inIdx;
			}

			inline int getMyPlaceTypeIdx(void){
				return myPlaceTypeIdx;
			}
			
			LC::misctools::BBox bbox;
			boost::geometry::ring_type<LC::misctools::Polygon3D>::type bg_parcelContour;
		private:	
			int myPlaceTypeIdx;

		};

}


#endif
