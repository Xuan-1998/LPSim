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
*		MTC Project - Geometry Project - Zone Class
*
*
*
*		@desc Class containing the zone information
*		@author cvanegas
*
************************************************************************************************/
#ifndef GEOMETRY_ZONE_H
#define GEOMETRY_ZONE_H

#include <vector>
#include "block.h"

namespace LC {

	//namespace geometry{

		/**
		* Zone.
		**/
		class Zone			
		{

		public:
			/**
			* Constructor.
			**/
			Zone();

			/**
			* Destructor.
			**/
			~Zone();

			/**
			* Copy constructor.
			**/
			Zone(const Zone &ref)
			{					
				blocks= ref.blocks;
				zoneContour = ref.zoneContour;
			}

			/**
			* Assignment operator.
			**/
			inline Zone &operator=(const Zone &ref)
			{		
				blocks= ref.blocks;
				zoneContour = ref.zoneContour;
				return (*this);
			}

			/**
			* Clear
			**/
			void clear(void);
				

			/**
			* Adapt objects in zone to terrain
			**/
			//void adaptZoneToTerrain(MTC::geometry::ElevationGrid *elGrid);
			
			/**
			* Group list of parcels into blocks based on adjacencies and rights of way
			**/
			static void groupParcelsIntoBlocks(std::list<Parcel> &inParcels,
				std::vector<Block> &outBlocks, bool trivial);			

			/**
			* Collection of blocks inside the zone
			**/
			std::vector< Block > blocks;

			/**
			* Contour of the zone
			**/
			LC::misctools::Polygon3D zoneContour;			

		private:				
		};

	//}

}

#endif
