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
*		MTC Project - Geometry Project - Building Wrapper Class
*
*
*		@desc 
*		@author cvanegas
*
************************************************************************************************/
#ifndef GEOMETRY_BUILDING_WRAP_H
#define GEOMETRY_BUILDING_WRAP_H

#include <QSettings>
#include "misctools/misctools.h"
#include "parcelBuildingAttributes.h"



namespace LC {

	const int BLDG_NO_BLDG=0;// 0 NO BLDG
	const int BLDG_WITH_BLDG=1;	// 1 BLDG


	class Building{

	public:

		Building(void);

		/**
		* Destructor
		**/
		~Building(void);

		/**
		* Copy Constructor
		**/
		Building(const Building &ref)
		{					
			bldType = ref.bldType;
			numStories = ref.numStories;
			buildingFootprint = ref.buildingFootprint;
		}

		/**
		* Assignment operator.
		**/
		inline Building &operator=(const Building &ref)
		{	
			bldType = ref.bldType;
			numStories = ref.numStories;
			buildingFootprint = ref.buildingFootprint;
			return (*this);
		}

		/**
		* Data members
		**/
		misctools::Polygon3D buildingFootprint;
		int bldType;
		int numStories;

	};
}

#endif
