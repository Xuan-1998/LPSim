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

#pragma once

//#include "misctools.h"
//#include "global.h"
#include <QColor>

namespace LC{
	//namespace geometry{

class ParcelBuildingAttributes
{
public:
	ParcelBuildingAttributes(void);
	~ParcelBuildingAttributes(void);

	
	float att_stories;					//	number of stories
	float att_footprint_area;			//	m^2 of building footprint
	float att_frontage_width;			//	width of building front in m
	float att_setback;					//	distance of building from road in m			
	float att_facades_area;				//  total area of the facade of the building
	float att_orientation;
	int att_building_type;			
	bool att_is_park;
	bool att_is_landmark;
	bool att_is_square;
	QColor att_facade_color;					//  building main color
	QColor att_roof_color;					//  building roof color
	//LC::misctools::t_land_use myLandUse;
	

	/**
	* Copy constructor.
	**/
	ParcelBuildingAttributes(const ParcelBuildingAttributes &ref)
	{		
		att_stories = ref.att_stories;		
		att_footprint_area = ref.att_footprint_area;
		att_frontage_width = ref.att_frontage_width;
		att_setback = ref.att_setback;
		att_facades_area = ref.att_facades_area;
		att_orientation = ref.att_orientation;
		att_building_type = ref.att_building_type;			
		att_is_park = ref.att_is_park;
		att_is_landmark = ref.att_is_landmark;
		att_is_square = ref.att_is_square;
		att_facade_color = ref.att_facade_color;
		att_roof_color = ref.att_roof_color;
		//myLandUse = ref.myLandUse;
	}

	/**
	* Assignment operator.
	**/
	inline ParcelBuildingAttributes &operator=(const ParcelBuildingAttributes &ref)
	{	
		att_stories = ref.att_stories;		
		att_footprint_area = ref.att_footprint_area;
		att_frontage_width = ref.att_frontage_width;
		att_setback = ref.att_setback;
		att_facades_area = ref.att_facades_area;
		att_orientation = ref.att_orientation;
		att_building_type = ref.att_building_type;			
		att_is_park = ref.att_is_park;
		att_is_landmark = ref.att_is_landmark;
		att_is_square = ref.att_is_square;
		att_facade_color = ref.att_facade_color;
		att_roof_color = ref.att_roof_color;
		//myLandUse = ref.myLandUse;
		return (*this);
	}

};
	//}
}
