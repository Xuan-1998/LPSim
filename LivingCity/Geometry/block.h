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
*		MTC Project - Geometry Project - Block Class
*
*
*		@desc Class containing the blockinformation
*		@author cvanegas
*
************************************************************************************************/
#ifndef GEOMETRY_BLOCK_H
#define GEOMETRY_BLOCK_H
#ifndef Q_MOC_RUN
#include <boost/graph/adjacency_list.hpp>
#endif
//#include "misctools.h"
#include "parcel.h"

namespace LC {

	//namespace geometry{

		/**
		* Block.
		**/

		class Block
		{

		public:
			/**
			* Constructor.
			**/
			Block();

			/**
			* Destructor.
			**/
			~Block();

			/**
			* Copy constructor.
			**/
			Block(const Block &ref)
			{					
				blockContour = ref.blockContour;
				blockContourRoadsWidths = ref.blockContourRoadsWidths;
				myNeighborhoodID = ref.myNeighborhoodID;
				myCityID = ref.myCityID;
				myParcels = ref.myParcels;
				bbox = ref.bbox;
				randSeed = ref.randSeed;
				myPlaceTypeIdx = ref.myPlaceTypeIdx;
				myColor = ref.myColor;
			}

			/**
			* Assignment operator.
			**/
			inline Block &operator=(const Block &ref)
			{	
				blockContour = ref.blockContour;
				blockContourRoadsWidths = ref.blockContourRoadsWidths;
				myNeighborhoodID = ref.myNeighborhoodID;
				myCityID = ref.myCityID;
				myParcels = ref.myParcels;
				bbox = ref.bbox;
				randSeed = ref.randSeed;
				myPlaceTypeIdx = ref.myPlaceTypeIdx;
				myColor = ref.myColor;
				return (*this);
			}

			/**
			* Clear
			**/
			void clear(void);

			void computeMyBBox(void);
			
			inline void setMyPlaceTypeIdx(int inIdx)
			{
				myPlaceTypeIdx = inIdx;
			}

			inline int getMyPlaceTypeIdx(void)
			{
				return myPlaceTypeIdx;
			}
			
			inline int getMyNeighborhoodID(void)
			{
				return myNeighborhoodID;
			}

			inline int getMyCityID(void)
			{						
				return myCityID;
			}
			

			/**
			* Compute parcel adjacency graph
			**/
			void computeParcelAdjacencyGraph(void);

			void buildableAreaMock(void);

			static void findParcelFrontAndBackEdges(Block &inBlock, Parcel &inParcel,
				std::vector<int> &frontEdges,
				std::vector<int> &rearEdges,
				std::vector<int> &sideEdges );
						
			/**
			* Contour of the block.
			**/
			LC::misctools::Polygon3D blockContour;

			/**
			* Boundary road widths
			**/
			std::vector<float> blockContourRoadsWidths;

			/**
			* Adapt block to terrain
			**/
			//void adaptBlockToTerrain(MTC::geometry::ElevationGrid *elGrid);
						
			/**
			* BGL Graph of parcels into which block is subdivided.
			**/				 

			typedef boost::adjacency_list
				<boost::vecS, boost::vecS, boost::undirectedS, Parcel> parcelGraph;				

			typedef boost::graph_traits<parcelGraph>::vertex_descriptor parcelGraphVertexDesc;

			typedef boost::graph_traits<parcelGraph>::vertex_iterator parcelGraphVertexIter;

			typedef boost::graph_traits<parcelGraph>::edge_iterator parcelGraphEdgeIter;

			typedef boost::graph_traits<parcelGraph>::adjacency_iterator parcelGraphAdjIter;// Carlos

			bool splitBlockParcelsWithRoadSegment(std::vector<QVector3D> &roadSegmentGeometry,
				float roadSegmentWidth, LC::misctools::BBox roadSegmentBBox, std::list<Parcel> &blockParcels);

			bool areParcelsAdjacent(parcelGraphVertexIter &p0, parcelGraphVertexIter &p1);

			parcelGraph myParcels;

			/**
			* Pointer to my place type
			**/			
			QVector3D myColor;

			LC::misctools::BBox bbox;

			int randSeed;

		private:
			int myPlaceTypeIdx;
			int myNeighborhoodID;
			int myCityID;
		};
	//}
}

#endif
