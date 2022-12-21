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
*		@desc Class containing the zone information
*		@author cvanegas
*
************************************************************************************************/

#include "zone.h"

namespace LC {

	//namespace geometry {

		/**
		* Constructor.
		**/

		Zone::Zone()
		{
		}

		/**
		* Destructor.
		**/

		Zone::~Zone()
		{
			this->clear();
		}

		/**
		* Clear
		**/
		void Zone::clear(void)
		{
			this->blocks.clear();
			this->zoneContour.contour.clear();
		}
		

		/**
		* Adapt zone to terrain
		**/
		/*void Zone::adaptZoneToTerrain(MTC::geometry::ElevationGrid *elGrid)
		{
			for(int i=0; i<blocks.size(); ++i){
				blocks[i].adaptBlockToTerrain(elGrid);
			}
		}*/
		
		/**
		* Group list of parcels into blocks based on adjacencies and rights of way
		**/
		void Zone::groupParcelsIntoBlocks(std::list<Parcel> &inParcels,
				std::vector<Block> &outBlocks, bool trivial) // trivial skips intersection and puts everything in one block
		{

			QTime dbTimer;
			dbTimer.start();
			std::cout << "START --- Grouping parcels into blocks\n";			

			std::list<Parcel> blockList;

			//while the list of initial parcels (LIP) is not empty
			//	(i.e., while there are still parcels that have not been assigned to blocks)

			int inParcelsSz = inParcels.size();

			while(!inParcels.empty()){

				if(inParcels.size() % 10 == 0){
					std::cout << "Parcels to process: " << inParcels.size() << "            \r";
					QCoreApplication::processEvents();
				}

				//start forming a new block
				Block newBlock;

				//get the first parcel in LIP, and move it to temporary list (Q)
				blockList.clear();
				blockList.push_back(inParcels.front());
				inParcels.pop_front();

				//while Q is not empty
				while(!blockList.empty()){

					//get the first element R in Q, and remove that element from LIP
					Parcel curParcel(blockList.front());
					blockList.pop_front();

					//for each parcel P in LIP
					std::list<Parcel>::iterator ite=inParcels.begin();
					while (ite != inParcels.end()){

						//determine if parcel P is adjacent to parcel R
						//if(ite->parcelContour.contour->
						if(trivial || curParcel.intersectsParcel(*ite)){
							blockList.push_back(*ite);
							ite = inParcels.erase(ite);
						} else {
							++ite;
						}

					}//endwhile

					//add R to current block											
					Block::parcelGraphVertexDesc tmpPGVD =
						boost::add_vertex(newBlock.myParcels);

					newBlock.myParcels[tmpPGVD] = curParcel;
				}//endwhile		

				outBlocks.push_back(newBlock);

			}//endwhile

			std::cout << "Parcels to process: " << inParcels.size() << "            \n";
			std::cout << inParcelsSz << " parcels have been grouped into " << outBlocks.size() << " blocks\n";
			std::cout << "END ----- Grouping parcels into blocks in " << dbTimer.elapsed() <<" ms \n";
		}
		
	//}// namespace geometry
}// namespace MTC
