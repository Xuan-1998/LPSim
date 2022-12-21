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


#include "VBOBlocks.h"

#include "VBORenderManager.h"
#include "VBOBuilding.h"

namespace LC {

	bool VBOBlocks::initialized=false;

	static std::vector<QString> sideWalkFileNames;
	static std::vector<QVector3D> sideWalkScale;

	void VBOBlocks::clearAll(VBORenderManager& rendManager){
		rendManager.removeStaticGeometry("sidewalk",false);
		rendManager.removeStaticGeometry("building",false);
	}//


	void VBOBlocks::initVBOBlocks(){
		printf("initVBOBlocks\n");
		QString pathName="data/textures/LC";
		// 3. sidewalk
		QDir directorySW(pathName+"/sidewalk/");
		QStringList nameFilter;
		nameFilter << "*.png" << "*.jpg" << "*.gif";
		QStringList list = directorySW.entryList( nameFilter, QDir::Files );
		for(int lE=0;lE<list.size();lE++){
			if(QFile::exists(pathName+"/sidewalk/"+list[lE])){
				sideWalkFileNames.push_back(pathName+"/sidewalk/"+list[lE]);
				QStringList scaleS=list[lE].split("_");
				if(scaleS.size()!=4)
					sideWalkScale.push_back(QVector3D(1.0f,1.0f,0));
				else{
					sideWalkScale.push_back(QVector3D(scaleS[1].toFloat(),scaleS[2].toFloat(),0));
				}
			}
		}
		initialized=true;
	}//

	void VBOBlocks::generateVBOOneBlock(VBORenderManager& rendManager,Block &cBlock){
		//printf(">> >> >>generateVBOOneBlock\n");
		if (cBlock.blockContour.contour.size() > 50){
			printf("ERROR: Block with more than 50 sides\n");
			return;
		}
		bool blockWithBuilding=false;
		LC::Block::parcelGraphVertexIter vi, viEnd;
		//1. check if there is buildible area and if there 
		//for(int pN=0;pN<cBlock.myParcels.size();pN++){
		for(boost::tie(vi, viEnd) = boost::vertices(cBlock.myParcels); vi != viEnd; ++vi){
			blockWithBuilding|=cBlock.myParcels[*vi].parcelBuildableAreaContour.contour.size()>2;
			//printf("BLD %d\n",cBlock.myParcels[*vi].parcelBuildableAreaContour.contour.size());
		}
		//printf(">> >> >>generateVBOOneBlock1\n");
		//render an offset of the block to make it look like sidewalks and streets
		if(blockWithBuilding){//just render sidewalks and streets of blocks with buildings 

			float offsetSidewalks = 0.0f;//+3.5f; 
			
			float offsetArterials =-3.5f;//-((LC::misctools::Global::global()->cuda_arterial_numLanes+1)*LC::misctools::Global::global()->roadLaneWidth);	

			LC::misctools::Polygon3D tmpPgonSideWalk;
			LC::misctools::Polygon3D tmpPgonRoads;
			//printf(">> >> >>generateVBOOneBlock2\n");
			cBlock.blockContour.computeInset(offsetSidewalks, tmpPgonSideWalk.contour, false);
			//printf(">> >> >>generateVBOOneBlock21\n");
			cBlock.blockContour.computeInset(offsetArterials, tmpPgonRoads.contour, false); 
			//printf(">> >> >>generateVBOOneBlock22\n");
			printf(".");//MAGIC NOT REMOVE
			//printf("..");//MAGIC NOT REMOVE
			// SIDE WALK
			/*if(tmpPgonSideWalk.contour.size()==4){
				std::vector<Vertex> vert;
				vert.push_back(Vertex(tmpPgonSideWalk.contour[0]+QVector3D(0,0,1.5f),QVector3D(0,0,0)));
				vert.push_back(Vertex(tmpPgonSideWalk.contour[1]+QVector3D(0,0,1.5f),QVector3D(0.0f,1.0f,0)));
				vert.push_back(Vertex(tmpPgonSideWalk.contour[2]+QVector3D(0,0,1.5f),QVector3D(1.0f,1.0f,0)));
				vert.push_back(Vertex(tmpPgonSideWalk.contour[3]+QVector3D(0,0,1.5f),QVector3D(1.0f,0.0f,0)));

				
				
				//printf("%f %f %f\n",tmpPgonSideWalk.contour[0].x(),tmpPgonSideWalk.contour[0].y(),tmpPgonSideWalk.contour[0].z());
				rendManager.addStaticGeometry("sidewalk",vert,"data/textures/LC/sidewalk/pavement.jpg",GL_QUADS,11);
			}*/
			//printf(">> >> >>generateVBOOneBlock23\n");
			int randSidewalk=qrand()%sideWalkFileNames.size();
			//printf(">> >> >>generateVBOOneBlock24 %d <? %d %d | %d\n",randSidewalk,sideWalkFileNames.size(),sideWalkScale.size(),tmpPgonSideWalk.contour.size());
			rendManager.addStaticConvexPoly("sidewalk",tmpPgonSideWalk.contour,1.5f,false,sideWalkFileNames[randSidewalk],2|LC::mode_AdaptTerrain,sideWalkScale[randSidewalk]);
			//rendManager.addStaticGeometry("sidewalk",tmpPgonSideWalk.contour,1.5f,false,"data/textures/LC/sidewalk/pavement.jpg",3,QVector3D(1,1,1));
			//printf(">> >> >>generateVBOOneBlock3\n");
			std::vector<Vertex> vert;
			for(int sN=0;sN<tmpPgonSideWalk.contour.size();sN++){
				int ind1=sN;
				int ind2=(sN+1)%tmpPgonSideWalk.contour.size();
				QVector3D dir=tmpPgonSideWalk.contour[ind2]-tmpPgonSideWalk.contour[ind1];
				float length=dir.length();
				dir/=length;
				if(length<2.0f*5.0f){
					QVector3D p1=tmpPgonSideWalk.contour[ind1]+QVector3D(0,0, 1.0f);
					QVector3D p2=tmpPgonSideWalk.contour[ind2]+QVector3D(0,0, 1.0f);
					QVector3D p3=tmpPgonSideWalk.contour[ind2]+QVector3D(0,0, 1.5f);
					QVector3D p4=tmpPgonSideWalk.contour[ind1]+QVector3D(0,0, 1.5f);
					QVector3D normal=QVector3D::crossProduct(p2-p1,p4-p1).normalized();
					vert.push_back(Vertex(p1,QVector3D(0.5f,0.5f,0.5f),normal,QVector3D()));
					vert.push_back(Vertex(p2,QVector3D(0.5f,0.5f,0.5f),normal,QVector3D()));
					vert.push_back(Vertex(p3,QVector3D(0.5f,0.5f,0.5f),normal,QVector3D()));
					vert.push_back(Vertex(p4,QVector3D(0.5f,0.5f,0.5f),normal,QVector3D()));
				}else{
					float segmentLeng;
					const float maxSegmentLeng=5.0f;
					int numSegments=ceil(length/5.0f);
					QVector3D a0,a1,a2,a3,normal;
					a1=tmpPgonSideWalk.contour[ind1]+QVector3D(0,0, 1.0f);
					a2=tmpPgonSideWalk.contour[ind1]+QVector3D(0,0, 1.5f);
					for(int nS=0;nS<numSegments;nS++){
						segmentLeng=std::min(maxSegmentLeng,length);
						a0=a1;
						a3=a2;
						a1=a1+dir*(segmentLeng);
						a2=a2+dir*(segmentLeng);
						normal=QVector3D::crossProduct(a1-a0,a3-a0).normalized();
						vert.push_back(Vertex(a0,QVector3D(0.5f,0.5f,0.5f),normal,QVector3D()));
						vert.push_back(Vertex(a1,QVector3D(0.5f,0.5f,0.5f),normal,QVector3D()));
						vert.push_back(Vertex(a2,QVector3D(0.5f,0.5f,0.5f),normal,QVector3D()));
						vert.push_back(Vertex(a3,QVector3D(0.5f,0.5f,0.5f),normal,QVector3D()));
						length-=segmentLeng;
					}
				}
				
			}
			//printf(">> >> >>generateVBOOneBlock4\n");
			rendManager.addStaticGeometry("sidewalk",vert,"",GL_QUADS,1|LC::mode_AdaptTerrain);
			//rendManager.addStaticGeometry("sidewalk",tmpPgonSideWalk.contour,1.5f,false,"data/textures/LC/sidewalk/pavement.jpg");
			
			/*int randSW=qrand()%sidewalksTex.size();
			addTexConvexPoly(sidewalksTex[randSW],tmpPgonSideWalk.contour,QVector3D(0.6f,0.6f,0.6f),QVector3D(0,0,1.0f),0.0f,false,false,sidewalkTexScale[randSW]);
			for(int sN=0;sN<tmpPgonSideWalk.contour.size();sN++){
				int ind1=sN;
				int ind2=(sN+1)%tmpPgonSideWalk.contour.size();
				std::vector<QVector3D> em;
				addTexQuad(-1,
					tmpPgonSideWalk.contour[ind1]+QVector3D(0,0,-0.5f),
					tmpPgonSideWalk.contour[ind2]+QVector3D(0,0,-0.5f),
					tmpPgonSideWalk.contour[ind2]+QVector3D(0,0,0.0f),
					tmpPgonSideWalk.contour[ind1]+QVector3D(0,0,0.0f),
					em,
					QVector3D(0.5f,0.5f,0.5f),
					QVector3D(),
					true);
			}


			// ROAD
			addTexConvexPoly(asfalt,tmpPgonRoads.contour,QVector3D(0.8f,0.8f,0.8f),QVector3D(0,0,1.0f),-0.5f);
			*/
		}
		//printf(">> >> <<generateVBOOneBlock\n");
	}//

	void VBOBlocks::generateVBOBlocks(VBORenderManager& rendManager,std::vector< Block > &blocks){
		//printf(">> generateVBOBlocks \n");
		clearAll(rendManager);
		if(initialized==false){
			initVBOBlocks();
		}
		//printf(">> >> generateVBOOneBlock\n");
		//////////////////////
		// BLOCK GEOMETRY
		//allBlocks.resize(blocks.size());
		for(int bN=0;bN<blocks.size();bN++){
			//printf(">> >> bN %d\n",bN);
			//allBlocks[bN].generateOneBlockVBO(rendManager,blocks[bN]);
			//printf("generateVBOOneBlock %d\n",bN);
			generateVBOOneBlock(rendManager,blocks[bN]);
			//break;/// !!! REMOVE
		}
		////////////////////////////////////////
		// BUILDING
		QTime timer;
		timer.start();
		// 1. collect buildings
		//printf(">> >> generateBuilding\n");
		LC::Block::parcelGraphVertexIter vi, viEnd;
		for(int bN=0;bN<blocks.size();bN++){
			if(bN==0||bN==1)continue;
			for(boost::tie(vi, viEnd) = boost::vertices(blocks[bN].myParcels);
				vi != viEnd; ++vi)
			{
				//blocks[bN].myParcels[*vi].parcelContour
				VBOBuilding::generateBuilding(rendManager,blocks[bN].myParcels[*vi].myBuilding);//.buildingFootprint,blocks[bN].myParcels[*vi].parcelBuildingAttributes.att_stories);
			}
		}
		
		printf("<< generateVBOBlocks--> Building generation: %d ms\n",timer.elapsed());
	}//


}// namespace LC
