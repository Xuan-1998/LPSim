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
*		@desc Class containing the road graph
*		@author igarciad
*
************************************************************************************************/

#include "VBORoadGraph.h"

#include "VBORenderManager.h"

namespace LC {

	VBORoadGraph::VBORoadGraph(){
	}//

	VBORoadGraph::~VBORoadGraph(){
	}//

	void VBORoadGraph::clearVBORoadGraph(VBORenderManager& rendManager){
		rendManager.removeStaticGeometry("roads");
	}//


	void VBORoadGraph::updateRoadGraph(VBORenderManager& rendManager,RoadGraph &roadGraph){
		
		clearVBORoadGraph(rendManager);

		//////////////////////////////////////////
		// LINES
		if(G::global().getInt("render_mode")==1||G::global().getInt("render_mode")==2){//2D mode
			rendManager.removeStaticGeometry("roads_2D_segments5",false);
			rendManager.removeStaticGeometry("roads_2D_segments2",false);
			//////////////////////////////////////////////
			// EDGES AS LINES
			{
				std::vector<Vertex>* vert;
				std::vector<Vertex> vert2;
				std::vector<Vertex> vert5;
				RoadGraph::roadGraphEdgeIter_BI ei, eiEnd;
				int numEdges=0;
				QVector3D p0,p1,coV;
				//glColor3f(0.5, 0.8, 0.2);		
				//qsrand(4521654179);
				for(boost::tie(ei, eiEnd) = boost::edges(roadGraph.myRoadGraph_BI);
					ei != eiEnd; ++ei)
				{
					/////////////////////////////
					// Multi segment
					if(roadGraph.myRoadGraph_BI[*ei].roadSegmentGeometry.size()>0){

						if( roadGraph.myRoadGraph_BI[*ei].numberOfLanes>=G::global().getInt("cuda_arterial_numLanes"))
							vert=&vert2;
						else
							vert=&vert5;
						// MULTI SEGMENT
			
						float divSize=1.0f/(roadGraph.myRoadGraph_BI[*ei].roadSegmentGeometry.size()-1);
						for(int sN=0;sN<roadGraph.myRoadGraph_BI[*ei].roadSegmentGeometry.size()-1;sN++){//note -1
							float co=sN*divSize;
							coV=QVector3D(co, co, co);
							vert->push_back(Vertex(roadGraph.myRoadGraph_BI[*ei].roadSegmentGeometry[sN],coV,QVector3D(),QVector3D()));
							vert->push_back(Vertex(roadGraph.myRoadGraph_BI[*ei].roadSegmentGeometry[sN+1],coV,QVector3D(),QVector3D()));
						}
					}else{
						////////////////////////////
						// Normal edges
						numEdges++;
						if( roadGraph.myRoadGraph_BI[*ei].numberOfLanes>=G::global().getInt("cuda_arterial_numLanes"))
							vert=&vert5;
						else
							vert=&vert2;

						p0 = roadGraph.myRoadGraph_BI[boost::source(*ei, roadGraph.myRoadGraph_BI)].pt;
						p1 = roadGraph.myRoadGraph_BI[boost::target(*ei, roadGraph.myRoadGraph_BI)].pt;
						p0.setZ(2.0f);
						p1.setZ(2.0f);
						/// N
						QVector3D dir=(p1-p0).normalized();
						QVector3D per=(QVector3D::crossProduct(dir,QVector3D(0,0,1.0f)).normalized());
						// find twin
						int numLTwin=0;
						std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e_twi=
							boost::edge(boost::target(*ei, roadGraph.myRoadGraph_BI),boost::source(*ei, roadGraph.myRoadGraph_BI),roadGraph.myRoadGraph_BI);
						if(e_twi.second==true){
							numLTwin=roadGraph.myRoadGraph_BI[e_twi.first].numberOfLanes;
						}else{
							//printf("No twin\n");
						}
						///
						int numL=roadGraph.myRoadGraph_BI[*ei].numberOfLanes;

						for(int lN=0;lN<numL;lN++){
							if( roadGraph.myRoadGraph_BI[*ei].numberOfLanes>=G::global().getInt("cuda_arterial_numLanes"))
								coV=QVector3D(1.0f, 0.99f, 0.54f);//yellow //glColor3f(0.2f, 0.19f, 0.04f);//yellow
							else 
								coV=QVector3D(1.0f, 1.0f, 1.0f);//white	//glColor3f(0.7f, 0.7f, 0.7f);//white

							float perShift=G::global().getFloat("roadLaneWidth")*(lN+numLTwin-(numL+numLTwin-1)/2.0f);

							QVector3D p0S=QVector3D(p0.x()+per.x()*perShift, p0.y()+per.y()*perShift, p0.z());
							//vert->push_back(Vertex(p0S,coV,QVector3D(),QVector3D()));
							vert->push_back(Vertex(p0S, QVector3D(roadGraph.myRoadGraph_BI[*ei].outAngle / 360.0f, roadGraph.myRoadGraph_BI[*ei].outAngle / 360.0f, roadGraph.myRoadGraph_BI[*ei].outAngle / 360.0f), QVector3D(), QVector3D()));
							QVector3D p1S=QVector3D(p1.x()+per.x()*perShift, p1.y()+per.y()*perShift, p1.z());
							//vert->push_back(Vertex(p1S,coV,QVector3D(),QVector3D()));
							vert->push_back(Vertex(p1S, QVector3D(roadGraph.myRoadGraph_BI[*ei].inAngle / 360.0f, roadGraph.myRoadGraph_BI[*ei].inAngle / 360.0f, roadGraph.myRoadGraph_BI[*ei].inAngle / 360.0f), QVector3D(), QVector3D()));
						}
					}
				}
				rendManager.addStaticGeometry("roads_2D_segments2",vert2,"",GL_LINES, 1|LC::mode_AdaptTerrain);
				rendManager.addStaticGeometry("roads_2D_segments5",vert5,"",GL_LINES, 1|LC::mode_AdaptTerrain);
				//printf(" Num edges %d ",numEdges);
			}
			
			//////////////////////////////////////
			// EDGES
			{
				rendManager.removeStaticGeometry("road_2D_GM",false);
				//RoadEdgeIter ei, eend;
				std::vector<Vertex> vert;
				std::vector<Vertex> vertBg;

				QVector3D p0, p1, p2, p3;
				QVector3D p0Bg, p1Bg, p2Bg, p3Bg;

				RoadGraph::roadGraphEdgeIter ei, eiEnd;
				for(boost::tie(ei, eiEnd) = boost::edges(roadGraph.myRoadGraph);
					ei != eiEnd; ++ei)
				{
					// Type
					QColor color;// = graph[*ei]->color;
					QColor colorO;
					float heightOffset = 0.0f;
					float heightOffsetO=0.0f;
					int numL=roadGraph.myRoadGraph[*ei].numberOfLanes;
					float halfWidth=numL*G::global().getFloat("roadLaneWidth");

					switch (numL) {
					case 5:case 6:case 7:case 8:case 9:
					//case 4://RoadEdge::TYPE_HIGHWAY:
						heightOffset = 1.2f;
						heightOffsetO = 0.6f;
						color=QColor(0xfa,0x9e,0x25);
						colorO=QColor(0x00, 0x00, 0x00);//QColor(0xdf,0x9c,0x13);
						break;
					case 3://RoadEdge::TYPE_BOULEVARD:
					case 4://RoadEdge::TYPE_HIGHWAY:
						heightOffset = 1.0f;
						heightOffsetO = 0.4f;
						color=QColor(0xff,0xe1,0x68);
						colorO=QColor(0x00, 0x00, 0x00);//QColor(0xe5,0xbd,0x4d);
						break;
					case 2://RoadEdge::TYPE_AVENUE:
					case 1://RoadEdge::TYPE_STREET:
					default:
						heightOffset = 0.8f;
						heightOffsetO = 0.2f;
						color=QColor(0xff,0xff,0xff);
						colorO=QColor(0x00, 0x00, 0x00);//QColor(0xd7,0xd1,0xc7);
						break;
					}

					//halfWidth+= G::global().getFloat("2DroadsExtraWidth");
					heightOffset+=0.45f;//to have park below
					heightOffsetO+=0.45f;//to have park below

					heightOffset*=6.0f;//z fighting
					heightOffsetO*=6.0f;

					float halfWidthBg = halfWidth+4.5f;// + G::global().getFloat("2DroadsStroke");//it should not depend on the type 3.5f

					QVector3D pt1 = roadGraph.myRoadGraph[boost::source(*ei, roadGraph.myRoadGraph)].pt;
					QVector3D pt2 = roadGraph.myRoadGraph[boost::target(*ei, roadGraph.myRoadGraph)].pt;

					QVector3D perp = pt2 - pt1;
					perp = QVector3D(-perp.y(), perp.x(), 0.0f);
					perp.normalize();


					p0 = pt1 + perp * halfWidth;
					p1 = pt1 - perp * halfWidth;
					p0Bg = pt1 + perp * halfWidthBg;
					p1Bg = pt1 - perp * halfWidthBg;

					p2 = pt2 - perp * halfWidth;
					p3 = pt2 + perp * halfWidth;
					p2Bg = pt2 - perp * halfWidthBg;
					p3Bg = pt2 + perp * halfWidthBg;
					QVector3D normal = QVector3D(0,0,1);//Util::calculateNormal(p0, p1, p2);


					vert.push_back(Vertex(p0.x(),p0.y(),p0.z()+heightOffset,color.redF(),color.greenF(),color.blueF(),0,0,1.0f,0,0,0));// pos color normal texture
					vert.push_back(Vertex(p1.x(),p1.y(),p1.z()+heightOffset,color.redF(),color.greenF(),color.blueF(),0,0,1.0f,0,0,0));// pos color normal texture
					vert.push_back(Vertex(p2.x(),p2.y(),p2.z()+heightOffset,color.redF(),color.greenF(),color.blueF(),0,0,1.0f,0,0,0));// pos color normal texture
					vert.push_back(Vertex(p3.x(),p3.y(),p3.z()+heightOffset,color.redF(),color.greenF(),color.blueF(),0,0,1.0f,0,0,0));// pos color normal texture

					vertBg.push_back(Vertex(p0Bg.x(),p0Bg.y(),p0Bg.z()+heightOffsetO,colorO.redF(),colorO.greenF(),colorO.blueF(),0,0,1.0f,0,0,0));// pos color normal texture
					vertBg.push_back(Vertex(p1Bg.x(),p1Bg.y(),p1Bg.z()+heightOffsetO,colorO.redF(),colorO.greenF(),colorO.blueF(),0,0,1.0f,0,0,0));// pos color normal texture
					vertBg.push_back(Vertex(p2Bg.x(),p2Bg.y(),p2Bg.z()+heightOffsetO,colorO.redF(),colorO.greenF(),colorO.blueF(),0,0,1.0f,0,0,0));// pos color normal texture
					vertBg.push_back(Vertex(p3Bg.x(),p3Bg.y(),p3Bg.z()+heightOffsetO,colorO.redF(),colorO.greenF(),colorO.blueF(),0,0,1.0f,0,0,0));// pos color normal texture
				}

				rendManager.addStaticGeometry("road_2D_GM", vert, "", GL_QUADS, 1);//MODE=1 color
				rendManager.addStaticGeometry("road_2D_GM", vertBg, "", GL_QUADS, 1);//MODE=1 color

			}
			
			/////////////////////////////////////////////////////
			// INTERSECTIONS
			{
				rendManager.removeStaticGeometry("road_2D_GM_int",false);

				RoadGraph::roadGraphVertexIter vi, vend;
				for (boost::tie(vi, vend) = boost::vertices(roadGraph.myRoadGraph); vi != vend; ++vi) {
					// get the largest width of the outing edges
					QColor color;// = graph[*ei]->color;
					QColor colorO;
					float heightOffset = 0.0f;
					float heightOffsetO=0.0f;
					bool render = false;
					int numL=-1;
					float halfWidth;

					RoadGraph::out_roadGraphEdgeIter oei, oeend;;
					//RoadOutEdgeIter oei, oeend;
					for (boost::tie(oei, oeend) = boost::out_edges(*vi, roadGraph.myRoadGraph); oei != oeend; ++oei) {
						
						//printf("type %d\n",roadGraph.myRoadGraph[*oei].numberOfLanes);
						if(numL<=roadGraph.myRoadGraph[*oei].numberOfLanes)
							continue;
						numL=roadGraph.myRoadGraph[*oei].numberOfLanes;
						halfWidth=numL*G::global().getFloat("roadLaneWidth");

						switch (numL) {
						case 5:case 6:case 7:case 8:case 9:
								heightOffset = 1.2f;
								heightOffsetO = 0.6f;
								color=QColor(0xfa,0x9e,0x25);
								colorO=QColor(0x00, 0x00, 0x00);//QColor(0xdf,0x9c,0x13);
							continue;
		
						case 3://RoadEdge::TYPE_BOULEVARD:
						case 4://RoadEdge::TYPE_HIGHWAY:
								heightOffset = 1.0f;
								heightOffsetO = 0.4f;
								color=QColor(0xff,0xe1,0x68);
								colorO=QColor(0x00, 0x00, 0x00);//QColor(0xe5,0xbd,0x4d);
								continue;
						case 2://RoadEdge::TYPE_AVENUE:
						case 1://RoadEdge::TYPE_STREET:
						default:
								heightOffset = 0.8f;
								heightOffsetO = 0.2f;
								color=QColor(0xff,0xff,0xff);
								colorO=QColor(0x00, 0x00, 0x00);//QColor(0xd7,0xd1,0xc7);
								continue;
						}
					}
					//printf("** numL %d\n",numL);
					//halfWidth+= G::global().getFloat("2DroadsStroke");//it should not depend on the type 3.5f
					heightOffset+=0.45f;//to have park below
					heightOffsetO+=0.45f;//to have park below

					heightOffset*=6.0f;//z fighting
					heightOffsetO*=6.0f;

					float max_r=halfWidth;
					float max_rO=halfWidth+4.5f;//+ G::global().getFloat("2DroadsStroke");//it should not depend on the type 3.5f

					std::vector<Vertex> vert(3*20);
					std::vector<Vertex> vertBg(3*20);

					for (int i = 0; i < 20; ++i) {
						float angle1 = 2.0 * M_PI * i / 20.0f;
						float angle2 = 2.0 * M_PI * (i + 1) / 20.0f;

						vert[i*3+0]=Vertex(roadGraph.myRoadGraph[*vi].pt.x(), roadGraph.myRoadGraph[*vi].pt.y(), roadGraph.myRoadGraph[*vi].pt.z() + heightOffset, color.redF(), color.greenF(), color.blueF(), 0, 0, 1.0f, 0, 0, 0);
						vert[i*3+1]=Vertex(roadGraph.myRoadGraph[*vi].pt.x() + max_r * cosf(angle1), roadGraph.myRoadGraph[*vi].pt.y() + max_r * sinf(angle1), roadGraph.myRoadGraph[*vi].pt.z() + heightOffset, color.redF(), color.greenF(), color.blueF(), 0, 0, 1.0f, 0, 0, 0);
						vert[i*3+2]=Vertex(roadGraph.myRoadGraph[*vi].pt.x() + max_r * cosf(angle2), roadGraph.myRoadGraph[*vi].pt.y() + max_r * sinf(angle2), roadGraph.myRoadGraph[*vi].pt.z() + heightOffset, color.redF(), color.greenF(), color.blueF(), 0, 0, 1.0f, 0, 0, 0);

						vertBg[i*3+0]=Vertex(roadGraph.myRoadGraph[*vi].pt.x(), roadGraph.myRoadGraph[*vi].pt.y(), roadGraph.myRoadGraph[*vi].pt.z() + heightOffsetO, colorO.redF(), colorO.greenF(), colorO.blueF(), 0, 0, 1.0f, 0, 0, 0);
						vertBg[i*3+1]=Vertex(roadGraph.myRoadGraph[*vi].pt.x() + max_rO * cosf(angle1), roadGraph.myRoadGraph[*vi].pt.y() + max_rO * sinf(angle1), roadGraph.myRoadGraph[*vi].pt.z() + heightOffsetO, colorO.redF(), colorO.greenF(), colorO.blueF(), 0, 0, 1.0f, 0, 0, 0);
						vertBg[i*3+2]=Vertex(roadGraph.myRoadGraph[*vi].pt.x() + max_rO * cosf(angle2), roadGraph.myRoadGraph[*vi].pt.y() + max_rO * sinf(angle2), roadGraph.myRoadGraph[*vi].pt.z() + heightOffsetO, colorO.redF(), colorO.greenF(), colorO.blueF(), 0, 0, 1.0f, 0, 0, 0);
					}
					rendManager.addStaticGeometry("road_2D_GM_int", vert, "", GL_TRIANGLES, 1);//MODE=1 color
					rendManager.addStaticGeometry("road_2D_GM_int", vertBg, "", GL_TRIANGLES, 1);//MODE=1 color
				}
			}

		}//end 2D

		/*for(boost::tie(ei, eiEnd) = boost::edges(roadGraph.myRoadGraph);
			ei != eiEnd; ++ei)
		{
			numEdges++;
			QVector3D col;
			if( roadGraph.myRoadGraph[*ei].numberOfLanes==LC::misctools::Global::global()->cuda_arterial_numLanes){//.isArterial() ){
				glLineWidth(5.0f);
				col=QVector3D(1.0f, 0.99f, 0.54f);//yellow
			} else {
				glLineWidth(2.0f);
				col=QVector3D(0.9f, 0.9f, 0.9f);//white	
			}


			p0 = roadGraph.myRoadGraph[boost::source(*ei, roadGraph.myRoadGraph)].pt;
			p1 = roadGraph.myRoadGraph[boost::target(*ei, roadGraph.myRoadGraph)].pt;
			p0.setZ(1.0f);
			p1.setZ(1.0f);
			
			if(true){
				QVector3D dir=p1-p0;
				float length=(dir).length();
				dir/=length;
				int numSegments=ceil(length/5.0f);
				if(numSegments<2){
					line_Vert.push_back(Vertex(p0.x(),p0.y(),p0.z(),col.x(),col.y(),col.z(),0,0,0,0,0,0));
					line_Vert.push_back(Vertex(p1.x(),p1.y(),p1.z(),col.x(),col.y(),col.z(),0,0,0,0,0,0));
				}else{
					QVector3D p0_,p1_;
					p0_=p0;
					for(int sN=0;sN<numSegments;sN++){
						float lenghToMove=5.0f;
						if(length<10)
							lenghToMove=length;
						p1_=p0_+dir*lenghToMove;
						length-=lenghToMove;
						line_Vert.push_back(Vertex(p0_.x(),p0_.y(),p0_.z(),col.x(),col.y(),col.z(),0,0,0,0,0,0));
						line_Vert.push_back(Vertex(p1_.x(),p1_.y(),p1_.z(),col.x(),col.y(),col.z(),0,0,0,0,0,0));
						p0_=p1_;
					}

				}
			}else{
				//RendUtil::addLine(line_Vert,p0,p1,col);
				line_Vert.push_back(Vertex(p0.x(),p0.y(),p0.z(),col.x(),col.y(),col.z(),0,0,0,0,0,0));
				line_Vert.push_back(Vertex(p1.x(),p1.y(),p1.z(),col.x(),col.y(),col.z(),0,0,0,0,0,0));
			}
			
		}*/

		//////////////////////////////////////////
		// 3D LINES QUADS
		if(G::global().getInt("render_mode")==0){//3D
			const int renderRoadType=1;
			const float deltaZ=5.0f;
			const float maxSegmentLeng=5.0f;
			const float roadWidth=G::global().getFloat("roadLaneWidth");
			
			// Roads with lines (not median strip)
			if(renderRoadType==0||renderRoadType==1){
				//const float texWidthArr[]={0,0.134252f,0.28125f,0.41964f,0.5578f,0.7058f,0.8529f,1.0f};
				RoadGraph::roadGraphEdgeIter ei, eiEnd;
				QVector3D p0,p1;
				QVector3D a0,a1,a2,a3;
				int numL;
				int numEdges=0;
				std::vector<Vertex> vertROAD;
				
				float dW,length,texWidth;
				for(boost::tie(ei, eiEnd) = boost::edges(roadGraph.myRoadGraph);ei != eiEnd; ++ei){
				
					numEdges++;
					numL=roadGraph.myRoadGraph[*ei].numberOfLanes;
					//texWidth=texWidthArr[std::min<int>(numL,7)];
					p0 = roadGraph.myRoadGraph[boost::source(*ei, roadGraph.myRoadGraph)].pt;
					p1 = roadGraph.myRoadGraph[boost::target(*ei, roadGraph.myRoadGraph)].pt;
					p0.setZ(deltaZ);
					p1.setZ(deltaZ);

					dW=(numL*roadWidth)/2.0f;

					QVector3D dir=(p1-p0);//.normalized();
					length=dir.length();
					dir/=length;
					QVector3D per=(QVector3D::crossProduct(dir,QVector3D(0,0,1.0f)).normalized());

					a1=QVector3D(p0.x()+per.x()*dW, p0.y()+per.y()*dW, p0.z());
					a2=QVector3D(p0.x()-per.x()*dW, p0.y()-per.y()*dW, p0.z());

					int numSegments=ceil(length/5.0f);
					float lengthMoved=0;
					
					for(int nS=0;nS<numSegments;nS++){
						float segmentLeng=std::min(maxSegmentLeng,length);
						a0=a1;
						a3=a2;
						a1=a1+dir*(segmentLeng);
						a2=a2+dir*(segmentLeng);
						vertROAD.push_back(Vertex(a0,QVector3D(),QVector3D(0,0,1.0f),QVector3D(0.106f,lengthMoved/dW,0.0f)));
						vertROAD.push_back(Vertex(a1,QVector3D(),QVector3D(0,0,1.0f),QVector3D(0.106f,(segmentLeng+lengthMoved)/dW,0)));
						vertROAD.push_back(Vertex(a2,QVector3D(),QVector3D(0,0,1.0f),QVector3D(numL,(segmentLeng+lengthMoved)/dW,0)));
						vertROAD.push_back(Vertex(a3,QVector3D(),QVector3D(0,0,1.0f),QVector3D(numL,lengthMoved/dW,0)));
						lengthMoved+=segmentLeng;
						length-=segmentLeng;
					}


				}
				rendManager.addStaticGeometry("roads",vertROAD,"data/textures/roads/road_1lines.jpg",GL_QUADS,2|LC::mode_AdaptTerrain);
			}
			//////////////////////////////////////////
			// 3D INTERSECTIONS
			if(renderRoadType==1){
				// INTERSECTIONS
				std::vector<Vertex> intersectCirclesV;
				RoadGraph::roadGraphVertexIter vi, vend;
				for (boost::tie(vi, vend) = boost::vertices(roadGraph.myRoadGraph); vi != vend; ++vi) {

					// get the largest width of the outing edges
					float max_numL = 0;
					
					float offset = 0.3f;
					RoadGraph::out_roadGraphEdgeIter oei, oeend;
					int numO=0;
					for (boost::tie(oei, oeend) = boost::out_edges(*vi,roadGraph.myRoadGraph); oei != oeend; ++oei) {

						float numL = roadGraph.myRoadGraph[*oei].numberOfLanes;
							if (numL > max_numL) {
								max_numL = numL;
							}
						numO++;
					}
					QVector3D center=roadGraph.myRoadGraph[*vi].pt;
					if(numO<=2)
						center.setZ(deltaZ-0.2f);//below
					else
						center.setZ(deltaZ+0.2f);//above

					float radi1 = roadWidth*max_numL /2.0f;
					//if(numO==2)radi1*=1.10f;
					//if(numO>3)radi1*=1.2f;

					const float numSides=20;
					const float deltaAngle=( 1.0f / numSides ) * 3.14159f * 2.0f;
					float angle=0.0f;
					QVector3D nP,oP;
					oP=QVector3D( radi1 * cos( angle ), radi1 * sin( angle ),0.0f );//init point
					for(int i=0;i<numSides+1;i++){
						angle=deltaAngle*i;
						nP=QVector3D( radi1 * cos( angle ), radi1 * sin( angle ),0.0f );

						intersectCirclesV.push_back(Vertex(center,center/7.5f));
						intersectCirclesV.push_back(Vertex(center+oP,(center+oP)/7.5f));
						intersectCirclesV.push_back(Vertex(center+nP,(center+nP)/7.5f));
						oP=nP;
					}


				}//all vertex
				rendManager.addStaticGeometry("roads",intersectCirclesV,"data/textures/roads/road_0lines.jpg",GL_TRIANGLES,2|mode_AdaptTerrain);

			}
			/*if(renderRoadType==1){
				std::vector<Vertex> intersectCirclesV;

				RoadGraph::roadGraphVertexIter vi, vend;
				for (boost::tie(vi, vend) = boost::vertices(roadGraph.myRoadGraph); vi != vend; ++vi) {

					int outDegree=boost::out_degree(*vi,roadGraph.myRoadGraph);
					////////////////////////
					// 2.1 JUST TWO OR LESS--> CIRCLE BELOW
					if(outDegree<=2){
						// get the largest width of the outing edges
						float max_numL = 0;
						int max_roadType = 0;
						float offset = 0.3f;
						RoadGraph::out_roadGraphEdgeIter oei, oeend;
						for (boost::tie(oei, oeend) = boost::out_edges(*vi, roadGraph.myRoadGraph); oei != oeend; ++oei) {
					

							float numL = roadGraph.myRoadGraph[*oei].numberOfLanes;
							if (numL > max_numL) {
								max_numL = numL;
							}
						}
						QVector3D center=roadGraph.myRoadGraph[*vi].pt;
						center.setZ(deltaZ-0.1f);//below

						float radi1 = roadWidth*max_numL /2.0f;
						if(outDegree==2)radi1*=1.10f;

						const float numSides=20;
						const float deltaAngle=( 1.0f / numSides ) * 3.14159f * 2.0f;
						float angle=0.0f;
						QVector3D nP,oP;
						oP=QVector3D( radi1 * cos( angle ), radi1 * sin( angle ),0.0f );//init point
						for(int i=0;i<numSides+1;i++){
							angle=deltaAngle*i;
							nP=QVector3D( radi1 * cos( angle ), radi1 * sin( angle ),0.0f );

							intersectCirclesV.push_back(Vertex(center,center/7.5f));
							intersectCirclesV.push_back(Vertex(center+oP,(center+oP)/7.5f));
							intersectCirclesV.push_back(Vertex(center+nP,(center+nP)/7.5f));
							oP=nP;
						}

					}else{
						////////////////////////
						// 2.2 TWO OR MORE--> COMPLEX INTERSECTION


						////////////
						// 2.2.1 For each vertex find edges and short by angle
						QVector2D referenceVector(0,1);
						QVector2D p0,p1;
						std::vector<std::pair<std::pair<QVector3D,QVector2D>,float>> edgeAngleOut;
						int numOutEdges=0;
						RoadGraph::out_roadGraphEdgeIter Oei, Oei_end;
						float angleRef=atan2(referenceVector.y(),referenceVector.x());
						for(boost::tie(Oei, Oei_end) = boost::out_edges(*vi,roadGraph.myRoadGraph); Oei != Oei_end; ++Oei){
							// find first segment 
							RoadEdgePtr edge = roadGraph.graph[*Oei];
							p0=edge->polyline[0];

							if(p0==roadGraph.graph[*vi]->pt){
								p1=edge->polyline[1];
							}else{
								p0=edge->polyline.back();
								p1=edge->polyline[edge->polyline.size()-2];
							}

							QVector2D edgeDir=(p1-p0).normalized();// NOTE p1-p0
							p1=p0+edgeDir*30.0f;//expand edge to make sure it is long enough

							float angle=angleRef-atan2(edgeDir.y(),edgeDir.x());
							float width=edge->getWidth();
							edgeAngleOut.push_back(std::make_pair(std::make_pair(QVector3D(p0.x(),p0.y(),width),p1),angle));//z as width
							numOutEdges++;
						}
						if(edgeAngleOut.size()>0){
							std::sort( edgeAngleOut.begin(), edgeAngleOut.end(), compare2ndPartTuple);
						}
						// 2.2.2 Create intersection geometry of the given edges
						QVector3D ed1p0,ed1p1,ed2p0,ed2p1;
						QVector3D ed1Per,ed2Per;
						float ed1W,ed2W;
						std::vector<QVector3D> interPoints;
						for(int eN=0;eN<edgeAngleOut.size();eN++){
							// a) ED1: this edge
							ed1W=edgeAngleOut[eN].first.first.z();//use z as width
							ed1p0=edgeAngleOut[eN].first.first;
							ed1p0.setZ(deltaZ);
							ed1p1=edgeAngleOut[eN].first.second.toVector3D();
							// compute right side
							QVector3D ed1Dir=(ed1p0-ed1p1).normalized();//ends in 0
							ed1Per=(QVector3D::crossProduct(ed1Dir,QVector3D(0,0,1.0f)).normalized());//need normalized()?
							ed1p0+=ed1Per*ed1W/2.0f;
							ed1p1+=ed1Per*ed1W/2.0f;

							// b) ED2: next edge
							int nextEdge=(eN+1)%edgeAngleOut.size();
							ed2W=edgeAngleOut[nextEdge].first.first.z();//use z as width
							ed2p0=edgeAngleOut[nextEdge].first.first;
							ed2p0.setZ(0);
							ed2p1=edgeAngleOut[nextEdge].first.second.toVector3D();
							// compute left side
							QVector3D ed2Dir=(ed2p0-ed2p1).normalized();//ends in 0
							ed2Per=(QVector3D::crossProduct(ed2Dir,QVector3D(0,0,1.0f)).normalized());//need normalized()?
							ed2p0-=ed2Per*ed2W/2.0f;
							ed2p1-=ed2Per*ed2W/2.0f;

							// c) computer intersection
							float tab,tcd;
							QVector3D intPoint;
							printf("ED1 %f %f --> %f %f  ED2 %f %f --> %f %f\n",ed1p0.x(),ed1p0.y(),ed1p1.x(),ed1p1.y(), ed2p0.x(),ed2p0.y(),ed2p1.x(),ed2p1.y());
							if(Util::segmentSegmentIntersectXY3D(ed1p0,ed1p1,ed2p0,ed2p1,&tab,&tcd, false,intPoint)==false&&false){
								printf("ERROR: Parallel!!!\n");
							}else{
								printf("Int %f %f\n",intPoint.x(),intPoint.y());
								printf("ADD: No Parallel!!!\n");
								interPoints.push_back(intPoint);
							}
						}
						if(interPoints.size()>2)
							//rendManager.addStaticGeometry2("3d_roads_interCom",interPoints,1.1f,false,"",GL_QUADS,1|mode_AdaptTerrain,QVector3D(1,1,9),QVector3D());
							rendManager.addStaticGeometry2("3d_roads_interCom",interPoints,1.1f,false,"../data/textures/roads/road_0lines.jpg",GL_QUADS,2|mode_AdaptTerrain,QVector3D(1.0f/7.5f,1.0f/7.5f,1),QVector3D());
						//////////////

					}

				}//all vertex
				rendManager.addStaticGeometry("3d_roads_inter",intersectCirclesV,"../data/textures/roads/road_0lines.jpg",GL_TRIANGLES,2|mode_AdaptTerrain);
			}*/

			/*RoadGraph::roadGraphEdgeIter ei, eiEnd;
			QVector3D p0,p1;
			int numEdges=0;

			std::vector<Vertex> vertROAD;
			std::vector<Vertex> vertINT;
			QVector3D a0,a1,a2,a3;
			float width=3.5f;
			for(boost::tie(ei, eiEnd) = boost::edges(roadGraph.myRoadGraph);
				ei != eiEnd; ++ei)
			{
				numEdges++;
				QVector3D col;
				if( roadGraph.myRoadGraph[*ei].numberOfLanes==G::global().getInt("cuda_arterial_numLanes")){//.isArterial() ){
					glLineWidth(5.0f);
					col=QVector3D(1.0f, 0.99f, 0.54f);//yellow
				} else {
					glLineWidth(2.0f);
					col=QVector3D(0.9f, 0.9f, 0.9f);//white	
				}


				p0 = roadGraph.myRoadGraph[boost::source(*ei, roadGraph.myRoadGraph)].pt;
				p1 = roadGraph.myRoadGraph[boost::target(*ei, roadGraph.myRoadGraph)].pt;
				p0.setZ(1.0f);
				p1.setZ(1.0f);


				QVector3D dir=(p1-p0);//.normalized();
				float length=dir.length();
				dir/=length;
				QVector3D per=(QVector3D::crossProduct(dir,QVector3D(0,0,1.0f)).normalized());

				int numSeg=0;
				const float intersectionClearance=3.5f;

				/// no room for one line
				if(length<2.0f*intersectionClearance){//no real line
					a0=QVector3D(p0.x()+per.x()*width, p0.y()+per.y()*width, p0.z());
					a1=QVector3D(p1.x()+per.x()*width, p1.y()+per.y()*width, p0.z());

					a2=QVector3D(p1.x()-per.x()*width, p1.y()-per.y()*width, p0.z());
					a3=QVector3D(p0.x()-per.x()*width, p0.y()-per.y()*width, p0.z());

					vertINT.push_back(Vertex(a0,QVector3D(),QVector3D(0,0,1.0f),a0));
					vertINT.push_back(Vertex(a1,QVector3D(),QVector3D(0,0,1.0f),a1));
					vertINT.push_back(Vertex(a2,QVector3D(),QVector3D(0,0,1.0f),a2));
					vertINT.push_back(Vertex(a3,QVector3D(),QVector3D(0,0,1.0f),a3));
				}else{
					QVector3D p0_,p1_;
					// START
					p0_=p0;
					p1_=p0+dir*intersectionClearance;
					a0=QVector3D(p0_.x()+per.x()*width, p0_.y()+per.y()*width, p0_.z());
					a1=QVector3D(p1_.x()+per.x()*width, p1_.y()+per.y()*width, p0_.z());

					a2=QVector3D(p1_.x()-per.x()*width, p1_.y()-per.y()*width, p0_.z());
					a3=QVector3D(p0_.x()-per.x()*width, p0_.y()-per.y()*width, p0_.z());

					vertINT.push_back(Vertex(a0,QVector3D(),QVector3D(0,0,1.0f),a0));
					vertINT.push_back(Vertex(a1,QVector3D(),QVector3D(0,0,1.0f),a1));
					vertINT.push_back(Vertex(a2,QVector3D(),QVector3D(0,0,1.0f),a2));
					vertINT.push_back(Vertex(a3,QVector3D(),QVector3D(0,0,1.0f),a3));

					// MIDDLE

					float middLenght=length-2.0f*intersectionClearance;
					float const maxSegmentLeng=5.0f;
					float segmentLeng;
					if(middLenght>2*maxSegmentLeng){
						int numSegments=ceil(length/5.0f);
						float lengthMoved=0;
						float dW=(2*width);
						for(int nS=0;nS<numSegments;nS++){
							segmentLeng=std::min(maxSegmentLeng,middLenght);
							a0=a1;
							a3=a2;
							a1=a1+dir*(segmentLeng);
							a2=a2+dir*(segmentLeng);
							vertROAD.push_back(Vertex(a0,QVector3D(),QVector3D(0,0,1.0f),QVector3D(0,lengthMoved/dW,0.0f)));
							vertROAD.push_back(Vertex(a1,QVector3D(),QVector3D(0,0,1.0f),QVector3D(0,(segmentLeng+lengthMoved)/dW,0)));
							vertROAD.push_back(Vertex(a2,QVector3D(),QVector3D(0,0,1.0f),QVector3D(1.0f,(segmentLeng+lengthMoved)/dW,0)));
							vertROAD.push_back(Vertex(a3,QVector3D(),QVector3D(0,0,1.0f),QVector3D(1.0f,lengthMoved/dW,0)));
							lengthMoved+=segmentLeng;
							middLenght-=segmentLeng;
						}

					}else{
						// JUST ONE MIDDLE SEGMENT
						a0=a1;
						a3=a2;
						a1=a1+dir*(middLenght);
						a2=a2+dir*(middLenght);

						vertROAD.push_back(Vertex(a0,QVector3D(),QVector3D(0,0,1.0f),QVector3D(0,0.0f,0)));
						vertROAD.push_back(Vertex(a1,QVector3D(),QVector3D(0,0,1.0f),QVector3D(0,middLenght/(2*width),0)));
						vertROAD.push_back(Vertex(a2,QVector3D(),QVector3D(0,0,1.0f),QVector3D(1.0f,middLenght/(2*width),0)));
						vertROAD.push_back(Vertex(a3,QVector3D(),QVector3D(0,0,1.0f),QVector3D(1.0f,0.0f,0)));
					}
					// END
					a0=a1;
					a3=a2;

					a1=a1+dir*intersectionClearance;
					a2=a2+dir*intersectionClearance;

					vertINT.push_back(Vertex(a0,QVector3D(),QVector3D(0,0,1.0f),a0));
					vertINT.push_back(Vertex(a1,QVector3D(),QVector3D(0,0,1.0f),a1));
					vertINT.push_back(Vertex(a2,QVector3D(),QVector3D(0,0,1.0f),a2));
					vertINT.push_back(Vertex(a3,QVector3D(),QVector3D(0,0,1.0f),a3));

				}
			}

			rendManager.addStaticGeometry("roads",vertINT,"data/textures/street_1.png",GL_QUADS, 2|LC::mode_AdaptTerrain);
			rendManager.addStaticGeometry("roads",vertROAD,"data/textures/street_0.png",GL_QUADS,2|LC::mode_AdaptTerrain);*/
		}
	}//


}
