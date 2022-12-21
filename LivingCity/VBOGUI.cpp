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
*		@desc Class containing the GUI
*		@author igarciad
*
************************************************************************************************/

#include "VBOGUI.h"

#include "VBORenderManager.h"
#include "global.h"

namespace LC {

	VBOGUI::VBOGUI(){
		//selectedBoundingPolygonVertexIdx=0;
	}//

	VBOGUI::~VBOGUI(){
	}//

	void VBOGUI::clearVBOGUI(VBORenderManager& rendManager){
		std::set<float>::iterator it;
		for (it = pointSizes.begin(); it != pointSizes.end(); it++) {
			rendManager.removeStaticGeometry("GUI_"+QString::number(*it));
		}
		pointSizes.clear();
	}//


	void VBOGUI::addPoint(VBORenderManager& rendManager,QVector3D pos,QVector3D col,float size){
		QString name="GUI_"+QString::number(size);
		pointSizes.insert(size);
		std::vector<Vertex> points(1);
		points[0]=Vertex(pos.x(),pos.y(),pos.z()+10.0f,col.x(),col.y(),col.z(),0,0,0,0,0,0);//(&pos,new QVector3D(1.0f,0,0));//pos color
		rendManager.addStaticGeometry(name,points,"",GL_POINTS,1|LC::mode_AdaptTerrain);// texture not lighting
	}//


	void VBOGUI::updateVBOGUI(VBORenderManager& rendManager,std::vector<PlaceType>& myPlaceTypes,int placeTypeSelInd){
		
		clearVBOGUI(rendManager);

		for(int i=0; i < G::global().getInt("num_place_types"); ++i){	
			float pointSize;
			float lineWidth;
			QVector3D color=QVector3D(0,0,0);
			bool isSelectedPlaceType=i==placeTypeSelInd;
			if(!isSelectedPlaceType){
				pointSize = 12.0f;
				lineWidth = 3.0f;
			} else {
				pointSize = 24.0f;
				lineWidth = 3.0f;
			}

			QVector3D centerPt,handlerPt,handler2Pt,handlerDirUPt,handlerDirVPt;

			centerPt = myPlaceTypes[i].getQVector3D("pt_pt");
			handlerPt = myPlaceTypes[i].getExternalHandlerPos();
			handler2Pt = myPlaceTypes[i].getExternalHandler2Pos();
			handlerDirUPt = myPlaceTypes[i].getlengthUHandlerPos();
			handlerDirVPt = myPlaceTypes[i].getlengthVHandlerPos();

			//line segment from handler to center-------
			float lineDensity=6.0f;
			QVector3D pos;
			if(isSelectedPlaceType){
				//printf("C %f %f %f H %f %f %f\n",centerPt.x(),centerPt.y(),centerPt.z(),handlerPt.x(),handlerPt.y(),handlerPt.z());
				QVector3D dir=handlerPt-centerPt;
				float distance=dir.length();
				dir/=distance;
				for(float mV=0;mV<distance;mV=mV+lineDensity){
					pos=centerPt+mV*dir;
					//pos.setZ(pos.z()+0.0f);
					addPoint(rendManager,pos,QVector3D(0,0,0),4.0f);
				}
			}


			//center point------------
			addPoint(rendManager,centerPt,color,pointSize);
			addPoint(rendManager,centerPt,QVector3D(0.72, 0.87, 0.32),pointSize-4.0f);

			//paint rectangle and radius
			if(isSelectedPlaceType){
				lineDensity=12.0f;
				//
				for(int qN=0;qN<myPlaceTypes[i].boundingRectangle.size();qN++){
					int ind1=qN;
					int ind2=(qN+1)%myPlaceTypes[i].boundingRectangle.size();

					QVector3D dir=myPlaceTypes[i].boundingRectangle[ind2]-myPlaceTypes[i].boundingRectangle[ind1];
					float distance=dir.length();
					dir/=distance;
					for(float mV=0;mV<distance;mV=mV+lineDensity){
						pos=myPlaceTypes[i].boundingRectangle[ind1]+mV*dir;
						//pos.setZ(pos.z()+0.0f);
						addPoint(rendManager,pos,QVector3D(0.2f,0.2f,0.2f),2.5f);
					}
				}
				//external handler------------
				addPoint(rendManager,handlerPt,QVector3D(0.0f, 0.0f, 0.0f),pointSize);
				addPoint(rendManager,handlerPt,QVector3D(1.0f, 1.0f, 1.0f),pointSize-4.0f);
				//external handler 2------------
				addPoint(rendManager,handler2Pt,QVector3D(0.0f, 0.0f, 0.0f),pointSize);
				addPoint(rendManager,handler2Pt,QVector3D(1.0f, 1.0f, 1.0f),pointSize-4.0f);

				//road length handlers-------------

				// direction U
				addPoint(rendManager,handlerDirUPt,QVector3D(0.0f, 0.0f, 0.0f),pointSize-4.0f);
				addPoint(rendManager,handlerDirUPt,QVector3D(0.5f, 0.5f, 0.5f),pointSize-8.0f);

				// direction V
				addPoint(rendManager,handlerDirVPt,QVector3D(0.0f, 0.0f, 0.0f),pointSize-4.0f);
				addPoint(rendManager,handlerDirVPt,QVector3D(0.5f, 0.5f, 0.5f),pointSize-8.0f);
			}


		}//each placetype
	}//


	void VBOGUI::renderVBOGUI(VBORenderManager& rendManager){
		glDisable(GL_DEPTH_TEST);
		std::set<float>::reverse_iterator  rit;//note reverse iterator
		for (rit = pointSizes.rbegin(); rit != pointSizes.rend(); rit++) {
			float pont=*rit;
			glPointSize(pont);//after GUI_
			rendManager.renderStaticGeometry("GUI_"+QString::number(pont));
		}
		glEnable(GL_DEPTH_TEST);
	}//

}
