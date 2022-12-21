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

#include "VBORoadLabels.h"


#include "VBORenderManager.h"
#include "VBOText.h"
#include "VBORoadLabels.h"

namespace LC {

	//std::vector<TextSt> VBORoadLabels::textSt;

	void VBORoadLabels::updateRoadLabels(VBORenderManager& rendManager,RoadGraph &roadGraph){

		printf("updateRoadLabels\n");

		rendManager.removeStaticGeometry("road_labels");
		
		
		const float deltaZ=10.0f;
		const float roadWidth=G::global().getFloat("roadLaneWidth");
		
		RoadGraph::roadGraphEdgeIter ei, eiEnd;
		QVector3D p0,p1;
		QVector3D a0,a1,a2,a3;
		int numL;
		int numEdges=0;
		std::vector<Vertex> vertROAD;
		
		float dW,length,texWidth;
		QString label;
		for(boost::tie(ei, eiEnd) = boost::edges(roadGraph.myRoadGraph);ei != eiEnd; ++ei){

			numEdges++;
			numL=roadGraph.myRoadGraph[*ei].numberOfLanes;
			label=roadGraph.myRoadGraph[*ei].label;
			if(label.length()<3)//skip short strings
				continue;
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

			/*a0=QVector3D(p0.x()+per.x()*dW, p0.y()+per.y()*dW, p0.z());
			a3=QVector3D(p0.x()-per.x()*dW, p0.y()-per.y()*dW, p0.z());

			a1=a0+dir*length;
			a2=a3+dir*length;*/

			//VBOText::addText(rendManager,"road_labels",label.toLower(),p0,dir,per,dW*2.0f);
			//VBOText::addText(rendManager,"road_labels",label.toLower(),p0,dir,-per,dW*2.0f);//dW*2.0f);

			VBOText::addText(rendManager,"road_labels",label.toLower(),(p0+p1)/2.0f,dir,-per,dW*2.0f,true);//dW*2.0f);

			//return;
			//printf("label %s p0 dir %f %f %f per %f %f %f\n",label.toUtf8().constData(),p0.x(),p0.y(),p0.z(),dir.x(),dir.y(),dir.z(),per.x(),per.y(),per.z());
		}
		

	}//

	void VBORoadLabels::renderRoadLabels(VBORenderManager& rendManager,int types){
		/*for(int tN=0;tN<textSt.size();tN++){
			TextSt& t=textSt[tN];
			VBOText::renderText(rendManager,t.text,t.pt,t.dir,t.dirUp,t.height);
		}*/
		glEnable(GL_BLEND);
		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glBlendEquation(GL_FUNC_ADD);
		rendManager.renderStaticGeometry("road_labels");
		glDisable(GL_BLEND);
	}//

} // namespace LC
