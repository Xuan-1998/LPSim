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

#include "VBOSkyBox.h"
#include <QFileInfo>
#include "VBOUtil.h"

#include "VBORenderManager.h"

namespace LC {

	VBOSkyBox::VBOSkyBox() {
		//initialized=0;
	}

	VBOSkyBox::~VBOSkyBox() {
	}

	void VBOSkyBox::init(VBORenderManager& rendManager){

		rendManager.removeStaticGeometry(QString("sky"));//in case of skychange

		float scale=rendManager.side*0.6f;//5000.0f;//0.6 bigger than half
		QString imgName="data/sky/skyCombined.png";
		VBOUtil::check_gl_error("Init SkyboxVBO");

		float r = 1.0005f; // If you have border issues change this to 1.005f
		// Common Axis X - LEFT side
		std::vector<Vertex> sky_Vert;
		sky_Vert.push_back(Vertex(scale*QVector3D(-1.0f,-r,-r),QVector3D(0.0f,2/3.0f,0)));	
		sky_Vert.push_back(Vertex(scale*QVector3D(-1.0f, r,-r),QVector3D(0.5f,2/3.0f,0)));	
		sky_Vert.push_back(Vertex(scale*QVector3D(-1.0f, r, r),QVector3D(0.5f,1.0f,0)));	
		sky_Vert.push_back(Vertex(scale*QVector3D(-1.0f,-r, r),QVector3D(0.0f,1.0f,0)));	

		// Common Axis Z - FRONT side
	
		sky_Vert.push_back(Vertex(scale*QVector3D(-r,1.0f,-r),QVector3D(0.5f,2/3.0f,0)));	
		sky_Vert.push_back(Vertex(scale*QVector3D( r,1.0f,-r),QVector3D(1.0f,2/3.0f,0))); 
		sky_Vert.push_back(Vertex(scale*QVector3D( r,1.0f, r),QVector3D(1.0f,1.0f,0)));
		sky_Vert.push_back(Vertex(scale*QVector3D(-r,1.0f, r),QVector3D(0.5f,1.0f,0)));

		// Common Axis X - Right side

		sky_Vert.push_back(Vertex(scale*QVector3D(1.0f, r,-r),QVector3D(0.0f,1/3.0f,0)));	
		sky_Vert.push_back(Vertex(scale*QVector3D(1.0f,-r,-r),QVector3D(0.5f,1/3.0f,0))); 
		sky_Vert.push_back(Vertex(scale*QVector3D(1.0f,-r, r),QVector3D(0.5f,2/3.0f,0)));
		sky_Vert.push_back(Vertex(scale*QVector3D(1.0f, r, r),QVector3D(0.0f,2/3.0f,0)));

		// Common Axis Z - BACK side
	
		sky_Vert.push_back(Vertex(scale*QVector3D( r,-1.0f,-r),QVector3D(0.5f,1/3.0f,0)));	
		sky_Vert.push_back(Vertex(scale*QVector3D(-r,-1.0f,-r),QVector3D(1.0f,1/3.0f,0))); 
		sky_Vert.push_back(Vertex(scale*QVector3D(-r,-1.0f, r),QVector3D(1.0f,2/3.0f,0)));
		sky_Vert.push_back(Vertex(scale*QVector3D( r,-1.0f, r),QVector3D(0.5f,2/3.0f,0)));

		// Common Axis Y - Draw Up side
	
		sky_Vert.push_back(Vertex(scale*QVector3D(-r, r,1.0f),QVector3D(0.0f,0.0f,0)));
		sky_Vert.push_back(Vertex(scale*QVector3D( r, r,1.0f),QVector3D(0.5f,0.0f,0))); 
		sky_Vert.push_back(Vertex(scale*QVector3D( r,-r,1.0f),QVector3D(0.5f,1/3.0f,0)));
		sky_Vert.push_back(Vertex(scale*QVector3D(-r,-r,1.0f),QVector3D(0.0f,1/3.0f,0)));


		// Common Axis Y - Draw Down side
	
		sky_Vert.push_back(Vertex(scale*QVector3D(-r,-r,-1.0f),QVector3D(0.5f,0.0f,0)));
		sky_Vert.push_back(Vertex(scale*QVector3D( r,-r,-1.0f),QVector3D(1.0f,0.0f,0))); 
		sky_Vert.push_back(Vertex(scale*QVector3D( r, r,-1.0f),QVector3D(1.0f,1/3.0f,0)));
		sky_Vert.push_back(Vertex(scale*QVector3D(-r, r,-1.0f),QVector3D(0.5f,1/3.0f,0)));

		//rendManager.createVAO(sky_Vert,skyVBO,skyVAO,numVertex);
		rendManager.addStaticGeometry(QString("sky"),sky_Vert,imgName,GL_QUADS,2);// texture not lighting


		//initialized=1;
		printf("Loading SkyBox images... loaded %d\n",sky_Vert.size());


		//initialized=1;
	}//

} // namespace LC
