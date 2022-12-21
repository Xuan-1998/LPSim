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

#include "VBOModel_StreetElements.h"
#include "VBORenderManager.h"

namespace LC {

	std::vector<VBOModel> VBOModel_StreetElements::streetElementsModels;
	bool VBOModel_StreetElements::streetElements_wasInitialized=false;


	void VBOModel_StreetElements::clearVBOModel_StreetElements(){
		//destroy models
		for(int mN=0;mN<streetElementsModels.size();mN++){
			streetElementsModels[mN].clearModel();
		}
		streetElementsModels.clear();
	}//

	void VBOModel_StreetElements::initStreetElements(){
		printf("initStreetElements...\n");
		// init models
		// tree
		VBOModel m;
		int ind=streetElementsModels.size();
		streetElementsModels.push_back(m);
		std::vector<QString> fileNames;

		fileNames.push_back("data/models/nacho_leaves2.obj");
		fileNames.push_back("data/models/nacho_trunk2.obj");
		
		streetElementsModels[ind].initModel(fileNames);
		//streetElementsModels[ind].initScale(0.1f);//trees too big
		streetElementsModels[ind].initScale(0.08f);
		streetElementsModels[ind].loadModel();

		// street light
		ind=streetElementsModels.size();
		streetElementsModels.push_back(m);
		fileNames.clear();
		fileNames.push_back("data/models/street_lamp.obj");//3m
		streetElementsModels[ind].initModel(fileNames);
		streetElementsModels[ind].initScale(5.0f);
		streetElementsModels[ind].loadModel();

		/*// sun
		ind=streetElementsModels.size();
		streetElementsModels.push_back(m);
		fileNames.clear();
		fileNames.push_back("data\\models\\sun.obj");//3m
		streetElementsModels[ind].initModel(fileNames);
		streetElementsModels[ind].initScale(1000.0f);
		streetElementsModels[ind].loadModel();*/

		streetElements_wasInitialized=true;
	}


	void VBOModel_StreetElements::renderOneStreetElement(int programId,ModelSpec& treeStr){
		if(streetElements_wasInitialized==false){
			initStreetElements();
		}
		glCullFace(GL_FRONT);
		glUniform1i (glGetUniformLocation (programId, "mode"), 5|LC::mode_Lighting|LC::mode_AdaptTerrain);//model obj: one color

		streetElementsModels[treeStr.type].renderModel(programId, treeStr);
		
		glCullFace(GL_BACK);
		VBOUtil::check_gl_error("RenderOneStreetElement");
	}//
	
}
