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
*		@desc Class containing the model loading and render
*		@author igarciad
*
************************************************************************************************/

#include "VBOModel.h"

namespace LC {


	VBOModel::VBOModel(){
		modelInitialized=0;
	}//

	VBOModel::~VBOModel(){

	}//

	void VBOModel::initModel(std::vector<QString>& names){
		fileNames=names;
		modelInitialized=1;
		scale=QVector3D(1.0f,1.0f,1.0f);
		positions.push_back(QVector3D());
	}//


	void VBOModel::initScale(float _scale){
		scale=QVector3D(_scale,_scale,_scale);
	}//

	void VBOModel::initScale(float scaleX,float scaleY,float scaleZ){
		scale=QVector3D(scaleX,scaleY,scaleZ);
	}//

	void VBOModel::setPositions(std::vector<QVector3D>& _positions){
		positions=_positions;
	}//

	void VBOModel::clearModel(){

	}//

	void VBOModel::loadModel(){
		if(modelInitialized==0){
			printf("ERROR: Init model before call load Model\n");
			return;
		}
		//printf("loadModel %d\n",fileNames.size());
		vertexVBO.resize(fileNames.size());
		indexVBO.resize(fileNames.size());
		vaoVBO.resize(fileNames.size());
		stride.resize(fileNames.size());
		normalOffset.resize(fileNames.size());
		numIndexCount.resize(fileNames.size());

		glGenVertexArrays(vaoVBO.size(),&vaoVBO[0]);
		glGenBuffers(vertexVBO.size(), &vertexVBO[0]);
		glGenBuffers(indexVBO.size(), &indexVBO[0]);

		for(int fN=0;fN<fileNames.size();fN++){

			//printf("fN %d\n",fN);
			QString modelFile=fileNames[fN];
			printf("loading OBJ tree mode... %s\n",modelFile.toUtf8().constData());
			nv::Model	*modelT;
			modelT = new nv::Model;
			if (!modelT->loadModelFromFile(modelFile.toUtf8().constData())) {
				printf("Error loading OBJ tree model %s\n",modelFile.toUtf8().constData());
				return;
			}
			//printf("2\n");

			if(scale!=QVector3D(1.0f,1.0f,1.0f)){
				modelT->scalePerNumber(scale.x(),scale.y(),scale.z());	
			}
			modelT->compileModel();
			//printf("3\n");
			int totalVertexSize = modelT->getCompiledVertexCount() * modelT->getCompiledVertexSize() * sizeof(GLfloat);
			int totalIndexSize = modelT->getCompiledIndexCount() * sizeof(GLuint);

			stride[fN] = modelT->getCompiledVertexSize() * sizeof(GLfloat);
			normalOffset[fN] = modelT->getCompiledNormalOffset() * sizeof(GLfloat);
			numIndexCount[fN]=modelT->getCompiledIndexCount();

			//printf("4 %d %d\n",modelT->getCompiledVertexCount(),modelT->getCompiledIndexCount());

			/////////////////////////////////////
			// create VAO and copy
			glBindVertexArray(vaoVBO[fN]);
			glBindBuffer(GL_ARRAY_BUFFER, vertexVBO[fN]);
			glBufferData(GL_ARRAY_BUFFER, totalVertexSize, modelT->getCompiledVertices(), GL_STATIC_DRAW);
			//glBindBuffer(GL_ARRAY_BUFFER, 0);
			
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVBO[fN]);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, totalIndexSize, modelT->getCompiledIndices(), GL_STATIC_DRAW);
			//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

			// Configure the attributes in the VAO.

			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,stride[fN],0);

			glEnableVertexAttribArray(2);
			glVertexAttribPointer(2,3,GL_FLOAT,GL_FALSE,stride[fN],(void*)(normalOffset[fN]));

			// Bind back to the default state.
			glBindVertexArray(0); 
			glBindBuffer(GL_ARRAY_BUFFER,0);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
			

			delete modelT;
		}
		modelInitialized=2;
	}//


	void VBOModel::renderModel(int programId,ModelSpec& modelSpec){
		if(modelInitialized==0){
			printf("ERROR: Model should be initialized\n");
			return;
		}
		if(modelInitialized==1){//load in case it is neccesary
			loadModel();
		}

		int justOneColor=glGetUniformLocation (programId, "justOneColor");
		int modelTransf=glGetUniformLocation (programId, "modelTransf");

		float transfArray[16];

		for(int fN=0;fN<fileNames.size();fN++){

			glBindVertexArray(vaoVBO[fN]);

			// set color
			if(modelSpec.colors.size()>fN)
				glUniform3f(justOneColor, modelSpec.colors[fN].x(),modelSpec.colors[fN].y(),modelSpec.colors[fN].z());

			// set transfor
			for(int i=0;i<16;i++)transfArray[i]=modelSpec.transMatrix.data()[i];
			glUniformMatrix4fv(modelTransf,1,false,transfArray);//color

			//glBindBuffer(GL_ARRAY_BUFFER, vertexVBO[fN]);
			//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVBO[fN]);

			glDrawElements(GL_TRIANGLES,(int) numIndexCount[fN] , GL_UNSIGNED_INT, NULL);
	
			glBindVertexArray(0);
		}
	}//
}
