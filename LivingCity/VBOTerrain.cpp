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

#include "VBOTerrain.h"
#include <QFileInfo>
#include "VBOUtil.h"
#include "qimage.h"
#include <QGLWidget>

#include "VBORenderManager.h"

namespace LC {

	VBOTerrain::VBOTerrain() {
		initialized=false;

		/// resolution of vertices
		resolutionX=200;
		resolutionY=200;
	}

	VBOTerrain::~VBOTerrain() {
	}

	GLuint elementbuffer;
	GLuint vbo;
	GLuint indicesCount;
	GLuint grassText;
	GLuint program;


	void VBOTerrain::init(VBORenderManager& rendManager){

		QVector3D minPos=rendManager.minPos;
		QVector3D maxPos=rendManager.maxPos;
		

		//////////////////
		// TERRAIN LAYER
		if(initialized==false){
			terrainLayer.init(minPos,maxPos,0,0,0,200,200);

			glActiveTexture(GL_TEXTURE7);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
			glBindTexture(GL_TEXTURE_2D,terrainLayer.texData); 
			glActiveTexture(GL_TEXTURE0);

			int numLevels=4;
			std::vector<QString> fileNames(numLevels);
			for(int i=0;i<numLevels;i++){
				fileNames[i]="data/textures/0"+QString::number(i+1)+"_terrain.jpg";
			}
			grassText=VBOUtil::loadImageArray(fileNames);
		}else{
			terrainLayer.minPos=minPos;
			terrainLayer.maxPos=maxPos;
		}

		//////////////////
		// VERTICES
		if(initialized==true){
			glDeleteBuffers(1, &vbo);
			glDeleteBuffers(1, &elementbuffer);
			vbo=0;elementbuffer=0;

		}

		program=rendManager.program;

		std::vector<Vertex> vert;
		float sideX=abs(maxPos.x()-minPos.x())/resolutionX;
		float sideY=abs(maxPos.y()-minPos.y())/resolutionY;
		
		// VERTEX
		//stackoverflow.com/questions/5915753/generate-a-plane-with-triangle-strips
		vert.resize((resolutionX+1)*(resolutionY+1));
		int count=0;

		int width=resolutionX+1;
		int height=resolutionY+1;
		float maxVertexHeight=0;
		for ( int row=0; row<height; row++ ) {
			for ( int col=0; col<width; col++ ) {
				QVector3D pos=QVector3D(col*sideX,row*sideY,0);
				pos+=minPos;
				vert[count]=Vertex(pos,pos/100.0f);
				count++;
			}
		}
		printf("max vertex height %f vert count %d\n",maxVertexHeight,count);
		glGenBuffers(1, &vbo);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex)*vert.size(), vert.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		// INDEX
		std::vector<ushort> indices;
		for ( int row=0; row<height-1; row++ ) {
			for ( int col=0; col<width-1; col++ ) {
				indices.push_back(col+row*width);
				indices.push_back((col+1)+row*width);
				indices.push_back((col+1)+(row+1)*width);
				indices.push_back((col)+(row+1)*width);
			}
		}


		printf("Num %d -> %d\n",indices.size(),(width*height) + (width-1)*(height-2));
		glGenBuffers(1, &elementbuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(ushort), &indices[0], GL_STATIC_DRAW);
		indicesCount=indices.size();
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

		initialized=true;
	}//


	void VBOTerrain::render(VBORenderManager& rendManager ){//bool editionMode,QVector3D mousePos
		bool editionMode=rendManager.editionMode;
		QVector3D mousePos=rendManager.mousePos3D;
		//////////////////////////////////////////
		// TERRAIN
		//glCullFace(GL_FRONT);
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		GLuint vao;
		glGenVertexArrays(1,&vao); 
		glBindVertexArray(vao); 
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);

		glActiveTexture(GL_TEXTURE8);
		glBindTexture(GL_TEXTURE_2D,0); 

		glBindTexture(GL_TEXTURE_2D_ARRAY, grassText);
		glActiveTexture(GL_TEXTURE0);

		// terrain textures
		glUniform1i (glGetUniformLocation (program, "mode"), 3);//MODE: terrain
		glUniform1i (glGetUniformLocation (program, "tex_3D"), 8);//tex0: 0

		// elevation
		glUniform1i (glGetUniformLocation (program, "terrain_tex"), 7);
		glUniform4f (glGetUniformLocation (program, "terrain_size"), 
			terrainLayer.minPos.x(),
			terrainLayer.minPos.y(),
			(terrainLayer.maxPos.x()-terrainLayer.minPos.x()), 
			(terrainLayer.maxPos.y()-terrainLayer.minPos.y())
		 );

		// layer
		glUniform1i (glGetUniformLocation (program, "layer_tex"), 5);//LAYER

		//printf("tex_3D %d\n",glGetUniformLocation (program, "tex_2DA"));
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)(3*sizeof(float)));
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)(6*sizeof(float)));
		glEnableVertexAttribArray(3);
		glVertexAttribPointer(3,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)(9*sizeof(float)));

		// Draw the triangles 
		glDrawElements(
			GL_QUADS, // mode
			indicesCount,    // count
			GL_UNSIGNED_SHORT,   // type
			(void*)0           // element array buffer offset
			);

		 glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		 glBindBuffer(GL_ARRAY_BUFFER, 0);
		 glCullFace(GL_BACK);
		 glEnable(GL_CULL_FACE);
		 glBindVertexArray(0);
		glDeleteVertexArrays(1,&vao);

		 //////////////////////
		 // EDITION
		 if(G::global().getInt("render_mode")!=2&&//not 
			 editionMode==true){
			 glLineWidth(5.0f);
			 glPointSize(10.0f);

			 VBOUtil::check_gl_error(">>editionMode");
			 printf("**************** editionMode\n");
			const float pointOffset=5.0f;
			GLuint vao;
			glGenVertexArrays(1,&vao);
			glBindVertexArray(vao);
			GLuint edPointsVBO;
			glGenBuffers(1, &edPointsVBO);
			glBindBuffer(GL_ARRAY_BUFFER, edPointsVBO);
			
			std::vector<Vertex> points;
			int numPointsCircle=20;
			points.resize(numPointsCircle);

			QVector3D posM=rendManager.mousePos3D;//(0,0,0);
			// circle
			float radius=G::global().getFloat("2DterrainEditSize")*rendManager.side;
			int degreesSegmentCircle=360.0f/numPointsCircle;
			for (int i=0; i < numPointsCircle; i++){
				float degInRad = i*degreesSegmentCircle*0.0174532925f;//degree to rad
				QVector3D pos(cos(degInRad)*radius,sin(degInRad)*radius,0);
				pos+=posM;
				pos.setZ(50.0f);
				points[i]=Vertex(pos.x(),pos.y(),pos.z(),0,0,1.0f,0,0,0,0,0,0);
			}


			glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex)*points.size(), points.data(), GL_DYNAMIC_DRAW);
			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),0);
			glEnableVertexAttribArray(1);
			VBOUtil::check_gl_error("aa editionMode");
			glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)(3*sizeof(float)));
			glEnableVertexAttribArray(2);
			glVertexAttribPointer(2,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)(6*sizeof(float)));
			glEnableVertexAttribArray(3);
			glVertexAttribPointer(3,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)(9*sizeof(float)));
			glUniform1i (glGetUniformLocation (program, "mode"), 1|mode_AdaptTerrain);//MODE: color

			//draw points
			glDrawArrays(GL_POINTS,0,points.size()-numPointsCircle);
			//draw circle
			glDrawArrays(GL_LINE_LOOP,points.size()-numPointsCircle,numPointsCircle);

			glDeleteBuffers(1,&edPointsVBO);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindVertexArray(0);
			glDeleteVertexArrays(1,&vao);


			printf("****************\n");
		 }
	}//

	void VBOTerrain::updateTerrainNewValue(float coordX,float coordY,float newValue,float rad){
		terrainLayer.updateLayerNewValue(coordX,coordY,newValue,rad);

		glActiveTexture(GL_TEXTURE7);
		glBindTexture(GL_TEXTURE_2D,terrainLayer.texData); 
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glActiveTexture(GL_TEXTURE0);
	}//

	void VBOTerrain::updateTerrain(float coordX,float coordY,float change,float rad){

		terrainLayer.updateLayer(coordX,coordY,change,rad);

		glActiveTexture(GL_TEXTURE7);
		glBindTexture(GL_TEXTURE_2D,terrainLayer.texData); 
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glActiveTexture(GL_TEXTURE0);
	}//

	float VBOTerrain::getTerrainHeight(float xM,float yM,bool actual){
		if(!actual && (G::global()["3d_render_mode"]==0||G::global()["3d_render_mode"]==3)){//for Gen Mode it is flat
			return 0.0f;
		}
		float value255=terrainLayer.getValue(xM,yM);
		const float maxHeight=7.0;//7=255*7 1785m (change in vertex as well)
		float height=maxHeight*value255;
		return height;
	}//

	void VBOTerrain::loadTerrain(QString& fileName){
		terrainLayer.loadLayer(fileName);
		glActiveTexture(GL_TEXTURE7);
		glBindTexture(GL_TEXTURE_2D,terrainLayer.texData); 
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glActiveTexture(GL_TEXTURE0);
	}//
	
	void VBOTerrain::saveTerrain(QString& fileName){
		terrainLayer.saveLayer(fileName);
	}//

	void VBOTerrain::smoothTerrain(){
		terrainLayer.smoothLayer();
		glActiveTexture(GL_TEXTURE7);
		glBindTexture(GL_TEXTURE_2D,terrainLayer.texData); 
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glActiveTexture(GL_TEXTURE0);
	}//

} // namespace LC
