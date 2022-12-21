//---------------------------------------------------------------------------------------------------------------------
// Copyright 2017, 2018 Purdue University, Ignacio Garcia Dorado, Daniel Aliaga
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the 
// following conditions are met:
//
// 1. Redistributions of source code must retain the aBve copyright notice, this list of conditions and the
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
*		@desc Shadow code
*
************************************************************************************************/

#include "LC_GLWidget3D_Shadows.h"
#include "LC_GLWidget3D.h"
#include "VBOUtil.h"

#define FAR_DIST 3000.0f//2500.0f

namespace LC {

	int shadowWidth=4096;//2048;
	int shadowHeight=4096;//2048;

	GLuint light_mvpMatrixLoc,light_dirLoc;
	GLuint light_biasMatrixLoc;

	uint FBO;
	#define MAX_SPLITS 4
	uint shadowMap[1];

	
	float split_weight = 0.75f;

	int cur_num_splits=1;

	QMatrix4x4 shad_cpm[MAX_SPLITS];

	GLWidgetSimpleShadow::GLWidgetSimpleShadow(){
		displayDepthTex=false;
		displayDepthTexInit=false;
		fustrumVBO=INT_MAX;
	}//

	GLWidgetSimpleShadow::~GLWidgetSimpleShadow(){
	}//
	

	void GLWidgetSimpleShadow::updateShadowMatrix(LCGLWidget3D* glWidget3D){

		QVector3D light_dir=glWidget3D->getCamera()->light_dir.toVector3D();
		//////////////////////////////////////////////////////
		float fov=45.0f;
		float aspect=(float)shadowWidth/(float)shadowHeight;

		float zfar=15000.0f*2;//3800.0
		float znear=7000.0f*2;//1500.0

		//float zfar=3800.0;
		//float znear=1500.0;

		float f = 1.0f / tan (fov * (M_PI / 360.0));
        float m[16]=
		{	f/aspect,	0,								0,									0,
		0,			f,								0,						 			0,
		0,			0,		(zfar+znear)/(znear-zfar),		(2.0f*zfar*znear)/(znear-zfar),
		0,			0,								-1,									0
		};

		light_pMatrix=QMatrix4x4(m);
		// BIAS MATRIX
		light_biasMatrix.setToIdentity();
		light_biasMatrix.scale(0.5f);
		light_biasMatrix.translate(0.5f,0.5f,0.5f);
		// UNIFORMS LOC
		light_mvpMatrixLoc= glGetUniformLocation(programId, "light_mvpMatrix");
		light_biasMatrixLoc= glGetUniformLocation(programId, "light_biasMatrix");
		light_dirLoc= glGetUniformLocation(programId, "lightDir");
		//printf("LOC lights %d %d %d\n",light_mvpMatrixLoc,light_biasMatrixLoc,light_dirLoc);

		// UPDATE MATRIX
		light_mvMatrix.setToIdentity();
		light_mvMatrix.lookAt(-light_dir*10000*2,//2500
			light_dir.normalized(),//QVector3D(-0.60f,0.55,-0.6),
			QVector3D(0.0f, 0.0f, 1.0f));
		light_mvpMatrix=light_pMatrix*light_mvMatrix;


		///////////////////////////////////////////////////////

		float light_mvpMatrixArray[16];
		float light_biasMatrixArray[16];
		for(int i=0;i<16;i++){
			light_mvpMatrixArray[i]=light_mvpMatrix.data()[i];
			light_biasMatrixArray[i]=light_biasMatrix.data()[i];
		}

		glUniformMatrix4fv(light_mvpMatrixLoc,1,GL_FALSE,light_mvpMatrixArray);
		glUniformMatrix4fv(light_biasMatrixLoc,1,GL_FALSE,light_biasMatrixArray);
		glUniform3f(light_dirLoc,light_dir.x(),light_dir.y(),light_dir.z());
	}//

	void CheckFramebufferStatus()
	{
		int status;
		status = (GLenum) glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
		switch(status) {
		case GL_FRAMEBUFFER_COMPLETE_EXT:
			break;
		case GL_FRAMEBUFFER_UNSUPPORTED_EXT:
			printf("Unsupported framebuffer format\n");exit(0);
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT:
			printf("Framebuffer incomplete, missing attachment\n");exit(0);
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT:
			printf("Framebuffer incomplete, incomplete attachment\n");exit(0);
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:
			printf("Framebuffer incomplete, attached images must have same dimensions\n");exit(0);
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:
			printf("Framebuffer incomplete, attached images must have same format\n");exit(0);
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT:
			printf("Framebuffer incomplete, missing draw buffer\n");exit(0);
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT:
			printf("Framebuffer incomplete, missing read buffer\n");exit(0);
			break;
		default:
			exit(0);
		}

	}

	void GLWidgetSimpleShadow::initShadow(int _programId,LCGLWidget3D* glWidget3D){
		printf("---------- >> INIT SHADOW\n");
		
		programId=_programId;

		// PROJECTION MATRIX
		float fov=45.0f; // Think if bigger
		float aspect=(float)shadowWidth/(float)shadowHeight;
		
		float zfar=3300.0f;// 
		float znear=1200.0f;// FIX

		float f = 1.0f / tan (fov * (M_PI / 360.0));
        float m[16]=
		{	f/aspect,	0,								0,									0,
			0,			f,								0,						 			0,
			0,			0,		(zfar+znear)/(znear-zfar),		(2.0f*zfar*znear)/(znear-zfar),
			0,			0,								-1,									0
		};

		

		light_pMatrix=QMatrix4x4(m);
		// BIAS MATRIX
		light_biasMatrix.setToIdentity();
		light_biasMatrix.scale(0.5f);
		light_biasMatrix.translate(0.5f,0.5f,0.5f);
		// UNIFORMS LOC
		light_mvpMatrixLoc= glGetUniformLocation(programId, "light_mvpMatrix");
		light_biasMatrixLoc= glGetUniformLocation(programId, "light_biasMatrix");
		light_dirLoc= glGetUniformLocation(programId, "lightDir");
		printf("LOC lights %d %d %d\n",light_mvpMatrixLoc,light_biasMatrixLoc,light_dirLoc);
		
		// UPDATE MATRIX
		//QVector3D light_position=glWidget3D->getCamera()->light_dir.toVector3D();
		light_mvMatrix.setToIdentity();
		
		// FIX

		light_mvMatrix.lookAt(QVector3D(1365.0,-1200.0f,1245.0),
			QVector3D(-0.60f,0.55,-0.6),
			QVector3D(0.0f, 0.0f, 1.0f));
		light_mvpMatrix=light_pMatrix*light_mvMatrix;
		
		updateShadowMatrix(glWidget3D);
		
		// INIT shadowMap
		// FBO
		glGenFramebuffers(1,&FBO);
		glBindFramebuffer(GL_FRAMEBUFFER,FBO);
		
		////////////////////
		glGenTextures(MAX_SPLITS, &shadowMap[0]);
	

			glActiveTexture(GL_TEXTURE6);
			glBindTexture(GL_TEXTURE_2D,shadowMap[0]);

			glTexImage2D(GL_TEXTURE_2D, 0,GL_DEPTH_COMPONENT32, shadowWidth, shadowHeight, 0,GL_DEPTH_COMPONENT, GL_FLOAT, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		

		glActiveTexture(GL_TEXTURE0);
		
		////
		VBOUtil::check_gl_error("Init ShadowMap");
		glUniform1i(glGetUniformLocation(programId,"shadowMap"), 6);//shadowMap in tex1
		glBindFramebuffer(GL_FRAMEBUFFER,0);
		printf("---------- << INIT SHADOW\n");
	}//




	void GLWidgetSimpleShadow::makeShadowMap(LCGLWidget3D* glWidget3D){
	
		glDisable(GL_TEXTURE_2D);

		int origWidth=glWidget3D->width();
		int origHeigh=glWidget3D->height();
		

		// update camera fustrum (for overview)
		//updateSplitDist(glWidget3D->getCam()->f, 1.0f, FAR_DIST);//FAR_DIST/6 to display
		
		// generate shadow map using drawScene(1)
		glBindFramebuffer(GL_FRAMEBUFFER,FBO);
		glViewport(0,0,shadowWidth,shadowHeight);
		//glEnable(GL_CULL_FACE);
		//glCullFace(GL_FRONT);
		//glPolygonOffset( 1.0f, 4096.0f);
		glPolygonOffset( 1.1f, 4096.0f);
		glEnable(GL_POLYGON_OFFSET_FILL);
		// draw all faces since our terrain is not closed.
		//glDisable(GL_CULL_FACE);
		for(int i=0; i<cur_num_splits; i++){//cur_num_splits
			//clientMain->mGLWidget_3D->myCam->updateFrustumPoints(clientMain->mGLWidget_3D->myCam->f[i], clientMain->mGLWidget_3D->myCam->cam_pos, clientMain->mGLWidget_3D->myCam->cam_view);

			updateShadowMatrix(glWidget3D);
		
			glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, shadowMap[i], 0);
			//glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowMap[i], 0);
			glDrawBuffer(GL_NONE); // No color buffer is drawn to.
			CheckFramebufferStatus();
			glClear(GL_DEPTH_BUFFER_BIT);
			float light_mvpMatrixArray[16];
			for(int j=0;j<16;j++){
				light_mvpMatrixArray[j]=light_mvpMatrix.data()[j];
			}
			glUniformMatrix4fv(light_mvpMatrixLoc,1,GL_FALSE,light_mvpMatrixArray);
			//RENDER
			glWidget3D->drawScene(1);//1 light mode1
			
			// save camera for later
			shad_cpm[i]=light_mvpMatrix;
		}

		glBindFramebuffer(GL_FRAMEBUFFER,0);
		// revert to normal back face culling as used for rendering
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		glDisable(GL_POLYGON_OFFSET_FILL);
		glViewport(0,0,origWidth,origHeigh);

	}//

	

	void GLWidgetSimpleShadow::shadowRenderScene(int program){
		GLuint mainProgram=program;//save
		//glUniformMatrix4fv(light_mvpMatrixLoc,1,GL_FALSE,&shadowMatrix[0][0]);
		glActiveTexture(GL_TEXTURE6);
		glBindTexture(GL_TEXTURE_2D, shadowMap[0]);//groundTex);//1);
		glUniform1i(glGetUniformLocation(mainProgram,"shadowMap"), 6);//shadowMap in tex1
		printf("------** showMap %d\n",glGetUniformLocation(mainProgram,"shadowMap"));
		glActiveTexture(GL_TEXTURE0);
		
		// L
		float light_mvpMatrixArray[16];
		for(int j=0;j<16;j++){
			light_mvpMatrixArray[j]=shad_cpm[0].data()[j];
		}

		glUniformMatrix4fv(light_mvpMatrixLoc,1,GL_FALSE,light_mvpMatrixArray);

	}//
	

	
}//
