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

#include "VBOUtil.h"
#include "qimage.h"
#include <QGLWidget>

#define DEBUG_GL 0

namespace LC {


	GLuint VBOUtil::loadImage(const QString fileName,bool mirroredHor,bool mirroredVert){
		QImage img;
		if( ! img.load( fileName ) ){
			printf("ERROR: loading %s\n",fileName.toUtf8().constData());
			return INT_MAX;
		}
		if(mirroredHor==true||mirroredVert)
			img=img.mirrored(mirroredHor,mirroredVert);
		QImage GL_formatted_image;
		GL_formatted_image = QGLWidget::convertToGLFormat(img);
		if( GL_formatted_image.isNull() ){
			printf("ERROR: GL_formatted_image\n");
			return INT_MAX;
		}

		glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);

		GLuint texture;
		glGenTextures(1, &texture);
		glBindTexture(GL_TEXTURE_2D, texture);
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA,
			GL_formatted_image.width(), GL_formatted_image.height(),
			0, GL_RGBA, GL_UNSIGNED_BYTE, GL_formatted_image.bits() );

		glGenerateMipmap(GL_TEXTURE_2D);
		return texture;

	}//

	GLuint VBOUtil::loadImageArray(std::vector<QString> filaNames){

		//////////////////
		// TEXTURE 3D
		int numLevels=filaNames.size();
		glEnable(GL_TEXTURE_2D_ARRAY);	
		glTexParameterf (GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);//GL_LINEAR_MIPMAP_LINEAR
		glTexParameterf (GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_GENERATE_MIPMAP, GL_TRUE);
		GLuint arrayTex;
		for(int i=0;i<numLevels;i++){
			QString fileName=filaNames[i];//"data/textures/0"+QString::number(i+1)+"_terrain.jpg";
			QImage img;
			if( ! img.load( fileName ) ){
				printf("ERROR: loading %s\n",fileName.toUtf8().constData());
				return 0;
			}

			if(i==0){
				glGenTextures(1,&arrayTex);
				glBindTexture(GL_TEXTURE_2D_ARRAY,arrayTex);
				//Allocate the storage.
				glTexStorage3D(GL_TEXTURE_2D_ARRAY, 4, GL_RGBA8, img.width(), img.height(), numLevels);
			}

			QImage GL_formatted_image;
			GL_formatted_image = QGLWidget::convertToGLFormat(img);
			if( GL_formatted_image.isNull() ){
				printf("ERROR: GL_formatted_image\n");
				return 0;
			}
			printf("img[%d] %d %d %s\n",i,img.width(), img.height(),fileName.toUtf8().constData());
			//Upload pixel data.
			glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, i, img.width(), img.height(), 1, GL_RGBA, GL_UNSIGNED_BYTE, GL_formatted_image.bits());
		}
		glGenerateMipmap(GL_TEXTURE_2D_ARRAY);
		//
		return arrayTex;
	}//

	using namespace std;
	void VBOUtil::check_gl_error(QString sourceTag) {
		if(DEBUG_GL==true){
			GLenum err (glGetError());

			while(err!=GL_NO_ERROR) {
				string error;

				switch(err) {
				case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
				case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
				case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
				case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
				case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
				}

				printf("GL_%s - %s\n",error.c_str(),sourceTag.toUtf8().constData());
				err=glGetError();
			}
		}
	}//

#define GL_GPU_MEM_INFO_TOTAL_AVAILABLE_MEM_NVX 0x9048
#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049

	void VBOUtil::disaplay_memory_usage() {
		GLint total_mem_kb = 0;
		glGetIntegerv(GL_GPU_MEM_INFO_TOTAL_AVAILABLE_MEM_NVX, 
			&total_mem_kb);

		GLint cur_avail_mem_kb = 0;
		glGetIntegerv(GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX, 
			&cur_avail_mem_kb);

		printf("GPU Memory Available %d of %d (%f%)\n",cur_avail_mem_kb,total_mem_kb,((float)cur_avail_mem_kb)/total_mem_kb);
	}//

}
