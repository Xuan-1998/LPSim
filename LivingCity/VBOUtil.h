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

#pragma once

#include "GL/glew.h"
#include "qstring.h"
#include "qvector3d.h"
#include <vector>

namespace LC {

	/////////////////////////
	// Contaisn the Vertex structure to do the rendering
	struct Vertex{
		float info[12];	
		// Contrustors
		Vertex(){
		}
		Vertex(float x,float y,float z,float r,float g,float b,float nX,float nY,float nZ,float tS,float tT,float tW){
			info[0]=x;
			info[1]=y;
			info[2]=z;

			info[3]=r;
			info[4]=g;
			info[5]=b;

			info[6]=nX;
			info[7]=nY;
			info[8]=nZ;

			info[9]=tS;
			info[10]=tT;
			info[11]=tW;
		}//
		Vertex(QVector3D pos,QVector3D color,QVector3D normal,QVector3D tex){
			info[0]=pos.x();
			info[1]=pos.y();
			info[2]=pos.z();

			info[3]=color.x();
			info[4]=color.y();
			info[5]=color.z();

			info[6]=normal.x();
			info[7]=normal.y();
			info[8]=normal.z();

			info[9]=tex.x();
			info[10]=tex.y();
			info[11]=tex.z();
		}//
		Vertex(QVector3D* pos,QVector3D* color=0,QVector3D* normal=0,QVector3D* tex=0){
			if(pos!=0){
				info[0]=pos->x();
				info[1]=pos->y();
				info[2]=pos->z();
			}
			if(color!=0){
				info[3]=color->x();
				info[4]=color->y();
				info[5]=color->z();
			}
			if(normal!=0){
				info[6]=normal->x();
				info[7]=normal->y();
				info[8]=normal->z();
			}
			if(tex!=0){
				info[9]=tex->x();
				info[10]=tex->y();
				info[11]=tex->z();
			}
		}//
		Vertex(QVector3D pos,QVector3D tex){
			info[0]=pos.x();
			info[1]=pos.y();
			info[2]=pos.z();

			info[9]=tex.x();
			info[10]=tex.y();
			info[11]=tex.z();
		}//
		// Overload bracket operator
		float operator [](int i) const    {return info[i];}
		float & operator [](int i) {return info[i];}
		void print(){
			printf("pos %f %f %f col %f %f %f normal %f %f %f tex %f %f %f\n",info[0],info[1],info[2],info[3],info[4],info[5],info[6],info[7],info[8],info[9],info[10],info[11]);
		}
	};


	class VBOUtil{

	public:

		static GLuint loadImage(const QString fileName,bool mirroredHor=false,bool mirroredVert=false);
		static GLuint loadImageArray(std::vector<QString> filaNames);

		static void check_gl_error(QString sourceTag);
		static void disaplay_memory_usage();
	};

}
