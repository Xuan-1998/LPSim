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
#include "qvector3d.h"
#include "qvector4d.h"
#include "qmatrix4x4.h"
#include "qtextstream.h"


namespace LC {


	class Camera3D
	{  
	public:

		
		// CAMERA

		QVector3D cam_pos; 
		QVector3D cam_view;
		float fovy;

		QVector4D light_dir;
		QMatrix4x4 mvMatrix;
		QMatrix4x4 mvpMatrix;
		QMatrix4x4 pMatrix;
		QMatrix3x3 normalMatrix;

		float sensitivity;
		float walk_speed;

		Camera3D();

		~Camera3D(){}
		//
		void resetCamera(void);

		void saveCameraPose(int numCam);
		void loadCameraPose(int numCam);

		void rotate_view(QVector3D& view, float angle, float x, float y, float z);

		void motion(int dx, int dy,bool moveLight=false);

		void updateCamMatrix();
		void updatePerspective(int width,int height);

		//void cameraInverse(float dst[16], float src[16]);

		void moveKey(int typeMode,float factor=1.0f);

		void printCamera();

		//void resizeCam(int halfW,int halfH);

	};
}
