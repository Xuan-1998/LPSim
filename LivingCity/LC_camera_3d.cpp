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

#include "LC_camera_3d.h"
#include "qfile.h"
#include <math.h>

namespace LC {

	Camera3D::Camera3D(){
		sensitivity = 0.002f;
		walk_speed = 150.0f;

		this->resetCamera();	
	}//


	void Camera3D::resetCamera(void){
		cam_pos=QVector3D(750.0, 500.0f, -900);
		cam_view=QVector3D(-0.5f, -0.5f, 0.7f);

		light_dir=QVector4D(-0.40f,0.81f,-0.51f,0.00f);
		fovy = 60.0f;
	}

	void Camera3D::rotate_view(QVector3D& view, float angle, float x, float y, float z){
		float new_x;
		float new_y;
		float new_z;

        float c = cos(angle);
		float s = sin(angle);

		new_x  = (x*x*(1-c) + c)	* view.x();
		new_x += (x*y*(1-c) - z*s)	* view.y();
		new_x += (x*z*(1-c) + y*s)	* view.z();

		new_y  = (y*x*(1-c) + z*s)	* view.x();
		new_y += (y*y*(1-c) + c)	* view.y();
		new_y += (y*z*(1-c) - x*s)	* view.z();

		new_z  = (x*z*(1-c) - y*s)	* view.x();
		new_z += (y*z*(1-c) + x*s)	* view.y();
		new_z += (z*z*(1-c) + c)	* view.z();

		view=QVector3D(new_x,new_y,new_z);
		view.normalize();
	}//

	void Camera3D::motion(int dx, int dy,bool moveLight){

		float rot_x, rot_y;
		QVector3D rot_axis;

		rot_x = -dx *sensitivity;//- to invert
		rot_y = dy *sensitivity;

		if(moveLight==true)	{
			printf("1Move light %f %f %f %f\n",light_dir.x(),light_dir.y(),light_dir.z());
			light_dir.setY(light_dir.y()+ rot_y);
			if(light_dir.y() < 0.2f)
				light_dir.setY(0.2f);
			light_dir.normalize();
			QVector3D light_dir3D=light_dir.toVector3D();
			rotate_view(light_dir3D, -rot_x, cam_view.x(), cam_view.y(), cam_view.z());
			light_dir=QVector4D(light_dir3D.x(),light_dir3D.y(),light_dir3D.z(),light_dir.w());
			printf("2Move light %f %f %f %f\n",light_dir.x(),light_dir.y(),light_dir.z());
		}else{
			rotate_view(cam_view, rot_x, 0.0f, 1.0f, 0.0f);
			rot_axis=QVector3D(-cam_view.z(),0,cam_view.x());
			rot_axis.normalize();
			rotate_view(cam_view, rot_y, rot_axis.x(), rot_axis.y(), rot_axis.z());
		}
	}

	void Camera3D::updateCamMatrix(){
		mvMatrix.setToIdentity();
		QVector3D dirV=cam_pos+ cam_view;
		mvMatrix.lookAt(QVector3D(cam_pos.x(),cam_pos.z(),cam_pos.y()),
			//cam_pos+ cam_view,
			QVector3D(dirV.x(),dirV.z(),dirV.y()),
			QVector3D(0.0f, 0.0f, 1.0f));
		// normal matrix
		normalMatrix=mvMatrix.normalMatrix();
		// mvp
		mvpMatrix=pMatrix*mvMatrix;
		//printf("cam3d updateM\n");
	}//

	void Camera3D::updatePerspective(int width,int height){
		float aspect=(float)width/(float)height;
		float zfar=300000.0f;//90000.0f;
		float znear=200.0f;

		float f = 1.0f / tan (fovy * (0.00872664625f));//PI/360

        float m[16]=
		{	 f/aspect,	0,								0,									0,
					0,	f,								0,						 			0,
			        0,	0,		(zfar+znear)/(znear-zfar),		(2.0f*zfar*znear)/(znear-zfar),
			        0,	0,		    				   -1,									0

		};
        pMatrix=QMatrix4x4(m);
	}

	void Camera3D::moveKey(int typeMode,float factor){
		if(typeMode==0){//GetAsyncKeyState('W')){
			cam_pos.setX(cam_pos.x()+ cam_view.x() * walk_speed*factor);
			cam_pos.setY(cam_pos.y()+ cam_view.y() * walk_speed*factor);
			cam_pos.setZ(cam_pos.z()+ cam_view.z() * walk_speed*factor);
		}
		if(typeMode==1){//if(GetAsyncKeyState('S')){
			cam_pos.setX(cam_pos.x() - cam_view.x() * walk_speed*factor);
			cam_pos.setY(cam_pos.y() - cam_view.y() * walk_speed*factor);
			cam_pos.setZ(cam_pos.z() - cam_view.z() * walk_speed*factor);
		}
		if(typeMode==2){//if(GetAsyncKeyState('A')){
			cam_pos.setX(cam_pos.x()+cam_view.z() * walk_speed*factor);
			cam_pos.setZ(cam_pos.z()-cam_view.x() * walk_speed*factor);		
		}
		if(typeMode==3){//if(GetAsyncKeyState('D')){
			cam_pos.setX(cam_pos.x()-cam_view.z() * walk_speed*factor);
			cam_pos.setZ(cam_pos.z()+cam_view.x() * walk_speed*factor);
		}

		if(typeMode==4){//if(GetAsyncKeyState(VK_SPACE)){
			cam_pos.setY( cam_pos.y()+ walk_speed*factor);
		}
	}//

	void Camera3D::printCamera(){
		printf("cam_pos  %f %f %f cam_view %f %f %f fovy %f\n",cam_pos.x(),cam_pos.y(),cam_pos.z(),cam_view.x(),cam_view.y(),cam_view.z(),fovy);
	}

	void Camera3D::saveCameraPose(int numCam)
	{
		QFile camFile("data/camPose"+QString::number(numCam)+".cam");
		if (!camFile.open(QIODevice::WriteOnly | QIODevice::Text)){
			printf("ERROR: Cannot open the file cam.txt for writing\n");
			return;
		}
		QTextStream stream( &camFile );	

		stream << this->cam_pos.x() << " " << this->cam_pos.y() << " " << this->cam_pos.z() << " " <<
			this->cam_view.x() << " " << this->cam_view.y() << " " << " " << this->cam_view.z() << " " << this->fovy;		

		camFile.close();
	}//

	void Camera3D::loadCameraPose(int numCam){
		QFile camFile("data/camPose"+QString::number(numCam)+".cam");

		if (!camFile.open(QIODevice::ReadOnly | QIODevice::Text)){ // Open the file		
			printf("Can't open file camPose%d.cam\n",numCam);
		}

		else {
			QTextStream stream( &camFile); // Set the stream to read from myFile
			float x,y,z,dx,dy,dz,fov;
			stream >> x;
			stream >> y;
			stream >> z;
			stream >> dx;
			stream >> dy;
			stream >> dz;
			stream >> fov;
			cam_pos=QVector3D(x,y,z);
			cam_view=QVector3D(dx,dy,dz);
			fovy=fov;
		}
	}//

}
