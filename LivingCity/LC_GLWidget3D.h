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
#include <QGLWidget>
#include "VBORenderManager.h"
#include "LC_camera_3d.h"

#include "Geometry/client_geometry.h"
#include "VBORoadGraph.h"
#include "VBOBlocks.h"
#include "VBOVegetation.h"
#include "VBORoadLabels.h"

#include "LC_GLWidget3D_Shadows.h"

#include "traffic/b18TrafficSimulator.h"

namespace LC {
	static const int M_NORMAL=0;
	static const int M_AREA_SELECT_CENTER=1;
	static const int M_RADIUS_AREA_SELECT_CENTER=2;
	static const int M_PTYPE_CENTER_EDITION=30;
	static const int M_PTYPE_EXT_HANDLER_EDITION=31;
	static const int M_PTYPE_EXT_HANDLER2_EDITION=32;
	static const int M_PTYPE_UDIST_HANDLER_EDITION=33;
	static const int M_PTYPE_VDIST_HANDLER_EDITION=34;

	static const int M_BOUNDING_POLYGON_EDITION=7;

//class UrbanMain;
class LCUrbanMain;

class LCGLWidget3D : public QGLWidget {
//	Q_OBJECT

protected:
	LCUrbanMain* urbanMain;
	Camera3D *myCam;
	
	bool shiftPressed;
	bool controlPressed;
	bool altPressed;
	bool keyMPressed;
	bool keyLPressed;
	QPoint lastPos;
	float farPlaneToSpaceRadiusFactor;
	float spaceRadius;
	float rotationSensitivity;
	float zoomSensitivity;

	QMatrix4x4 mvpMatrix;
	QMatrix4x4 pMatrix;
	QMatrix3x3 normalMatrix;

	
	GLWidgetSimpleShadow shadow;
	bool shadowEnabled;
public:
	VBORenderManager vboRenderManager;

	//traffic
	B18TrafficSimulator b18TrafficSimulator;

	LCGLWidget3D(QWidget *parent = 0);
	~LCGLWidget3D();	

	Camera3D* getCamera() { return myCam; }

	void updateMe();
	QSize minimumSizeHint() const;
	QSize sizeHint() const;
	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);
	int keyHold;

	void resetMyCam();

	void setLightPosition(double altitude, double azimuth);
	void setLightPosition(GLfloat x, GLfloat y, GLfloat z);

	bool mouseTo3D(int x, int y, QVector3D* result);
	bool mouseMoved;

	void drawScene(int drawMode);

	/// GEOMETRY
	ClientGeometry cg;
	VBORoadGraph vboRoadGraph;

	bool generateGeometry(int flag);
	void updateCamera();
	void changeRender2D3D();
signals:	

protected:
	void initializeGL();

	void resizeGL(int width, int height);
	void paintGL();    
		
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);

	void changeXRotation(float angle);
	void changeYRotation(float angle);
	void changeZRotation(float angle);
	void changeXYZTranslation(float dx, float dy, float dz);
};

} // namespace LC
