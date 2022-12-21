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
#include "VBOShader.h"
#include "VBOUtil.h"
#include "qmap.h"

#include "VBOWater.h"
#include "VBOSkyBox.h"
#include "VBOTerrain.h"
#include "VBOGUI.h"
#include "VBOText.h"

#include "VBOPeopleJobInfoLayer.h"

#include "VBOModel_StreetElements.h"

#ifndef Q_MOC_RUN
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/multi/multi.hpp>
#include <boost/polygon/polygon.hpp>
#endif

namespace LC {

	//class VBOTerrain;
	//class VBOSkyBox;
	//class vboWater;

	const int mode_AdaptTerrain=0x0100;
	const int mode_Lighting=0x0200;
	const int mode_TexArray=0x0400;
	const int mode_Tex3D=0x0800;
	//0x0100 --> adapt terrain
	//0x0200 --> lighting

	struct RenderSt{
		uint texNum;//0 means use color
		//int gridIndex;
		GLuint vbo;
		GLuint vao;
		int numVertex;//defines if the vbo has been created
		std::vector<Vertex> vertices;

		GLenum geometryType;
		int shaderMode;

		RenderSt(uint _texNum,std::vector<Vertex> _vertices,GLenum geoT,int shModer){
			texNum=_texNum;
			vertices=_vertices;
			geometryType=geoT;
			shaderMode=shModer;
			numVertex=-1;
		}
		RenderSt(){
			numVertex=-1;
		}
	};

	typedef QHash<uint,RenderSt> renderGrid;

	/////////////////////////////////////
	// VBORenderManager

	class VBORenderManager {
	
	public:

		// POLYGON
		typedef boost::polygon::point_data<double> pointP;
		typedef boost::polygon::polygon_set_data<double> polygon_setP;
		typedef boost::polygon::polygon_with_holes_data<double> polygonP;
		typedef std::pair<pointP, pointP> edgeP;
		typedef std::vector<boost::polygon::polygon_data<double> > PolygonSetP;


		QMap<QString,int> geoNameToGeoNum;
		GLuint program;
		int currentIndexGeo;

		VBORenderManager();
		~VBORenderManager();

		void init();

		// terrain
		bool editionMode;
		QVector3D mousePos3D;
		VBOTerrain vboTerrain;
		void changeTerrainDimensions(float terrainSide,int resolution);
		float getTerrainHeight(float xP,float xY,bool actual=false);/// !!
		void changeTerrainShader(int newMode);
		QVector3D minPos;
		QVector3D maxPos;
		float side;

		// layer
		PeopleJobInfoLayers layers;
		void activeLayersMode(bool activeLayers,int layerActive);
		
		// sky
		VBOSkyBox vboSkyBox;
		
		/// water
		VBOWater vboWater;
		void renderWater();

		/// GUI
		VBOGUI vboGUI;

		// textures
		QHash<QString,GLuint> nameToTexId;
		GLuint loadTexture(const QString fileName,bool mirrored=false);
		GLuint loadArrayTexture(QString texName,std::vector<QString> fileNames);

		//static
		bool addStaticGeometry(QString geoName,std::vector<Vertex>& vert,QString textureName,GLenum geometryType,int shaderMode);
		bool addStaticGeometry2(QString geoName,std::vector<QVector3D>& pos,float zShift,bool inverseLoop,QString textureName,GLenum geometryType,int shaderMode);
        bool addStaticConvexPoly(QString geoName,std::vector<Vector3D>& pos,float zShift,bool inverseLoop,QString textureName,int shaderMode,QVector3D texScale,bool tesselate=true,QVector3D* color=0);
		bool removeStaticGeometry(QString geoName,bool warning=true);
		void renderStaticGeometry(QString geoName);


		//grid
		bool addGridGeometry(QString geoName,std::vector<Vertex>& vert,QString textureName);
		bool removeGridGeometry(QString geoName);
		void renderGridGeometry(QString geoName);

		//models
		QHash<QString,std::vector<ModelSpec>> nameToVectorModels;
		bool initializedStreetElements;
		void addStreetElementModel(QString name,ModelSpec mSpec);
		void renderAllStreetElementName(QString name);
		void removeAllStreetElementName(QString name);

		
		void renderAll(bool cleanVertex);
	private:

		QHash<QString,QHash<int,renderGrid>> geoName2RenderGrids;
		QHash<QString,renderGrid> geoName2StaticRender;

		void renderVAO(RenderSt& renderSt,bool cleanVertex);
		bool createVAO(std::vector<Vertex>& vert,GLuint& vbo,GLuint& vao,int& numVertex);
		void cleanVAO(GLuint vbo,GLuint vao);
		
	};

} // namespace LC
