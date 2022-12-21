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

#include "VBORenderManager.h"
#include "triangle/triangle.c"

namespace LC {


	VBORenderManager::VBORenderManager(){
		editionMode=false;
		side=5000.0f;
		G::global()["worldWidth"]=side;
		minPos=QVector3D (-side/2.0f,-side/2.0f,0);
		maxPos=QVector3D (side/2.0f,side/2.0f,0);
		initializedStreetElements=false;
	}

	VBORenderManager::~VBORenderManager() {
		Shader::cleanShaders();
	}

	void VBORenderManager::init(){
		// init program shader
        QString vertex_shader_path("data/shaders/lc_vertex.glsl");
        QString fragment_shader_path("data/shaders/lc_fragment.glsl");
        program=Shader::initShader(vertex_shader_path, fragment_shader_path);
		glUseProgram(program);

		// init
		vboTerrain.init(*this);
		vboSkyBox.init(*this);
		layers.initInfoLayer(*this,200,true);

		nameToTexId[""]=0;

		printf("VBORenderManager\n");

	}//

	GLuint VBORenderManager::loadTexture(const QString fileName,bool mirrored){
		GLuint texId;
		if(nameToTexId.contains(fileName)){
			texId=nameToTexId[fileName];
		}else{
			texId=VBOUtil::loadImage(fileName,mirrored);
			nameToTexId[fileName]=texId;
		}
		return texId;
	}//

	GLuint VBORenderManager::loadArrayTexture(QString texName,std::vector<QString> fileNames){
		GLuint texId;
		if(nameToTexId.contains(texName)){
			texId=nameToTexId[texName];
		}else{
			texId=VBOUtil::loadImageArray(fileNames);
			nameToTexId[texName]=texId;
		}
		return texId;
	}

	// ATRIBUTES
	// 0 Vertex
	// 1 Color
	// 2 Normal
	// 3 UV

	// UNIFORMS
	// 0 mode
	// 1 tex0

	bool VBORenderManager::createVAO(std::vector<Vertex>& vert,GLuint& vbo,GLuint& vao,int& numVertex){
		glGenVertexArrays(1,&vao);
		glBindVertexArray(vao);
		// Crete VBO
		glGenBuffers(1, &vbo);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex)*vert.size(), vert.data(), GL_STATIC_DRAW);
	
		// Configure the attributes in the VAO.
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)(3*sizeof(float)));
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)(6*sizeof(float)));
		glEnableVertexAttribArray(3);
		glVertexAttribPointer(3,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)(9*sizeof(float)));


		// Bind back to the default state.
		glBindVertexArray(0); 
		glBindBuffer(GL_ARRAY_BUFFER,0);
		
	
		// clean space
		numVertex=vert.size();
		//vert.clear();
		return true;
	}//

	void VBORenderManager::renderVAO(RenderSt& renderSt,bool cleanVertex){
		//printf("renderVAO numVert %d texNum %d vao %d numVertVert %d\n",renderSt.numVertex,renderSt.texNum,renderSt.vao,renderSt.vertices.size());
		// 1. Create if necessary
		if(renderSt.numVertex!=renderSt.vertices.size()&&renderSt.vertices.size()>0){
			if(renderSt.numVertex!=-1){
				cleanVAO(renderSt.vbo,renderSt.vao);
			}
			// generate vao/vbo
			createVAO(renderSt.vertices,renderSt.vbo,renderSt.vao,renderSt.numVertex);
			if(cleanVertex)
				renderSt.vertices.clear();
		}
		// 2. Render
		// 2.1 TEX
		int mode=renderSt.shaderMode;
		if((mode&mode_TexArray)==mode_TexArray){
			// MULTI TEX
			mode=mode&(~mode_TexArray);//remove tex array bit
			glActiveTexture(GL_TEXTURE8);

			glBindTexture(GL_TEXTURE_2D,0); 
			glBindTexture(GL_TEXTURE_2D_ARRAY, renderSt.texNum);
			glActiveTexture(GL_TEXTURE0);
			glUniform1i (glGetUniformLocation (program, "tex_3D"), 8);
		}else{
			glBindTexture(GL_TEXTURE_2D, renderSt.texNum);
		}
		// 2.2 mode
		//if(renderSt.texNum==0){
			//glUniform1i (glGetUniformLocation (program, "mode"), 1|(renderSt.shaderMode&0xFF00));//MODE: same modifiers but just color (renderSt.shaderMode&0xFF00)
		//}else{
			glUniform1i (glGetUniformLocation (program, "mode"), mode);
		//}

		glUniform1i (glGetUniformLocation (program, "tex0"), 0);//tex0: 0

		glBindVertexArray(renderSt.vao);
		glDrawArrays(renderSt.geometryType,0,renderSt.numVertex);
		glBindVertexArray(0);
	}

	void VBORenderManager::renderAll(bool cleanVertex){
		//vboTerrain.render(editionMode,mousePos3D);
		//vboWater.render(*this);
		
		////////////////////////
		// 1. STATIC
		QHash<QString,renderGrid>::iterator statcIt;
		// 1.1 iterate over geoNames
		for (statcIt = geoName2StaticRender.begin(); statcIt != geoName2StaticRender.end(); ++statcIt){
			printf("Render: %s\n",statcIt.key().toUtf8().constData());
			// 1.1.1 iterate over textures
			renderGrid::iterator i;
			for (i = statcIt.value().begin(); i != statcIt.value().end(); ++i){
				renderVAO(i.value(),cleanVertex);
			}
		}

	}//

	

	void VBORenderManager::cleanVAO(GLuint vbo,GLuint vao){
		glDeleteBuffers(1, &vbo);
		glDeleteVertexArrays(1, &vao);
	}//

	void VBORenderManager::renderWater(){
		vboWater.render(*this);
	}//

	float VBORenderManager::getTerrainHeight(float xP,float xY,bool actual){
		float xM=1.0f-(side/2.0f-xP)/side;
		float yM=1.0f-(side/2.0f-xY)/side;
		return vboTerrain.getTerrainHeight(xM,yM,actual);
		//return 90.0f;
		//vboTerrain.updateTerrain(coordX,coordY,change,rad);
	}

	void VBORenderManager::changeTerrainDimensions(float terrainSide,int resolution){
		side=terrainSide;
		G::global()["worldWidth"]=side;
		minPos=QVector3D (-side/2.0f,-side/2.0f,0);
		maxPos=QVector3D (side/2.0f,side/2.0f,0);
		vboTerrain.resolutionX=resolution;
		vboTerrain.resolutionY=resolution;
		vboTerrain.init(*this);
		vboSkyBox.init(*this);
		float samplingResolution = std::min<float>(1000.0f, side / 10.0f);
		layers.initInfoLayer(*this, samplingResolution, true);
	}//

	void VBORenderManager::changeTerrainShader(int newMode){
		glUniform1i(glGetUniformLocation(program, "terrainMode"),newMode);//newMode
	}//

	//void VBORenderManager::updateLayer(QVector3D mouse3D,float change){
	//	vboTerrain.updateTerrain(mouse3D,change);
	//}//

	///////////////////////////////////////////////////////////////////
	// STATIC
	bool VBORenderManager::addStaticGeometry(QString geoName,std::vector<Vertex>& vert,QString texName,GLenum geometryType,int shaderMode){
		GLuint texId;
		if(nameToTexId.contains(texName)){
			texId=nameToTexId[texName];
		}else{
			printf("load img %s\n",texName.toUtf8().constData());
			texId=VBOUtil::loadImage(texName);
			nameToTexId[texName]=texId;
		}
		if(geoName2StaticRender.contains(geoName)){
			// 1.1 If already in manager
			if(geoName2StaticRender[geoName].contains(texId)){
				if(geoName2StaticRender[geoName][texId].vertices.size()==0){
					//1.1.1 if also contains texture and the number of vertex=0--> vao created--> remove
					cleanVAO(geoName2StaticRender[geoName][texId].vbo,geoName2StaticRender[geoName][texId].vao);
					geoName2StaticRender[geoName][texId]=RenderSt(texId,vert,geometryType,shaderMode);
				}else{
					//1.1.1 if also contains texture and the number of vertex!=0--> vao no created--> just append
					geoName2StaticRender[geoName][texId].vertices.insert(geoName2StaticRender[geoName][texId].vertices.end(),vert.begin(),vert.end());
				}
			}else{
				geoName2StaticRender[geoName][texId]=RenderSt(texId,vert,geometryType,shaderMode);
			}
		}else{
			// 1.2 No yet in manager
			geoName2StaticRender[geoName][texId]=RenderSt(texId,vert,geometryType,shaderMode);
		}
		return true;
	}//

	void report(struct triangulateio *io, int markers, int reporttriangles, int reportneighbors, int reportsegments,
		int reportedges, int reportnorms)
	{
		int i, j;

		for (i = 0; i < io->numberofpoints; i++) {
			printf("Point %4d:", i);
			for (j = 0; j < 2; j++) {
				printf("  %.6g", io->pointlist[i * 2 + j]);
			}
			if (io->numberofpointattributes > 0) {
				printf("   attributes");
			}
			for (j = 0; j < io->numberofpointattributes; j++) {
				printf("  %.6g",
					io->pointattributelist[i * io->numberofpointattributes + j]);
			}
			if (markers) {
				printf("   marker %d\n", io->pointmarkerlist[i]);
			} else {
				printf("\n");
			}
		}
		printf("\n");

		if (reporttriangles || reportneighbors) {
			for (i = 0; i < io->numberoftriangles; i++) {
				if (reporttriangles) {
					printf("Triangle %4d points:", i);
					for (j = 0; j < io->numberofcorners; j++) {
						printf("  %4d", io->trianglelist[i * io->numberofcorners + j]);
					}
					if (io->numberoftriangleattributes > 0) {
						printf("   attributes");
					}
					for (j = 0; j < io->numberoftriangleattributes; j++) {
						printf("  %.6g", io->triangleattributelist[i *
							io->numberoftriangleattributes + j]);
					}
					printf("\n");
				}
				if (reportneighbors) {
					printf("Triangle %4d neighbors:", i);
					for (j = 0; j < 3; j++) {
						printf("  %4d", io->neighborlist[i * 3 + j]);
					}
					printf("\n");
				}
			}
			printf("\n");
		}

		if (reportsegments) {
			for (i = 0; i < io->numberofsegments; i++) {
				printf("Segment %4d points:", i);
				for (j = 0; j < 2; j++) {
					printf("  %4d", io->segmentlist[i * 2 + j]);
				}
				if (markers) {
					printf("   marker %d\n", io->segmentmarkerlist[i]);
				} else {
					printf("\n");
				}
			}
			printf("\n");
		}

		if (reportedges) {
			for (i = 0; i < io->numberofedges; i++) {
				printf("Edge %4d points:", i);
				for (j = 0; j < 2; j++) {
					printf("  %4d", io->edgelist[i * 2 + j]);
				}
				if (reportnorms && (io->edgelist[i * 2 + 1] == -1)) {
					for (j = 0; j < 2; j++) {
						printf("  %.6g", io->normlist[i * 2 + j]);
					}
				}
				if (markers) {
					printf("   marker %d\n", io->edgemarkerlist[i]);
				} else {
					printf("\n");
				}
			}
			printf("\n");
		}
	}//

    bool VBORenderManager::addStaticConvexPoly(QString geoName,std::vector<Vector3D>& pos,float zShift,bool inverseLoop,QString textureName,int shaderMode,QVector3D texScale,bool tesselate,QVector3D* color){

		//printf("-- addStaticConvexPoly\n");
		if(pos.size()<3){
			printf("ERROR: addStaticConvexPoly %d points\n",pos.size());
			return false;
		}
		/*if(pos.size()==3){
			std::vector<Vertex> verte;
			verte.resize(pos.size());
			for(int i=0;i<pos.size();i++)
				verte[i]=Vertex(pos[i].x(),pos[i].y(),pos[i].z(),0,0,0,0,0,1.0,pos[i].x(),pos[i].y(),0);
			addStaticGeometry(geoName,verte,textureName,GL_TRIANGLES,shaderMode);
			return true;
		}
		if(pos.size()==4){
			std::vector<Vertex> verte;
			verte.resize(3*2);//two triangles
			for(int i=0;i<3;i++)
				verte[i]=Vertex(pos[i].x(),pos[i].y(),pos[i].z(),0,0,0,0,0,1.0,pos[i].x(),pos[i].y(),0);
			
			verte[3]=Vertex(pos[2].x(),pos[2].y(),pos[2].z(),0,0,0,0,0,1.0,pos[2].x(),pos[2].y(),0);
			verte[4]=Vertex(pos[3].x(),pos[3].y(),pos[3].z(),0,0,0,0,0,1.0,pos[3].x(),pos[3].y(),0);
			verte[5]=Vertex(pos[0].x(),pos[0].y(),pos[0].z(),0,0,0,0,0,1.0,pos[0].x(),pos[0].y(),0);
			addStaticGeometry(geoName,verte,textureName,GL_TRIANGLES,shaderMode);
			return true;
		}*/
		///*struct triangulateio in, mid, out, vorout;
		//in.numberofpoints = pos.size();
		//in.numberofpointattributes = 0;
		//in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
		//for(int pN=0;pN<pos.size();pN++){
		//	in.pointlist[2*pN+0]=pos[pN].x();
		//	in.pointlist[2*pN+1]=pos[pN].y();
		//}
		///* Make necessary initializations so that Triangle can return a */
		///*   triangulation in `mid' and a voronoi diagram in `vorout'.  */

		//mid.pointlist = (REAL *) NULL;            /* Not needed if -N switch used. */
		///* Not needed if -N switch used or number of point attributes is zero: */
		//mid.pointattributelist = (REAL *) NULL;
		//mid.pointmarkerlist = (int *) NULL; /* Not needed if -N or -B switch used. */
		//mid.trianglelist = (int *) NULL;          /* Not needed if -E switch used. */
		///* Not needed if -E switch used or number of triangle attributes is zero: */
		//mid.triangleattributelist = (REAL *) NULL;
		//mid.neighborlist = (int *) NULL;         /* Needed only if -n switch used. */
		///* Needed only if segments are output (-p or -c) and -P not used: */
		//mid.segmentlist = (int *) NULL;
		///* Needed only if segments are output (-p or -c) and -P and -B not used: */
		//mid.segmentmarkerlist = (int *) NULL;
		//mid.edgelist = (int *) NULL;             /* Needed only if -e switch used. */
		//mid.edgemarkerlist = (int *) NULL;   /* Needed if -e used and -B not used. */

		///* Triangulate the points.  Switches are chosen to read and write a  */
		///*   PSLG (p), preserve the convex hull (c), number everything from  */
		///*   zero (z), assign a regional attribute to each element (A), and  */
		///*   produce an edge list (e), a Voronoi diagram (v), and a triangle */
		///*   neighbor list (n). */
		//printf("a\n");
		//triangulate("pczen", &in, &mid, &vorout);
		//printf("2\n");
		//report(&mid, 1, 1, 1, 1, 1, 0);
		//printf("3\n");
		/*if(pos.size()==3||pos.size()==4){
			printf("a\n");
			std::vector<Vertex> verte;
			QVector3D normal=QVector3D(0,0,1);
			if(color==0)
				color=new QVector3D(1,1,1);
			verte.push_back(Vertex(QVector3D(pos[0].x(),pos[0].y(),0.0f+zShift),*color,normal,QVector3D()));
			verte.push_back(Vertex(QVector3D(pos[1].x(),pos[1].y(),0.0f+zShift),*color,normal,QVector3D()));
			verte.push_back(Vertex(QVector3D(pos[2].x(),pos[2].y(),0.0f+zShift),*color,normal,QVector3D()));
			if(pos.size()==4){
				verte.push_back(Vertex(QVector3D(pos[0].x(),pos[0].y(),0.0f+zShift),*color,normal,QVector3D()));
				verte.push_back(Vertex(QVector3D(pos[2].x(),pos[2].y(),0.0f+zShift),*color,normal,QVector3D()));
				verte.push_back(Vertex(QVector3D(pos[3].x(),pos[3].y(),0.0f+zShift),*color,normal,QVector3D()));
			}
			addStaticGeometry(geoName,verte,textureName,GL_TRIANGLES,shaderMode);
			addStaticGeometry(geoName,verte,textureName,GL_TRIANGLES,shaderMode);
			printf("z\n");
			return true;
		}*/
		//printf("-- addStaticConvexPoly2\n");
		struct triangulateio in, mid, out, vorout;

		/* Define input points. */

		in.numberofpoints = 4;
		in.numberofpointattributes = 0;

		in.numberofpoints = pos.size();
		//printf("--- pos.size() %d\n", pos.size());
		//std::vector<REAL> pointlist(in.numberofpoints * 2);
		//in.pointlist = &pointlist[0];
		in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
		
		for(int pN=0;pN<pos.size();pN++){
			in.pointlist[2*pN+0]=pos[pN].x();
			in.pointlist[2*pN+1]=pos[pN].y();
			//printf("%f %f\n",pos[pN].x(),pos[pN].y());
		}

		//std::vector<int> pointmarkerlist(in.numberofpoints);
		//in.pointmarkerlist = &pointmarkerlist[0];
		in.pointmarkerlist = (int *) malloc(in.numberofpoints * sizeof(int));

		for(int pN=0;pN<pos.size();pN++){
			in.pointmarkerlist[pN] = 1;
		}

		/*in.numberofsegments = 0;
		in.numberofholes = 0;
		in.numberofregions = 1;
		in.regionlist = (REAL *) malloc(in.numberofregions * 4 * sizeof(REAL));
		in.regionlist[0] = 0.5;
		in.regionlist[1] = 5.0;
		in.regionlist[2] = 7.0;            // Regional attribute (for whole mesh). 
		in.regionlist[3] = 0.1;          // Area constraint that will not be used. 
		*/
		//printf("-- addStaticConvexPoly3\n");
		//printf("Input point set:\n\n");
		//report(&in, 1, 0, 0, 0, 0, 0);

		/* Make necessary initializations so that Triangle can return a */
		/*   triangulation in `mid' and a voronoi diagram in `vorout'.  */

		mid.pointlist = (REAL *) NULL;            /* Not needed if -N switch used. */
		/* Not needed if -N switch used or number of point attributes is zero: */
		mid.pointattributelist = (REAL *) NULL;
		mid.pointmarkerlist = (int *) NULL; /* Not needed if -N or -B switch used. */
		mid.trianglelist = (int *) NULL;          /* Not needed if -E switch used. */
		/* Not needed if -E switch used or number of triangle attributes is zero: */
		mid.triangleattributelist = (REAL *) NULL;
		mid.neighborlist = (int *) NULL;         /* Needed only if -n switch used. */
		/* Needed only if segments are output (-p or -c) and -P not used: */
		mid.segmentlist = (int *) NULL;
		/* Needed only if segments are output (-p or -c) and -P and -B not used: */
		mid.segmentmarkerlist = (int *) NULL;
		mid.edgelist = (int *) NULL;             /* Needed only if -e switch used. */
		mid.edgemarkerlist = (int *) NULL;   /* Needed if -e used and -B not used. */

		vorout.pointlist = (REAL *) NULL;        /* Needed only if -v switch used. */
		/* Needed only if -v switch used and number of attributes is not zero: */
		vorout.pointattributelist = (REAL *) NULL;
		vorout.edgelist = (int *) NULL;          /* Needed only if -v switch used. */
		vorout.normlist = (REAL *) NULL;         /* Needed only if -v switch used. */

		/* Triangulate the points.  Switches are chosen to read and write a  */
		/*   PSLG (p), preserve the convex hull (c), number everything from  */
		/*   zero (z), assign a regional attribute to each element (A), and  */
		/*   produce an edge list (e), a Voronoi diagram (v), and a triangle */
		/*   neighbor list (n).                                              */

		//triangulate("pcz", &in, &mid, (struct triangulateio *) NULL);

		//printf("Initial triangulation:\n\n");
		//report(&mid, 1, 1, 0, 1, 0, 0);
		//printf("-- addStaticConvexPoly31\n");
		try{

			if (tesselate == true){// from last code "p" has been removed!!
                triangulate("ca500.0zQ", &in, &mid, (struct triangulateio *) NULL);//Q quiet
			}
            else{
                triangulate("czQ", &in, &mid, (struct triangulateio *) NULL);
			}
		}catch(...){ printf("Exception occurred: triangulate\n"); }
		//printf("-- addStaticConvexPoly4\n");
		//printf("Initial triangulation:\n\n");
		//report(&mid, 1, 1, 0, 0, 0, 0);
		//exit(0);
		///////////////////////
		// VERT
		//printf("VERT\n");
		std::vector<Vertex> verte;
		verte.resize(3*mid.numberoftriangles);
		for (int i = 0; i <mid.numberoftriangles; i++) {
			//printf("Triangle %4d points %d:", i,mid.numberofcorners);
			for (int j = 0; j < mid.numberofcorners; j++) {
				int indexT=(mid.trianglelist[i * mid.numberofcorners + j]);
				//printf(" %d ",indexT);
				//printf(" %f %f | ",mid.pointlist[indexT * 2 + 0],mid.pointlist[indexT * 2 + 1]);
				QVector3D pos(mid.pointlist[indexT * 2 + 0],mid.pointlist[indexT * 2 + 1],zShift);
				verte[i*3+j]=Vertex(pos.x(),pos.y(),pos.z(),0,0,0,0,0,1.0,pos.x(),pos.y(),0);
			}
			/*for (int j = 0; j < mid.numberofcorners; j++) {
				int indexT=(mid.trianglelist[i * mid.numberofcorners + j]);
				//printf("  %4d", mid.trianglelist[i * io->numberofcorners + j]);
				QVector3D pos(mid.pointlist[indexT * 2 + 0],mid.pointlist[indexT * 2 + 1],zShift);
				verte.push_back(Vertex(pos,QVector3D(),QVector3D(0,0,1),pos));
			}*/
			//printf("\n");

		}
		//printf("-- addStaticConvexPoly5\n");
		addStaticGeometry(geoName,verte,textureName,GL_TRIANGLES,shaderMode);
		//printf("-- addStaticConvexPoly6\n");
		return true;
	}//


	using namespace boost::polygon::operators;

	bool VBORenderManager::addStaticGeometry2(QString geoName,std::vector<QVector3D>& pos,float zShift,bool inverseLoop,QString textureName,GLenum geometryType,int shaderMode){
		if(pos.size()<3){
			return false;
		}
		PolygonSetP polySet;
		polygonP tempPolyP;

		std::vector<pointP> vP;
		vP.resize(pos.size());
		float minX=FLT_MAX,minY=FLT_MAX;
		float maxX=-FLT_MAX,maxY=-FLT_MAX;

		for(int pN=0;pN<pos.size();pN++){
			vP[pN]=boost::polygon::construct<pointP>(pos[pN].x(),pos[pN].y());
			minX=std::min<float>(minX,pos[pN].x());
			minY=std::min<float>(minY,pos[pN].y());
			maxX=std::max<float>(maxX,pos[pN].x());
			maxY=std::max<float>(maxY,pos[pN].y());
		}

		boost::polygon::set_points(tempPolyP,vP.begin(),vP.end());
		polySet+=tempPolyP;
		std::vector<polygonP> allP;
		boost::polygon::get_trapezoids(allP,polySet);
		
		std::vector<Vertex> vert;

		for(int pN=0;pN<allP.size();pN++){
			//glColor3ub(qrand()%255,qrand()%255,qrand()%255);
			boost::polygon::polygon_with_holes_data<double>::iterator_type itPoly=allP[pN].begin();
			std::vector<QVector3D> points;
			std::vector<QVector3D> texP;
			while(itPoly!=allP[pN].end()){
				pointP cP=*itPoly;
				if(inverseLoop==false)
					points.push_back(QVector3D(cP.x(),cP.y(),pos[0].z()+zShift));
				else
					points.insert(points.begin(),QVector3D(cP.x(),cP.y(),pos[0].z()+zShift));

				//if(texZeroToOne==true){
					texP.push_back(QVector3D((cP.x()-minX)/(maxX-minX),(cP.y()-minY)/(maxY-minY),0.0f));
				//}else{
				//	texP.push_back(QVector3D((cP.x()-minX)*texScale.x(),(cP.y()-minY)*texScale.y(),0.0f));
				//}
				itPoly++;
			}
			/*if(points.size()==4){//last vertex repited
				addTexTriang(texInd,points,texP,col,norm);
			}
			if(points.size()==5){
				addTexQuad(texInd,points,texP,col,norm);

			}
			if(points.size()==6){
				//addTexQuad(texInd,std::vector<QVector3D>(&points[0],&points[3]),std::vector<QVector3D>(&texP[0],&texP[3]),col,norm);

				addTexQuad(texInd,points,texP,col,norm);
				//addTexTriang(texInd,std::vector<QVector3D>(&points[3],&points[6]),std::vector<QVector3D>(&texP[3],&texP[6]),col,norm);
				//addTexTriang(texInd,std::vector<QVector3D>(&points[4],&points[6]),std::vector<QVector3D>(&texP[4],&texP[6]),col,norm);
			}*/
			vert.push_back(Vertex(points[0],QVector3D(),QVector3D(0,0,1),texP[0]));
			vert.push_back(Vertex(points[1],QVector3D(),QVector3D(0,0,1),texP[1]));
			vert.push_back(Vertex(points[2],QVector3D(),QVector3D(0,0,1),texP[2]));
			vert.push_back(Vertex(points[3],QVector3D(),QVector3D(0,0,1),texP[3]));
		}

		return addStaticGeometry(geoName,vert,textureName,geometryType,shaderMode);
	}//

	bool VBORenderManager::removeStaticGeometry(QString geoName,bool warning){
		if(geoName2StaticRender.contains(geoName)){
			// iterate and remove
			renderGrid::iterator i;
			for (i = geoName2StaticRender[geoName].begin(); i != geoName2StaticRender[geoName].end(); ++i){
				cleanVAO(geoName2StaticRender[geoName][i.key()].vbo,geoName2StaticRender[geoName][i.key()].vao);
			}
			geoName2StaticRender[geoName].clear();
			geoName2StaticRender.remove(geoName);
		}else{
			if(warning==true)
				printf("ERROR: Remove Geometry %s but it did not exist\n",geoName.toUtf8().constData());
			return false;
		}

		return true;
	}//

	void VBORenderManager::renderStaticGeometry(QString geoName){

		if(geoName2StaticRender.contains(geoName)){
			// iterate and remove
			renderGrid::iterator i;
			for (i = geoName2StaticRender[geoName].begin(); i != geoName2StaticRender[geoName].end(); ++i){
				renderVAO(i.value(),false);
			}
		}else{
			printf("ERROR: Render Geometry %s but it did not exist\n",geoName.toUtf8().constData());
			return;
		}
	}//

	///////////////////////////////////////////////////////////////////
	// GRID
	bool VBORenderManager::addGridGeometry(QString geoName,std::vector<Vertex>& vert,QString textureName){
		return false;
	}//
	bool VBORenderManager::removeGridGeometry(QString geoName){
		return false;
	}//
	
	void VBORenderManager::renderGridGeometry(QString geoName){
		
	}//
	///////////////////////////////////////////////////////////////////
	// MODEL
	//QHash<QString,std::vector<ModelSpec>> nameToVectorModels;
	void VBORenderManager::addStreetElementModel(QString name,ModelSpec mSpec){
		nameToVectorModels[name].push_back(mSpec);
	}//
	void VBORenderManager::renderAllStreetElementName(QString name){
		for(int i=0;i<nameToVectorModels[name].size();i++){
			VBOModel_StreetElements::renderOneStreetElement(program,nameToVectorModels[name][i]);
		}
		printf("name %s --> %d\n",name.toUtf8().constData(),nameToVectorModels[name].size());
	}//
	void VBORenderManager::removeAllStreetElementName(QString name){
		nameToVectorModels[name].clear();
	}

	///////////////////////////////////////////////////////////////////
	// LAYERS
	void VBORenderManager::activeLayersMode(bool activeLayers,int layerActive){


	}
	


} // namespace LC
