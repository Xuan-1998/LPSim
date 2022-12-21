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

//#include "cudaTrafficDesigner.h"
//#include "cudaTrafficMCMC.h"
////#include "client_main.h"
//
//#define WIDTH_WATER 4000.0f
//#define DESIGNER_DEBUG 0
//
//namespace LC {
//
//	//extern ClientMain *clientMain;
//
//	CUDATrafficDesigner::CUDATrafficDesigner(): QObject(NULL){
//		designerTrafficSimulator.simRoadGraph=0;
//		currentBestThread=-1;
//		currentBestScore=FLT_MAX;
//	}//
//	CUDATrafficDesigner::~CUDATrafficDesigner(){
//	}//
//
//
//
//
//	////////////////////////////////////
//	// DRAWING
//	void CUDATrafficDesigner::drawDrawingCircle(){
//		float radius=LC::misctools::Global::global()->cuda_drawing_rad;
//		const int circle_points = 100;
//		double angle;
//		double posX, posY, posZ;
//		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//		glEnable (GL_BLEND);
//		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//		switch (LC::misctools::Global::global()->cuda_drawing_selected){
//		case 0:
//			glColor4f(0.0, 0.0, 0.0,0.8f);
//			break;
//		case 1:
//			glColor4ub(0x17,0xe5,0x17,0xFF);
//			break;
//		case 2:
//			glColor4ub(0xe5,0xde,0x17,0xFF);//e5de17
//			break;
//		case 3:
//			glColor4ub(0xe5,0x17,0x17,0xFF);//e51717
//			break;
//		case 4:
//			glColor4ub(0xff,0xff,0xff,0xFF);//remove
//			break;
//		}
//		if(float radius=LC::misctools::Global::global()->cuda_drawing_selected==0){
//
//		}
//
//		glLineWidth(5.0);
//		glBegin(GL_POLYGON);
//		float x=mousePos.x();
//		float y=mousePos.y();
//		float z=mousePos.z();
//		posZ=10.0f;
//		for (int i = 0; i < circle_points; i++) {
//			angle = 2*M_PI*i/circle_points;
//			posX = x+radius*cos(angle);
//			posY = y+radius*sin(angle);
//			glVertex3f(posX, posY, posZ);
//
//		}
//		glEnd();
//		glColor4f(0.1, 0.1, 0.1,1.0f);
//		glDisable (GL_BLEND);
//		glLineWidth(1.0);
//		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//
//	}//
//
//	bool polyIntersectPoly(LC::CUDATrafficDesigner::polygon& p1,LC::CUDATrafficDesigner::polygon& p2){
//		return boost::geometry::intersects(p1, p2);
//	}//
//
//	void CUDATrafficDesigner::updateUserSelection(QVector3D& mouse3DPos){
//		if(DESIGNER_DEBUG)printf("updateUserSelection %f %f %f\n",mouse3DPos.x(),mouse3DPos.y(),mouse3DPos.z());
//		// 1. Create new circle
//		const int circle_points = 20;
//		float radius=LC::misctools::Global::global()->cuda_drawing_rad;
//		float posX,posY,posZ,angle;
//		float x=mousePos.x();
//		float y=mousePos.y();
//		float z=mousePos.z();
//		//posZ=10.0f;
//		polygon tempPoly;
//		for (int i = circle_points-1; i >=0; i--) {
//			angle = 2*M_PI*i/circle_points;
//			posX = x+radius*cos(angle);
//			posY = y+radius*sin(angle);
//			boost::geometry::append(tempPoly,mPoint (posX,posY) );
//		}
//		boost::geometry::append(tempPoly,tempPoly.outer()[0] );
//
//		// 2. Put it in the correct group
//		std::vector<polygon>* vec;
//		std::vector<std::vector<polygon>*> otherVec;
//		switch (LC::misctools::Global::global()->cuda_drawing_selected){
//		case 0:
//			vec=&blPolys;
//			otherVec.push_back(&ltPolys);
//			otherVec.push_back(&mtPolys);
//			otherVec.push_back(&htPolys);
//			break;
//		case 1:
//			vec=&ltPolys;
//			otherVec.push_back(&blPolys);
//			otherVec.push_back(&mtPolys);
//			otherVec.push_back(&htPolys);
//			break;
//		case 2:
//			vec=&mtPolys;
//			otherVec.push_back(&blPolys);
//			otherVec.push_back(&ltPolys);
//			otherVec.push_back(&htPolys);
//			break;
//		case 3:
//			vec=&htPolys;
//			otherVec.push_back(&blPolys);
//			otherVec.push_back(&ltPolys);
//			otherVec.push_back(&mtPolys);
//			break;
//		case 4:
//			vec=0;
//			otherVec.push_back(&blPolys);
//			otherVec.push_back(&ltPolys);
//			otherVec.push_back(&mtPolys);
//			otherVec.push_back(&htPolys);
//			break;
//		}
//		// 3. Diference of new polygon with the other sets (3 before 4 so poly is simpler)
//		if(DESIGNER_DEBUG)printf("4. Diference of new polygon with the other sets\n");
//		for(int oV=0;oV<otherVec.size();oV++){
//			for(int oVI=0;oVI<otherVec[oV]->size();oVI++){
//				if(polyIntersectPoly(tempPoly,otherVec[oV]->at(oVI))){
//					std::vector<polygon> pl;
//					boost::geometry::difference(otherVec[oV]->at(oVI),tempPoly, pl);
//					if(pl.size()==0){//difference is zero--> remove poly
//						otherVec[oV]->erase(otherVec[oV]->begin()+oVI);
//						oVI--;
//					}else{
//						if(pl.size()==1){// replace poly for diference
//							otherVec[oV]->at(oVI)=pl[0];
//						}else{
//							// it created more poly, copy first and add rest to the end
//							otherVec[oV]->at(oVI)=pl[0];
//							for(int oP=1;oP<pl.size();oP++){
//								otherVec[oV]->push_back(pl[oP]);
//							}
//						}
//					}
//				}
//			}
//		}
//		if(vec==0)//case 4 (just remove)
//			return;
//		// 4. Check if intersects with any poly and put them together
//		for(int p=0;p<vec->size();p++){
//			if(polyIntersectPoly(tempPoly,vec->at(p))){
//				//Intersect so union
//				std::vector<polygon> pl;
//				boost::geometry::union_(tempPoly,vec->at(p), pl);
//				//float zValue=10.0f;//tempPoly[0].z();
//				tempPoly.clear();
//				QVector3D tmpPt;
//				if(pl.size()!=1){
//					printf("ERROR: they should intersect pl.size() %d\n",pl.size());
//				}
//				tempPoly=pl[0];
//
//				vec->erase(vec->begin()+p);
//				p--;
//				//printf("size %d\n",pl.size());
//			}
//
//		}
//		vec->push_back(tempPoly);
//
//	}//
//
//	void renderNonConvexDes(LC::CUDATrafficDesigner::polygon& contour,
//			QVector3D& myNormal,QVector3D& color,uchar alpha)
//		{
//			const float height=0.0f;
//			glColor4ub(color.x(),color.y(),color.z(),alpha);
//			//Render inside fill
//			if((contour.outer()).size() == 3){
//				glBegin(GL_TRIANGLES);
//				for(size_t i=0; i<(contour.outer()).size(); i++){
//					glNormal3f(myNormal.x(), myNormal.y(), myNormal.z());
//					glVertex3f((contour.outer())[i].x(), (contour.outer())[i].y(), height);//(contour.outer())[i].z());
//				}
//				glEnd();
//			} else if((contour.outer()).size() == 4){
//				glBegin(GL_QUADS);
//				for(int i=0; i<(contour.outer()).size(); i++){
//					glNormal3f(myNormal.x(), myNormal.y(), myNormal.z());
//					glVertex3f((contour.outer())[i].x(), (contour.outer())[i].y(),height);// (contour.outer())[i].z());
//				}
//				glEnd();
//			} else {
//
//				// create tessellator
//				GLUtesselator *tess = gluNewTess();
//
//				double *vtxData = new double[3*(contour.outer()).size()];
//				for(size_t i=0; i<(contour.outer()).size(); i++){
//					vtxData[3*i]=(contour.outer())[i].x();
//					vtxData[3*i+1]=(contour.outer())[i].y();
//					vtxData[3*i+2]=height;//(contour.outer())[i].z();
//				}
//
//				// register callback functions
//				gluTessCallback(tess, GLU_TESS_BEGIN,
//					(void (__stdcall *)(void))glBegin);
//				gluTessCallback(tess, GLU_TESS_VERTEX,
//					(void (__stdcall *)(void))glVertex3dv);
//				gluTessCallback(tess, GLU_TESS_END, glEnd);
//
//				// describe non-convex polygon
//				gluTessBeginPolygon(tess, NULL);
//
//				// contour
//				gluTessBeginContour(tess);
//
//				for(size_t i=0; i<(contour.outer()).size(); i++){
//					//HACK
//					glNormal3f(myNormal.x(), myNormal.y(), fabs(myNormal.z()));
//					gluTessVertex(tess, &vtxData[3*i], &vtxData[3*i]);
//				}
//				gluTessEndContour(tess);
//				gluTessEndPolygon(tess);
//
//				// delete tessellator after processing
//				gluDeleteTess(tess);
//
//				delete [] vtxData;
//			}
//
//			glColor4f(color.x()/255.0f,color.y()/255.0f,color.z()/255.0f,1.0f);
//			glBegin(GL_LINE_LOOP);
//			for(size_t i=0; i<(contour.outer()).size(); i++){
//				glVertex3f((contour.outer())[i].x(),(contour.outer())[i].y(),height);
//			}
//			glEnd();
//
//		}//
//
//
//	void CUDATrafficDesigner::renderRegions(){
//		glEnable (GL_BLEND);
//		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//		uchar alpha=125;
//		for(int b=0;b<blPolys.size();b++){
//			renderNonConvexDes(blPolys[b],QVector3D(0,0,1),QVector3D(0,0,0),alpha);
//		}
//		for(int b=0;b<ltPolys.size();b++){
//			renderNonConvexDes(ltPolys[b],QVector3D(0,0,1),QVector3D(0x17,0xe5,0x17),alpha);
//		}
//
//		glColor4ub(0xe5,0xde,0x17,alpha);//e5de17
//		for(int b=0;b<mtPolys.size();b++){
//			renderNonConvexDes(mtPolys[b],QVector3D(0,0,1),QVector3D(0xe5,0xde,0x17),alpha);
//		}
//		for(int b=0;b<htPolys.size();b++){
//			renderNonConvexDes(htPolys[b],QVector3D(0,0,1),QVector3D(0xe5,0x17,0x17),alpha);
//		}
//
//		glColor4f(0.1, 0.1, 0.1,1.0f);
//		glDisable (GL_BLEND);
//
//	}//
//
//	// DESIGNER
//	void setVisitValue(RoadGraph& inRoadGraph,uchar value){
//		RoadGraph::roadGraphEdgeIter_BI ei, eiEnd;
//		for(boost::tie(ei, eiEnd) = boost::edges(inRoadGraph.myRoadGraph_BI);
//			ei != eiEnd; ++ei)
//		{
//			inRoadGraph.myRoadGraph_BI[*ei].cuda_design_visited=value;
//		}
//	}//
//
//	void bsfFromEdge(RoadGraph& inRoadGraph,LC::RoadGraph::roadGraphEdgeDesc_BI& edge,int design_state,int currentDistance,int maxDistance){
//
//		// check if too far or already visited
//		if(currentDistance>=maxDistance)
//			return;
//		// check not block
//		if(inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==0)
//			return;
//		// check not other area
//		if(design_state==1&&(inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==2||inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==3))
//			return;
//		if(design_state==2&&(inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==1||inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==3))
//			return;
//		if(design_state==3&&(inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==1||inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==2))
//			return;
//		// check not same area type but reached
//		if(design_state==inRoadGraph.myRoadGraph_BI[edge].cuda_design_state && currentDistance>0)
//			return;
//		// check if not visited same region
//		if(inRoadGraph.myRoadGraph_BI[edge].cuda_design_visited==0)
//			return;
//		// check if not further
//		if(inRoadGraph.myRoadGraph_BI[edge].cuda_design_visited!=0xFFFF&&currentDistance>=(inRoadGraph.myRoadGraph_BI[edge].cuda_design_visited%10))
//			return;
//		printf("state %d cDist %d, maxD %d\n",design_state,currentDistance,maxDistance);
//		bool sameState=inRoadGraph.myRoadGraph_BI[edge].cuda_design_state==design_state;
//
//
//		if(currentDistance==0&&sameState==true){
//			inRoadGraph.myRoadGraph_BI[edge].cuda_design_visited=0;
//		}else{
//		//if(currentDistance>0||sameState==false){
//			currentDistance++;
//			inRoadGraph.myRoadGraph_BI[edge].cuda_design_state=design_state*10+currentDistance;//10|30+distance
//		}
//
//		// 2. expand for both sides
//		LC::RoadGraph::roadGraphVertexDesc_BI ver[2];
//		ver[0]=boost::source(edge, inRoadGraph.myRoadGraph_BI);
//		ver[1]=boost::target(edge, inRoadGraph.myRoadGraph_BI);
//		//mark twin edge as well
//		std::pair<RoadGraph::roadGraphEdgeDesc_BI, bool> e0_pair=boost::edge(ver[1],ver[0],inRoadGraph.myRoadGraph_BI);
//		if(e0_pair.second==false){
//			printf("Error: Twin edge does not exits\n");
//		}else{
//			inRoadGraph.myRoadGraph_BI[e0_pair.first].cuda_design_visited=inRoadGraph.myRoadGraph_BI[edge].cuda_design_visited;//same state than twin
//			inRoadGraph.myRoadGraph_BI[e0_pair.first].cuda_design_state=inRoadGraph.myRoadGraph_BI[edge].cuda_design_state;
//		}
//		//check that we are not too far
//		if(currentDistance>=maxDistance)
//			return;
//		RoadGraph::out_roadGraphEdgeIter_BI Oei, Oei_end;
//		for(int verN=0;verN<2;verN++){
//			for(boost::tie(Oei, Oei_end) = boost::out_edges(ver[verN],inRoadGraph.myRoadGraph_BI); Oei != Oei_end; ++Oei){
//				bsfFromEdge(inRoadGraph,*Oei,design_state,currentDistance,maxDistance);
//			}
//		}
//	}//
//
//
//
//
//	void CUDATrafficDesigner::changeNetwork(CUDATrafficSimulator& inTrafficSimulator,int _numPasses,int _numChains,int _numSteps,CUDADesignOptions designOptions,PlaceTypesMainClass* currentPlaceTypes){
//		printf(">>changeNetwork\n");
//		clientMain->ui.cudaDesignChangeNetworkButton->setEnabled(false);
//		////
//		currentBestScore=FLT_MAX;
//		currentMinCost=FLT_MAX;
//		// copy input simulator to design
//		designerTrafficSimulator=inTrafficSimulator;
//		designerTrafficSimulator.simRoadGraph=new RoadGraph(*inTrafficSimulator.simRoadGraph);
//
//		// clear modified edges
//		RoadGraph::roadGraphEdgeIter_BI ei, eiEnd;
//		for(boost::tie(ei, eiEnd) = boost::edges(designerTrafficSimulator.simRoadGraph->myRoadGraph_BI);
//			ei != eiEnd; ++ei)
//		{
//			designerTrafficSimulator.simRoadGraph->myRoadGraph_BI[*ei].cuda_design_modified=0x0000;
//		}
//
//		std::vector<std::vector<polygon>*> allVec;
//		allVec.push_back(&blPolys);
//		allVec.push_back(&ltPolys);
//		allVec.push_back(&mtPolys);
//		allVec.push_back(&htPolys);
//
//		////////////////////////////////////
//		// 3. Call MCMC
//		int idealNumThreads=QThread::idealThreadCount()-3;//not to use the 100%
//		int numTheads=std::min<int>(_numChains,idealNumThreads);
//		int numChainsToRunPerThread=std::ceil((float)_numChains/ numTheads);
//		printf("Start MCMC with %d threads and %d chains per thread\n",numTheads,numChainsToRunPerThread);
//
//		threads.resize(numTheads);
//		chains.resize(numTheads);
//		//threadSimulators.resize(numTheads);
//		float temperature=0;
//		int numSteps=_numSteps;
//		int indexToEvaluate=LC::misctools::Global::global()->cuda_current_time_slider-70;
//		//float tempsArray[] = {80.0f, 160.0f, 320.0f, 640.0f, 1280.0f, 2560.0f};// Carlos temperatures
//		float tempsArray[] = {4.0f,16.0f, 64.0f, 256.0f};
//		//const float goalScore=1.0f;
//		findBestCost=false;
//		//scoreGoal=0.5f;
//		scoreGoal=4.0f;
//
//		for(int i=0;i<numTheads;i++){
//			threads[i] = new QThread;
//			chains[i] = new CUDATrafficMCMC();
//			//chains[i]->name="Main";
//			//threadSimulators[i]=new CUDATrafficSimulator(designerTrafficSimulator);
//			//threadSimulators[i]->simRoadGraph=new RoadGraph(*designerTrafficSimulator.simRoadGraph);
//			//designerTrafficSimulator=inTrafficSimulator;
//			//designerTrafficSimulator.simRoadGraph=new RoadGraph(*inTrafficSimulator.simRoadGraph);
//			temperature=tempsArray[i%4];
//			currentBestThread=0;
//			chains[i]->initChain(i,numChainsToRunPerThread,&designerTrafficSimulator,temperature,numSteps,_numPasses,indexToEvaluate,designOptions,allVec,currentPlaceTypes);
//			chains[i]->initMinCost(scoreGoal);
//			chains[i]->moveToThread(threads[i]);
//			c
//			connect(chains[i], SIGNAL(newResult(int)), this, SLOT(newResult(int)));
//			if(findBestCost==true){
//				connect(threads[i], SIGNAL(started()), chains[i], SLOT(processMinCost()));
//			}else{
//				connect(threads[i], SIGNAL(started()), chains[i], SLOT(process()));
//			}
//			connect(chains[i], SIGNAL(finished()), threads[i], SLOT(quit()));
//			//connect(chains[i], SIGNAL(finished()), this, SLOT(thredEnds()));
//			//connect(chain, SIGNAL(finished()), chain, SLOT(deleteLater()));
//			//connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
//			printf("T[%d]: About to start\n",i);
//			threads[i]->start();
//		}
//		clientMain->ui.progressBar->show();
//			/*printf("T: Wait\n");
//
//			while(true){
//				// check if all finished
//				bool finished=true;
//				for(int i=0;i<numTheads;i++){
//					if(threads[i]->isRunning()){
//						finished=false;
//						break;
//					}
//				}
//				// one not finished, wait
//				if(finished==true)break;
//				printf(".");
//				QMutex mutex;
//				mutex.lock();
//				QWaitCondition waitCondition;
//				QCoreApplication::processEvents();
//				waitCondition.wait(&mutex, 200);//ms
//				QCoreApplication::processEvents();
//				mutex.unlock();
//			}
//			///!!
//			//chains[0]->moveToThread(QThread::currentThread());
//			printf("---->%s\n",chains[0]->name.toLatin1().constData());
//			//clientMain->cudaTrafficDesigner.designerTrafficSimulator=chains[0]->getSimulator();
//			//
//			//clientMain->ui.progressBar->hide();
//		//printf("T: Done\n");
//		//allVec.clear();
//
//
//
//		printf("<<changeNetwork\n");
//
//	}//
//
//	void CUDATrafficDesigner::errorString(QString err){
//		printf("ERROR THREAD: %s\n",err.toLatin1().constData());
//	}//
//
//	/*void CUDATrafficDesigner::thredEnds(){
//
//	}//*/
//
//
//
//	void CUDATrafficDesigner::newResult(int threadNum){
//		if(threadNum>=0){
//			//printf("[T%d] -->New result %d findBestCost %d\n",threadNum,findBestCost);
//			// BEST COST
//			if(findBestCost==true){
//				if((currentBestScore>scoreGoal&&currentBestScore>chains[threadNum]->currentScore)||
//					(currentBestScore<=scoreGoal&&chains[threadNum]->currentScore<=scoreGoal&&chains[threadNum]->minCost<currentMinCost)){
//					printf("NR: [T%d] C %f %f S %f %f\n",threadNum,currentMinCost,chains[threadNum]->currCost,currentBestScore,chains[threadNum]->currentScore);
//					currentMinCost=chains[threadNum]->minCost;
//					currentBestScore=chains[threadNum]->currentScore;
//					currentBestThread=threadNum;
//					clientMain->mGLWidget_3D_BottomRight->updateGL();
//					//clientMain->ui.infoLabel->setText("Score["+QString::number(currentBestThread)+"]: "+QString::number(currentBestScore,'g',2));
//					clientMain->ui.infoLabel->setText("T["+QString::number(currentBestThread)+"]-> C:"+QString::number(currentMinCost,'g',2)+" S: "+QString::number(currentBestScore,'g',2));
//				}
//				if(currentMinCost==0){//best possible score--> terminate other threads
//					for(int i=0;i<threads.size();i++){
//						//threads[i]->terminate();
//						chains[i]->done=true;
//					}
//				}
//
//
//			}else{
//			// BEST OPTIMIZATION
//				if(currentBestScore>chains[threadNum]->currentScore){
//					printf("Improved Old Score %f New Score %f",currentBestScore,chains[threadNum]->currentScore);
//					currentBestScore=chains[threadNum]->currentScore;
//					currentBestThread=threadNum;
//					clientMain->mGLWidget_3D_BottomRight->updateGL();
//					clientMain->ui.infoLabel->setText("Score["+QString::number(currentBestThread)+"]: "+QString::number(currentBestScore,'g',2));
//				}
//				if(currentBestScore==0){//best possible score--> terminate other threads
//					for(int i=0;i<threads.size();i++){
//						//threads[i]->terminate();
//						chains[i]->done=true;
//					}
//				}
//			}
//		}else{// just check
//			//printf("***Check end %d\n",threadNum);
//			bool finished=true;
//			for(int i=0;i<threads.size();i++){
//				if(chains[i]->done==false){
//					finished=false;
//					break;
//				}
//			}
//			if(finished==true){
//				clientMain->ui.progressBar->hide();
//				clientMain->ui.cudaDesignCopyResultButton->setEnabled(true);
//				printf("T: Done\n");
//				clientMain->ui.cudaDesignChangeNetworkButton->setEnabled(true);
//			}
//		}
//
//	}//
//
//	void CUDATrafficDesigner::copyResultToLeft(){
//		//clientMain->cudaTrafficDesigner.threadSimulators[currentBestThread]
//
//		clientMain->mGLWidget_3D->cudaTrafficSimulator=*chains[currentBestThread]->bestSimulationState;
//		clientMain->cg.roadGraph.myRoadGraph_BI.clear();
//		clientMain->cg.roadGraph=*chains[currentBestThread]->bestSimulationState->simRoadGraph;//copy
//		// layers
//		clientMain->mGLWidget_3D->infoLayers=chains[currentBestThread]->bestSimulationState->simPeopleJobInfoLayers;
//		clientMain->mGLWidget_3D->infoLayers.layers[0].updateImage();
//		clientMain->mGLWidget_3D->infoLayers.layers[1].updateImage();
//		//make sure that the density is used
//		if(clientMain->ui.cudaUseDensityCheckBox->isChecked()==false){
//			clientMain->ui.cudaUseDensityCheckBox->setChecked(true);
//		}
//
//	}//
//
//	void CUDATrafficDesigner::stopDesigner(){
//		for(int i=0;i<threads.size();i++){
//			chains[i]->done=true;
//		}
//	}//
//}


