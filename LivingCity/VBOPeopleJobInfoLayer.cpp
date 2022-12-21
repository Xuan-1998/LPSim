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

/************************************************************************************************
*		@desc Object that contains and holds layers info
*		@author igarciad
************************************************************************************************/


#include "VBOPeopleJobInfoLayer.h"
#include "VBORenderManager.h"
#include "global.h"

#define DEBUG_LAYERS 0
namespace LC {

	PeopleJobInfoLayers::PeopleJobInfoLayers(){
		initialized=false;
	}
	PeopleJobInfoLayers::~PeopleJobInfoLayers(){

	}


	void PeopleJobInfoLayers::updateLayer(int layerNum,QVector3D& mousePos,float change){
		if(DEBUG_LAYERS)printf("**update layer %d\n",layerNum);
		// update amplitude value
		int indexClosest=-1;
		float minDist=FLT_MAX;
		for(int mP=0;mP<samplePosition.size();mP++){
			float cDist=(samplePosition[mP]-mousePos).lengthSquared();
			if(minDist>cDist){
				minDist=cDist;
				indexClosest=mP;
			}
		}
		if(DEBUG_LAYERS)printf("change %f\n",change);
		//change/=100.0f;//0-1
		if(DEBUG_LAYERS)printf("Old val %f\n",layers[layerNum].amplitudes[indexClosest]);
		layers[layerNum].amplitudes[indexClosest]+=change;
		if(DEBUG_LAYERS)printf("New val %f\n",layers[layerNum].amplitudes[indexClosest]);
		//if(layers[layerNum].amplitudes[indexClosest]>1.0f)layers[layerNum].amplitudes[indexClosest]=1.0f;
		if(layers[layerNum].amplitudes[indexClosest]<0.0f)
			layers[layerNum].amplitudes[indexClosest]=0.0f;
		if(layers[layerNum].amplitudes[indexClosest]>100.0f)
			layers[layerNum].amplitudes[indexClosest]=100.0f;
		// update image
		layers[layerNum].updateImage();
	}//

	void PeopleJobOneLayer::updateImage(){
		if(DEBUG_LAYERS)printf(">>PeopleJobOneLayer::updateImage amplitudes %d\n",amplitudes.size());
		float sigmaX,sigmaY;
		sigmaX=sigmaY=stdev;
		
		float x,y,x0,y0,A;
		//for each pixel update the value
		layerData=cv::Mat::zeros(layerData.rows,layerData.cols,CV_32FC1);
		for(int aN=0;aN<amplitudes.size();aN++){
			x0=samplePosition[aN].x()+worldWidth/2.0f;//0-4000
			y0=samplePosition[aN].y()+worldWidth/2.0f;
			A=amplitudes[aN];
			if(A==0)
				continue;
			for(int c=0;c<layerData.cols;c++){
				for(int r=0;r<layerData.rows;r++){
					//printf("c %d r %d\n",c,r);
		
					x=(0.5f+c)*worldWidth/layerData.cols;//0-4000
					y=(0.5f+r)*worldWidth/layerData.rows;//0-4000

					layerData.at<float>(r,c)+=A*qExp(-( (((x-x0)*(x-x0))/(2*sigmaX*sigmaX))+ (((y-y0)*(y-y0))/(2*sigmaY*sigmaY)) ));

					if(layerData.at<float>(r,c)>100)
						layerData.at<float>(r,c)=100.0f;
				}
			}
		}
		//printf("normalize
		
		updateTexFromData();
		if(DEBUG_LAYERS)printf("<<PeopleJobOneLayer::updateImage\n");
	}//

	void PeopleJobInfoLayers::drawDrawingCircle(VBORenderManager& rendManager){
		QVector3D pos=rendManager.mousePos3D;
		////////////////////////////////////////////////
		// render points
		rendManager.removeStaticGeometry("PJ_points",false);

		float posZ=10.0f;
		std::vector<Vertex> vertP(samplePosition.size());
		for(int mP=0;mP<samplePosition.size();mP++){
			vertP[mP]=Vertex(QVector3D(samplePosition[mP].x(),samplePosition[mP].y(),posZ),QVector3D(1.0f,0,0),QVector3D(),QVector3D());
		}
		rendManager.addStaticGeometry("PJ_points",vertP,"",GL_POINTS,1|mode_AdaptTerrain);

		glPointSize(8.0f);
		rendManager.renderStaticGeometry("PJ_points");
		rendManager.removeStaticGeometry("PJ_points",false);
		////////////////////////////////////////////////
		// render circle
		rendManager.removeStaticGeometry("PJ_Circle",false);

		float radius=stdev*2.0f;
		const int circle_points = 100;	
		double angle;
		double posX, posY;
		std::vector<Vertex> vertC;
		int indexClosest=-1;
		indexClosest=getIndexClosestToPoint(pos);
		//printf("/////////////////////// indexClosest %d\n",indexClosest);
		float x=samplePosition[indexClosest].x();
		float y=samplePosition[indexClosest].y();
		for (int i = 0; i < circle_points; i++) {    
			angle = 2*M_PI*i/circle_points; 
			posX = x+radius*cos(angle);
			posY = y+radius*sin(angle);			
			vertC.push_back(Vertex(QVector3D(posX, posY, posZ),QVector3D(0.1f,0,1.0f),QVector3D(),QVector3D()));				
		}	
		glLineWidth(5.0);
		rendManager.addStaticGeometry("PJ_Circle",vertC,"",GL_LINE_LOOP,1|mode_AdaptTerrain);
		rendManager.renderStaticGeometry("PJ_Circle");
		rendManager.removeStaticGeometry("PJ_Circle");
	}//


	PeopleJobOneLayer::PeopleJobOneLayer(){
		cameraTex=0;
	}

	void PeopleJobOneLayer::updateTexFromData(){
		if(DEBUG_LAYERS)printf(">>PeopleJobOneLayer::updateTexFromData()\n");
		cv::Mat layerDataConv=cv::Mat::zeros(layerData.rows,layerData.cols,CV_8UC3);

		for(int c=0;c<layerData.cols;c++){
			for(int r=0;r<layerData.rows;r++){
				float oldV=layerData.at<float>(r,c);
				float o1=1.0f-(oldV/100.0f);
				float rC,gC,bC;
				LC::misctools::HSVtoRGB(&rC,&gC,&bC,120.0f*o1,0.9f,0.8f);//0 red 120` green
				layerDataConv.at<cv::Vec3b>(r,c)=cv::Vec3b((uchar)(255*bC),(uchar)(255*gC),(uchar)(255*rC));
			}
		}

		if(cameraTex==0){
			glGenTextures(1, &cameraTex);                  // Create The Texture
		}
		if(DEBUG_LAYERS)printf("-->put in texture cameraTex %u\n",cameraTex);
		
		glActiveTexture(GL_TEXTURE5);
		glBindTexture(GL_TEXTURE_2D, cameraTex);               
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);//GL_REPLACE);
		//printf("put in texture3\n");
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR_MIPMAP_LINEAR);//GL_LINEAR

		//printf("put in texture4 layerDataConv %d %d\n",layerDataConv.cols,layerDataConv.rows);
        //gluBuild2DMipmaps(GL_TEXTURE_2D, 4, layerDataConv.cols, layerDataConv.rows, GL_BGR_EXT, GL_UNSIGNED_BYTE,  layerDataConv.ptr()); //!!!N!!!
		glActiveTexture(GL_TEXTURE0);
		if(DEBUG_LAYERS)printf("<<PeopleJobOneLayer::updateTexFromData()\n");
	}//

	void PeopleJobInfoLayers::clear(){
		layers.clear();
		initialized=false;
	}//


	void PeopleJobInfoLayers::initInfoLayer(VBORenderManager& rendManager,float _samplingDistance,bool initLayer){
		samplingDistance=_samplingDistance;
		worldWidth=rendManager.side;

		const float val_stdev=samplingDistance/2.0f;

		// init positions
		int numSamples=worldWidth/samplingDistance;
		printf("++++ LAYER numSamples %d\n",numSamples);
		float leftValue=-worldWidth/2.0f;
		stdev=val_stdev;
		samplePosition.resize(numSamples*numSamples);

		for(int x=0;x<numSamples;x++){
			for(int y=0;y<numSamples;y++){
				float xV=leftValue+(0.5f+x)*samplingDistance;
				float yV=leftValue+(0.5f+y)*samplingDistance;
				samplePosition[x+y*numSamples]=QVector3D(xV,yV,0.0f);
			}
		}
		// init vectors
		if(layers.size()!=0){
			layers.clear();
		}

		PeopleJobOneLayer jobL;
		jobL.name="Jobs";
		PeopleJobOneLayer peopleL;
		peopleL.name="People";

		layers.push_back(jobL);
		layers.push_back(peopleL);

		for(int l=0;l<layers.size();l++){
			layers[l].worldWidth=worldWidth;

			layers[l].initLayerGauss(l,numSamples,stdev,samplePosition);
			if(initLayer==true)
				layers[l].createRandomDistribution();
            QVector3D zero(0,0,0);
            updateLayer(l, zero, 0);

		}
		//set pointers to be able to access
		G::global()["jobDist"]=qVariantFromValue((void*)(&layers[0].layerData));
		G::global()["peopleDist"]=qVariantFromValue((void*)(&layers[1].layerData));
		initialized=true;

	}//

	void updateMinMaxPJ(QVector3D& newPoint,QVector3D& minBox,QVector3D& maxBox){
		if(newPoint.x()<minBox.x()){
			minBox.setX(newPoint.x());
		}
		if(newPoint.y()<minBox.y()){
			minBox.setY(newPoint.y());
		}
		if(newPoint.x()>maxBox.x()){
			maxBox.setX(newPoint.x());
		}
		if(newPoint.y()>maxBox.y()){
			maxBox.setY(newPoint.y());
		}
	}//

	// Given the terrain layout create a ramdom distribution
	void PeopleJobOneLayer::createRandomDistribution(){
		if(DEBUG_LAYERS)printf("  >>createRandomDistribution\n");
		// create bounding box
		if(DEBUG_LAYERS)printf("    create bounding box\n");
		QVector3D minBox(FLT_MAX,FLT_MAX,FLT_MAX);
		QVector3D maxBox(-FLT_MAX,-FLT_MAX,-FLT_MAX);
		insideSamplings.clear();
		for(int mP=0;mP<samplePosition.size();mP++){
			if(LC::misctools::isPointWithinLoop(G::boundingPolygon,samplePosition[mP])){
				updateMinMaxPJ(samplePosition[mP],minBox,maxBox);
				insideSamplings.insert(mP);
			}
		}
		// divide bbox into four sections and put the proper index
		if(DEBUG_LAYERS)printf("    divide bbox into four sections and put the proper index\n");
		std::vector<QSet<int>> groups;
		groups.resize(4);
		QSet<int>::const_iterator i = insideSamplings.constBegin();
		QVector3D minPointBBox=(minBox+maxBox)/2.0f;
		while (i != insideSamplings.constEnd()) {
			QVector3D cPoint=samplePosition[*i];
			int groupNumber;
			if(cPoint.x()<minPointBBox.x()){
				if(cPoint.y()<minPointBBox.y()){
					groupNumber=0;
				}else{
					groupNumber=1;
				}
			}else{
				if(cPoint.y()<minPointBBox.y()){
					groupNumber=2;
				}else{
					groupNumber=3;
				}
			}
			groups[groupNumber].insert(*i);
			++i;
		}
		// randomly choose edges
		if(DEBUG_LAYERS)printf("    randomly choose edges\n");
		if(layerType==0){
			qsrand(54321654);
			groups[0].clear();
			groups[3].clear();
		}else{
			qsrand(2121747);
			groups[1].clear();
			groups[2].clear();
		}
		for(int gN=0;gN<4;gN++){
			if(groups[gN].size()<=0)
				continue;
			QList<int> gList=groups[gN].toList();
			int numToChange=gList.size()/3;
			if(numToChange<3)numToChange=3;
			for(int iT=0;iT<numToChange;iT++){
				int ranEl=qrand()%gList.size();
				int index=gList[ranEl];
				amplitudes[index]=(qrand()%80)+20;
			}
		}
		if(DEBUG_LAYERS)printf("  <<createRandomDistribution\n");
	}//


	void PeopleJobOneLayer::initLayerGauss(int _layerType,int _numSamples,float _stdev,std::vector<QVector3D>& _samplePosition){
		if(DEBUG_LAYERS)printf("_layerType %d\n",_layerType);
		layerType=_layerType;
		numSamples=_numSamples;
		stdev=_stdev;
		samplePosition=_samplePosition;
		amplitudes.resize(numSamples*numSamples);
		for(int x=0;x<numSamples;x++){
			for(int y=0;y<numSamples;y++){
				//amplitudes[x+y*numSamples]=0.0f;
				amplitudes[x+y*numSamples]=0;//((float)qrand())/RAND_MAX;//random
			}
		}
		layerData=cv::Mat::zeros(512,512,CV_32FC1);//init size
		updateImage();
		//printf("--** cameraTex %u\n",cameraTex);
	}//

	int PeopleJobInfoLayers::getIndexClosestToPoint(QVector3D& point){
		int numSaples=worldWidth/samplingDistance;
		int xCoord;
		if(point.x()<-worldWidth/2.0f){
			xCoord=0;
		}
		if(point.x()>worldWidth/2.0f){
			xCoord=(worldWidth/samplingDistance)-1;
		}
		xCoord=(point.x()+worldWidth/2.0f)/samplingDistance;//0-4000/sampling

		int yCoord;
		if(point.y()<-worldWidth/2.0f){
			yCoord=0;
		}
		if(point.y()>worldWidth/2.0f){
			yCoord=(worldWidth/samplingDistance)-1;
		}
		yCoord=(point.y()+worldWidth/2.0f)/samplingDistance;//0-4000/sampling

		return xCoord+yCoord*numSaples;
	}//

	// go through all the inside 
	int PeopleJobOneLayer::getRandomIndexWithAmpl(){
		if(insideSamplings.size()<=0){
			printf("ERROR: PeopleJobOneLayer::getRandomIndexWithAmpl--> Empty\n");
			return 0;
		}
		QSet<int>::const_iterator i = insideSamplings.constBegin();
		std::vector<float> accProbability;
		QMap<int,int> posInArrayIndexInSamples;
		int posInArray=0;
		while (i != insideSamplings.constEnd()) {
			float ampl=amplitudes[*i];
			if(ampl>0){
				float accProb=ampl;
				if(accProbability.size()>0){
					accProb+=accProbability[accProbability.size()-1];
				}
				accProbability.push_back(accProb);
				posInArrayIndexInSamples.insert(posInArray,*i);
				posInArray++;
			}
			++i;
		}
		int maxAccValue=accProbability[accProbability.size()-1];
		float randAmpl=(((float)qrand())/RAND_MAX)*maxAccValue;//sample in the acc
		int aN;
		bool found=true;
		for(aN=0;aN<accProbability.size();aN++){
			if(accProbability[aN]>randAmpl){
				found=true;
				break;
			}
		}
		return posInArrayIndexInSamples[aN];
	}//

	int PeopleJobOneLayer::getRandomIndexWithin(){
		if(insideSamplings.size()<=0){
			printf("ERROR: PeopleJobOneLayer::getRandomIndexWithAmpl--> Empty\n");
			return 0;
		}
		QSet<int>::const_iterator i = insideSamplings.constBegin();
		std::vector<float> accProbability;
		QMap<int,int> posInArrayIndexInSamples;
		int posInArray=0;
		while (i != insideSamplings.constEnd()) {
			float ampl=amplitudes[*i];
			
				float accProb=ampl;
				if(accProbability.size()>0){
					accProb+=accProbability[accProbability.size()-1];
				}
				accProbability.push_back(accProb);
				posInArrayIndexInSamples.insert(posInArray,*i);
				posInArray++;
			
			++i;
		}
		int maxAccValue=accProbability[accProbability.size()-1];
		float randAmpl=(((float)qrand())/RAND_MAX)*maxAccValue;//sample in the acc
		int aN;
		bool found=true;
		for(aN=0;aN<accProbability.size();aN++){
			if(accProbability[aN]>randAmpl){
				found=true;
				break;
			}
		}
		return posInArrayIndexInSamples[aN];

	}//


	void PeopleJobInfoLayers::saveToFile(){
		QString fileName="data/peopleJobLayers.txt";
		QFile file(fileName);
		if(!file.open(QIODevice::WriteOnly)) {
			printf("ERROR: Not possible to read %s\n",fileName.toUtf8().constData());
			return;
		}
		
		QTextStream out(&file);
		// save pre info
		out<<"NumOfLayers:"<<layers.size()<<"\n";
		out<<"samplingDistance:"<<samplingDistance<<"\n";
		out<<"stdev:"<<stdev<<"\n";
		out<<"samplePosition.size():"<<samplePosition.size()<<"\n";
		for(int nS=0;nS<samplePosition.size();nS++){
			out<<"samplePosition["<<nS<<"]:"<<samplePosition[nS].x()<<":"<<samplePosition[nS].y()<<":"<<samplePosition[nS].z()<<"\n";
		}
		// save each layer
		for(int nL=0;nL<layers.size();nL++){
			//amplitudes
			for(int nS=0;nS<samplePosition.size();nS++){
				out<<"amplitudes["<<nS<<"]:"<<layers[nL].amplitudes[nS]<<"\n";
			}
			out<<"name:"<<layers[nL].name<<"\n";
			out<<"layerType:"<<layers[nL].layerType<<"\n";
		}
		out.flush();
		file.close();
	}//
		
	
	void PeopleJobInfoLayers::loadFromFile(){
		//read texture file
		QString fileName="data/peopleJobLayers.txt";
		QFile file(fileName);
		if(!file.open(QIODevice::ReadOnly)) {
			printf("ERROR: Not possible to read %s\n",fileName.toUtf8().constData());
			return;
		}
		QTextStream in(&file);
		int numLayers=in.readLine().split(":").at(1).toInt();
		//printf("NumLayers %d\n",numLayers);
		layers.resize(numLayers);
		samplingDistance=in.readLine().split(":").at(1).toFloat();
		//printf("2\n");
		stdev=in.readLine().split(":").at(1).toFloat();
		int numSamples=in.readLine().split(":").at(1).toInt();
		//printf("3\n");
		samplePosition.resize(numSamples);
		for(int nS=0;nS<numSamples;nS++){
			QStringList splList=in.readLine().split(":");//.at(1).toFloat(),in.readLine().split(":").at(2).toFloat(),in.readLine().split(":").at(3).toFloat()
			samplePosition[nS]=QVector3D(splList.at(1).toFloat(),splList.at(2).toFloat(),splList.at(3).toFloat());
		}
		initialized=true;
		printf("NumLayers %d Num Samples %d\n",numLayers,numSamples);
		for(int nL=0;nL<numLayers;nL++){
			// amplitudes
			for(int nS=0;nS<numSamples;nS++){
				layers[nL].amplitudes[nS]=in.readLine().split(":").at(1).toFloat();
			}
			layers[nL].samplePosition=samplePosition;
			//auto recalcule
			layers[nL].layerData=cv::Mat::zeros(512,512,CV_32FC1);//init size
			// read
			layers[nL].name=in.readLine().split(":").at(1);
			layers[nL].layerType=in.readLine().split(":").at(1).toInt();
			layers[nL].numSamples=numSamples;
			layers[nL].stdev=stdev;
			layers[nL].updateImage();
		}
		file.close();
	}//

	void PeopleJobInfoLayers::clearLayers(){
		//set values to zero
		for(int nL=0;nL<layers.size();nL++){
			std::fill(layers[nL].amplitudes.begin(),layers[nL].amplitudes.end(),0.0f);
		}

	}//

	bool PeopleJobInfoLayers::layersEmpty(){
		//printf("layersEmpty\n");
		bool isEmpty = true;
		for (int nL = 0; nL < layers.size() && isEmpty; nL++){
			for (int aN = 0; aN < layers[nL].amplitudes.size() && isEmpty; aN++){
				if (layers[nL].amplitudes[aN] != 0){
					isEmpty = false;
					//break;
				}
			}
		}
		//printf("layersEmpty2\n");
		return isEmpty;
	}//

}// namespace LC
