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

#pragma once

#include <qmatrix4x4.h>

#include "highgui/highgui.hpp"
#include "imgproc/imgproc.hpp"

#include <QtCore/qmath.h>
#include <qfile.h>

namespace LC {

	class VBORenderManager;

	class PeopleJobOneLayer{
	public:
		PeopleJobOneLayer();

		cv::Mat layerData;
		std::vector<float> amplitudes;
		std::vector<QVector3D> samplePosition;

		unsigned int cameraTex;

		QString name;
		int layerType;
		int numSamples;
		float stdev;
		float worldWidth;
		QSet<int> insideSamplings;// contains the samples that are inside of the polygon
		int getRandomIndexWithAmpl();
		int getRandomIndexWithin();
		
		void updateTexFromData();

		void createRandomDistribution();
		void initLayerGauss(int layerType,int numSamples,float stdev,std::vector<QVector3D>& samplePosition);
		void updateImage();
	};


	class PeopleJobInfoLayers 
	{

	public:
		PeopleJobInfoLayers();
		~PeopleJobInfoLayers();	

		float samplingDistance;
		float worldWidth;

		std::vector<PeopleJobOneLayer> layers;


		void drawDrawingCircle(VBORenderManager& rendManager);
		void updateLayer(int layerNum,QVector3D& pos,float change);
		QVector3D brushPos;
		float stdev;

		std::vector<QVector3D> samplePosition;

		void clear();
		bool initialized;
		void initInfoLayer(VBORenderManager& rendManager,float samplingDistance,bool initLayer=true);
		int getIndexClosestToPoint(QVector3D& point);

		void saveToFile();
		void loadFromFile();
		void clearLayers();
		bool layersEmpty();
	};
}
