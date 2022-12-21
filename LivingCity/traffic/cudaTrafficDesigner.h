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
*
*		LC Project - Traffic Designer
*
*
*		@desc Class that calculate, display, density and take decissions of changes
*		@author igaciad
*
************************************************************************************************/

#ifndef LC_CUDA_TRAFFIC_DESIGNER_H
#define LC_CUDA_TRAFFIC_DESIGNER_H

//#include "../misctools/misctools.h"
//
////#include <QGLWidget>
//#include "qthread.h"
//
////#include <QtGlobal>
//
//#include "cudaTrafficPerson.h"
//#include "RoadGraph/roadGraph.h"
//#ifndef Q_MOC_RUN
////#include <boost/graph/dijkstra_shortest_paths.hpp>
//#include <boost/polygon/polygon.hpp>
//#endif
//
//#include "cudaTrafficMCMC.h"
//#include "cudaTrafficSimulator.h"
//
//namespace LC {
//
//
//
//	class CUDATrafficDesigner : public QObject
//	{
//		Q_OBJECT
//	public:
//		CUDATrafficDesigner();
//		~CUDATrafficDesigner();
//
//		// drawing
//		QVector3D mousePos;
//		void drawDrawingCircle();
//		void updateUserSelection(QVector3D& mouse3DPos);
//
//		//different regions
//		typedef boost::geometry::model::d2::point_xy<double> mPoint;
//		typedef boost::geometry::model::polygon<mPoint> polygon;
//
//		std::vector<polygon> blPolys;
//		std::vector<polygon> ltPolys;
//		std::vector<polygon> mtPolys;
//		std::vector<polygon> htPolys;
//		void renderRegions();
//
//		void copyResultToLeft();
//		void stopDesigner();
//
//		//designer
//		//bool isSetOriginalTrafficSimulator;
//		CUDATrafficSimulator designerTrafficSimulator;
//		//std::vector<CUDATrafficSimulator*> threadSimulators;
//		std::vector<CUDATrafficMCMC*> chains;
//		std::vector<QThread*> threads;
//		int currentBestThread;
//		float currentBestScore;
//
//		float currentMinCost;
//		bool findBestCost;
//		float scoreGoal;
//		//RoadGraph* designerRoadGraph;
//		void changeNetwork(CUDATrafficSimulator& originalTrafficSimulator,int numPasses,int numChains,int numSteps,CUDADesignOptions designOptions,PlaceTypesMainClass* currentPlaceTypes);
//		//CUDATrafficSimulator cudaTrafficSimulator;
//	public slots:
//			void newResult(int threadNum);
//			void errorString(QString err);
//			//void thredEnds();
//
//	};
//}

#endif
