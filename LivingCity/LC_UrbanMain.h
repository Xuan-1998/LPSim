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
#include <QMainWindow>
#include "ui_LC_UrbanMain.h"

namespace LC {

class LCGLWidget3D;
class UrbanModule;
class TextureManager;

class LCUrbanMain : public QMainWindow {
	Q_OBJECT

protected:
	
public:
	Ui::LCUrbanMain ui;
	LCGLWidget3D* glWidget3D;

    LCUrbanMain(QWidget *parent = 0, Qt::WindowFlags flags = 0);
	~LCUrbanMain();

	void init();
	bool initModeActive;
	void render(TextureManager* textureManager);

public slots:

  ////////////////////////////////////////
  // PROCEDURAL MODELING
	void cudaLanesNumArterialSlot(int val);
	void cudaLanesNumRoadSlot(int val);


	void arterialEdgesIrregularitySlot(int);
	void arterialEdgesCurvatureSlot(int);
	void arterialNumDepartingSlot(int);

	void localEdgesLengthUSlot(int);
	void localEdgesLengthVSlot(int);
	void localEdgesIrregularitySlot(int);
	void localEdgesCurvatureSlot(int);

	void numPlaceTypesSlot(int);

	//---parcels
	void parcelAreaMeanSlot(int);
	void parcelAreaDeviationSlot(int);
	void parcelSplitDeviationSlot(int);
	void parcelSetbackFrontSlot(int);
	void parcelSetbackSidesSlot(int);


	//---buildings
	void buildingHeightMeanSlot(int);
	void buildingHeightDeviationSlot(int);

	//---land use
	void landUseParkPercentageSlot(int);

	void onRender2DSlot(bool);
	void onRender3DSlot(bool);

	//--terrain
	void updateTerrainLabels(int newValue);
	void onNewTerrain();
	void onLoadTerrain();
	void onSaveTerrain();

	//--layer
	void onLoadLayers(bool);
	void onSaveLayers(bool);
	void onClearLayers(bool);
	void onLayerEnable(int);

  void onProceduralModeling(int);

  ////////////////////////////////////////
  // TRAFFIC B18

	//--label
	//void onB18LabelPressed(bool);

	//--bTraffic
	void onB18SimulateCPUPressed(bool);
	void onB18SimulateGPUPressed(bool);

	void onB18CreateRandomOD(bool);
  void onB18LoadB18OD(bool);
  void b18CreateOD(bool random);

  // Load OD and Rorutes.
	void onB18LoadODR(bool);
	void onB18SaveODR(bool);


private:

};

} // namespace LC
