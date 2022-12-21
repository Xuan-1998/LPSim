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

#include <qmatrix4x4.h>

#include "highgui/highgui.hpp"
#include "imgproc/imgproc.hpp"

#include <QtCore/qmath.h>
#include <qfile.h>

#include "VBOUtil.h"

namespace LC {

class Layer {
 public:
  Layer();
  void init(QVector3D _minPos, QVector3D _maxPos, int _resolutionX,
            int _resolutionY, int _typeLayer, int imgResX, int imgResY);
  //std::vector<float> amplitudes;
  //std::vector<QVector3D> samplePosition;
  //float stdev;

  //void updateGeometry();
  //std::vector<Vertex> vert;

  bool initialized;
  QVector3D maxPos;
  QVector3D minPos;

  int imgResX;
  int imgResY;
  int resolutionX;
  int resolutionY;
  float sideX;
  float sideY;

  int typeLayer;
  // 0 vboRenderManager

  //void createRandomDistribution();
  //float sample(QVector3D samPos);

  //text
  //void updateTexture();
  GLuint texData;

  // control
  //void updateLayer(QVector3D pos,float change);
  void updateLayer(float coordX, float coordY, float change, float rad);
  void updateLayerNewValue(float coordX, float coordY, float newValue, float rad);
  float getValue(float xM, float yM);
  void smoothLayer();
  //int closestSample(QVector3D pos);

  // perlin noise
  void randomPerlineNoise(cv::Mat &perlinNoise);

  void loadLayer(QString &fileName);
  void saveLayer(QString &fileName);

 public: // GEN: to access from the outside
  void updateTexFromData();
  cv::Mat layerData;

};//

}// namespace LC
