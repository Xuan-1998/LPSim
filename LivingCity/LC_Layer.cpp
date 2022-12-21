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


#include "LC_Layer.h"
#include <QStringList>
#include "VBOUtil.h"


namespace LC {

Layer::Layer() {
  initialized = false;
  texData = 0;
}//

void Layer::init(QVector3D _minPos, QVector3D _maxPos, int _resolutionX,
                 int _resolutionY, int _typeLayer, int _imgResX, int _imgResY) {

  minPos = _minPos;
  maxPos = _maxPos;
  resolutionX = _resolutionX;
  resolutionY = _resolutionY;
  imgResX = _imgResX;
  imgResY = _imgResY;

  typeLayer = _typeLayer;

  sideX = abs(maxPos.x() - minPos.x()) / resolutionX;
  sideY = abs(maxPos.y() - minPos.y()) / resolutionY;


  //layerData=cv::Mat::zeros(imgResY,imgResX,CV_8UC1);
  layerData = cv::Mat(imgResY, imgResX, CV_8UC1, 10);
  //randomPerlineNoise(layerData);// REMOVE !!!
  updateTexFromData();

  initialized = true;
}//

//////////////////////////////////////////////////////////////
//dreamincode.net/forums/topic/66480-perlin-noise/
inline double findnoise2(double x, double y) {
  int n = (int)x + (int)y * 57;
  n = (n << 13)^n;
  int nn = (n * (n * n * 60493 + 19990303) + 1376312589) & 0x7fffffff;
  return 1.0 - ((double)nn / 1073741824.0);
}//
inline double interpolate(double a, double b, double x) {
  double ft = x * 3.1415927;
  double f = (1.0 - cos(ft)) * 0.5;
  return a * (1.0 - f) + b * f;
}//

double noise(double x, double y) {
  double floorx = (double)((int)
                           x); //This is kinda a cheap way to floor a double integer.
  double floory = (double)((int)y);
  double s, t, u, v; //Integer declaration
  s = findnoise2(floorx, floory);
  t = findnoise2(floorx + 1, floory);
  u = findnoise2(floorx, floory +
                 1); //Get the surrounding pixels to calculate the transition.
  v = findnoise2(floorx + 1, floory + 1);
  double int1 = interpolate(s, t, x - floorx); //Interpolate between the values.
  double int2 = interpolate(u, v,
                            x - floorx); //Here we use x-floorx, to get 1st dimension. Don't mind the x-floorx thingie, it's part of the cosine formula.
  return interpolate(int1, int2,
                     y - floory); //Here we use y-floory, to get the 2nd dimension.
}//

void Layer::randomPerlineNoise(cv::Mat &perlinNoise) {
  if (perlinNoise.rows == 0 || perlinNoise.cols == 0) {
    perlinNoise = cv::Mat::zeros(imgResY, imgResX, CV_8UC1);
  }

  int octaves = 6;
  double zoom = 75.0; //zoom in and out on it
  double p = 0.6f; //roughness of the picture

  for (int r = 0; r < perlinNoise.rows;
       r++) { //Loops to loop trough all the pixels
    for (int c = 0; c < perlinNoise.cols; c++) {
      /*// flat center
      uchar centerH=60;//30;
      float rad=0.02;
      if(r>=perlinNoise.cols/2-perlinNoise.cols*rad&&r<=perlinNoise.cols/2+perlinNoise.cols*rad&&
        c>=perlinNoise.rows/2-perlinNoise.rows*rad&&c<=perlinNoise.rows/2+perlinNoise.rows*rad){

        perlinNoise.at<uchar>(r,c)=centerH;
        continue;
      }*/
      double getnoise = 0;

      for (int a = 0; a < octaves - 1; a++) { //This loops trough the octaves.
        double frequency = pow(2.0,
                               a); //This increases the frequency with every loop of the octave.
        double amplitude = pow(p,
                               a); //This decreases the amplitude with every loop of the octave.
        getnoise += noise(((double)c) * frequency / zoom,
                          ((double)r) / zoom * frequency) *
                    amplitude; //This uses our perlin noise functions. It calculates all our zoom and frequency and amplitude
      }//											It gives a decimal value, you know, between the pixels. Like 4.2 or 5.1

      int color = (int)((getnoise * 128.0) + 128.0) -
                  40; //Convert to 0-256 values. //MAGIC 40 to decrease the average

      if (color > 255) {
        color = 255;
      }

      if (color < 0) {
        color = 0;
      }

      //SetPixel(ret,c,r,(int)((r/255.0)*(double)color),(int)((g/255.0)*(double)color),(int)((b/255.0)*(double)color));//This colours the image with the RGB values
      perlinNoise.at<uchar>(r, c) = (uchar)(color);
    }//														   given at the beginning in the function.
  }
}
//////////////////////////////////////////////////////////////



void Layer::updateTexFromData() {
  cv::imwrite("data/height.png", layerData);

  if (texData != 0) {
    glDeleteTextures(1, &texData);
    texData = 0;
  }

  texData = VBOUtil::loadImage("data/height.png", false, true);
}//

void Layer::updateLayer(float coordX, float coordY, float change, float rad) {
  float sigmaX, sigmaY;
  float x, y, x0, y0, A;

  x0 = coordX * imgResX; //0-imgRes
  y0 = coordY * imgResY;
  A = change;
  const float factor = 0.5f;
  sigmaX = imgResX * rad * 0.5f * factor;
  sigmaY = imgResY * rad * 0.5f * factor;

  if (change == FLT_MAX) {
    printf("Hack, flat area\n");
    // HACK Flat area
    int flatValue = (int)layerData.at<uchar>(y0, x0);
    int radImgX = rad * imgResX;
    int radImgY = rad * imgResY;

    for (int c = 0; c < layerData.cols; c++) {
      for (int r = 0; r < layerData.rows; r++) {
        if (abs(c - x0) < radImgX && abs(r - y0) < radImgY) {
          layerData.at<uchar>(r, c) = flatValue;
        }
      }
    }
  } else {
    // Normal
    printf("UpdateLayer x %f y %f ch %f rad %f x0 %f y0 %f imgResX %d imgResY %d layerData.cols %d layerData.rows %d\n",
           coordX, coordY, change, rad, x0, y0, imgResX, imgResY, layerData.cols,
           layerData.rows);
    float diff;

    for (int c = 0; c < layerData.cols; c++) {
      for (int r = 0; r < layerData.rows; r++) {

        x = (0.5f + c);
        y = (0.5f + r);
        diff = (A * qExp(-((((x - x0) * (x - x0)) / (2 * sigmaX * sigmaX)) + (((
                             y - y0) * (y - y0)) / (2 * sigmaY * sigmaY))))) * 255.0f; //0-255
        int newV = std::floor((int)layerData.at<uchar>(r,
                              c) + diff + 0.5f); //floor +0.5--> round

        if (newV > 255) {
          newV = 255;
        }

        if (newV < 0) {
          newV = 0;
        }

        layerData.at<uchar>(r, c) = newV;
      }
    }
  }

  // update image
  updateTexFromData();
}//

void Layer::updateLayerNewValue(float coordX, float coordY, float newValue,
                                float rad) {
  float sigmaX, sigmaY;
  float x, y, x0, y0, A;

  x0 = coordX * imgResX; //0-imgRes
  y0 = coordY * imgResY;
  sigmaX = imgResX * rad / 2.0f;
  sigmaY = imgResY * rad / 2.0f;


  int flatValue = (int)newValue;
  int radImg = rad * std::max<int>(imgResX, imgResY);

  printf("updateLayerNewValue x0 y0 %f %f \n");

  for (int c = 0; c < layerData.cols; c++) {
    for (int r = 0; r < layerData.rows; r++) {
      //if(abs(c-x0)<radImgX&&abs(r-y0)<radImgY){
      if (((c - x0) * (c - x0) + (r - y0) * (r - y0)) <= radImg * radImg) {
        layerData.at<uchar>(r, c) = flatValue;
      }
    }
  }

  // update image
  updateTexFromData();
}//

/*
float Layer::getValue(float xM,float yM){
	int c=xM*imgResX;//0-imgRes
	int r=yM*imgResY;
	if (c < 0) c = 0;
	if (c >= layerData.cols) c = layerData.cols - 1;
	if (r < 0) r = 0;
	if (r >= layerData.rows) r = layerData.rows - 1;
	return float((int)layerData.at<uchar>(r,c));
}
*/

// return the interpolated value
float Layer::getValue(float xM, float yM) {
  int c1 = floor(xM * imgResX); //0-imgRes
  int c2 = c1 + 1;
  int r1 = floor(yM * imgResY);
  int r2 = r1 + 1;

  if (c1 < 0) {
    c1 = 0;
  }

  if (c1 >= layerData.cols) {
    c1 = layerData.cols - 1;
  }

  if (c2 < 0) {
    c2 = 0;
  }

  if (c2 >= layerData.cols) {
    c2 = layerData.cols - 1;
  }

  if (r1 < 0) {
    r1 = 0;
  }

  if (r1 >= layerData.rows) {
    r1 = layerData.rows - 1;
  }

  if (r2 < 0) {
    r2 = 0;
  }

  if (r2 >= layerData.rows) {
    r2 = layerData.rows - 1;
  }

  float v1 = layerData.at<uchar>(r1, c1);
  float v2 = layerData.at<uchar>(r2, c1);
  float v3 = layerData.at<uchar>(r1, c2);
  float v4 = layerData.at<uchar>(r2, c2);

  float v12, v34;

  if (yM * imgResY <= r1) {
    v12 = v1;
    v34 = v3;
  } else if (yM * imgResY >= r2) {
    v12 = v2;
    v34 = v4;
  } else {
    float s = yM * imgResY - r1;
    float t = r2 - yM * imgResY;
    v12 = (v1 * t + v2 * s) / (s + t);
    v34 = (v3 * t + v4 * s) / (s + t);
  }

  if (xM * imgResX <= c1) {
    return v12;
  } else if (xM * imgResX >= c2) {
    return v34;
  } else {
    float s = xM * imgResX - c1;
    float t = c2 - xM * imgResX;
    return (v12 * t + v34 * s) / (s + t);
  }
}

void Layer::loadLayer(QString &fileName) {
  layerData = cv::imread(fileName.toUtf8().constData(), 0); //load one channel
  // update image
  updateTexFromData();
}//

void Layer::saveLayer(QString &fileName) {
  cv::imwrite(fileName.toUtf8().constData(), layerData);
}//

void Layer::smoothLayer() {
  int smoothV = std::max(imgResX / 80, 3);

  if (smoothV % 2 == 0) {
    smoothV++;
  }

  printf("smoothV %d\n", smoothV);
  cv::blur(layerData, layerData, cv::Size(smoothV, smoothV));
  updateTexFromData();
}//

}// LC namespace
