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
*		Place Type Description
*		@author igarciad
************************************************************************************************/
#pragma once

#include <QVariant>
#include <QSettings>
#include <QFile>
#include <QVector3D>
#include <vector>
#include <qvariant.h>

#include <QTextStream>
#include "../misctools/polygon_3D.h"
#include "../misctools/misctools.h"

namespace LC {

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

/**
* PlaceType class contains a place type instance in the city.
**/
class PlaceType {
 public:
  PlaceType(void);
  ~PlaceType(void);

  void initializePlaceType(void);

  PlaceType(const PlaceType &ref) {
    attributes = ref.attributes;
    initializePlaceType();
  }
  inline PlaceType &operator=(const PlaceType &ref) {
    attributes = ref.attributes;
    initializePlaceType();
    return (*this);
  }

  void savePlaceType(QTextStream &stream);
  bool loadPlaceType(QString &line, int versionNumber);

  QVector3D getExternalHandlerPos(void);
  QVector3D getExternalHandler2Pos(void);
  QVector3D getlengthUHandlerPos(void);
  QVector3D getlengthVHandlerPos(void);

  static const int kNumPTAttributes = 27;

  //returns true if bounding rectangle contains testPt
  bool containsPoint(QVector3D &testPt);


  //bounding rectangle
  LC::misctools::Loop3D boundingRectangle;
  boost::geometry::ring_type<LC::misctools::Polygon3D>::type bg_boudingRectangle;

  void updateBoundingRectangle(void);

  QVariant operator [](QString i) const    {
    return attributes[i];
  }
  QVariant &operator [](QString i) {
    return attributes[i];
  }

  QVector3D getQVector3D(QString i) {
    if (!attributes.contains(i)) {
      printf("PlaceType does not contain type %s\n", i.toLatin1().constData());
      return QVector3D();
    }

    return attributes[i].value<QVector3D>();
  }
  float getFloat(QString i) {
    if (!attributes.contains(i)) {
      printf("PlaceType does not contain type %s\n", i.toLatin1().constData());
      return 0;
    }

    return attributes[i].toFloat();
  }
  float getInt(QString i) {
    if (!attributes.contains(i)) {
      printf("PlaceType does not contain type %s\n", i.toLatin1().constData());
      return 0;
    }

    return attributes[i].toInt();
  }

 private:
  //attributes
  QHash<QString, QVariant> attributes;



};

//////////////////////////////////////////////////
////////////////////////////////////////////////
/**
* Main container class for place types
**/
class PlaceTypesMainClass {
 public:
  PlaceTypesMainClass() {
  }

  ~PlaceTypesMainClass() {
  }

  /**
  * Copy constructor.
  **/
  PlaceTypesMainClass(const PlaceTypesMainClass &ref) {
    myPlaceTypes = ref.myPlaceTypes;
  }

  /**
  * Assignment operator.
  **/
  inline PlaceTypesMainClass &operator=(const PlaceTypesMainClass &ref) {
    myPlaceTypes = ref.myPlaceTypes;
    return (*this);
  }

  PlaceType operator [](int i) const    {
    return myPlaceTypes[i];
  }
  PlaceType &operator [](int i) {
    return myPlaceTypes[i];
  }

  bool savePlaceTypeInstancesToFile(QString filename);
  int loadPlaceTypeInstancesFromFile(QString filename);
  int loadPlaceTypeInstancesFromFile(QString filename,
                                     LC::misctools::Polygon3D &pgon);

  std::vector<PlaceType> myPlaceTypes;

  int addPlaceType(int ptIdx);
  int removePlaceType(int ptIdx);
  int sendPlaceTypeToTop(int ptIdx);
  int sendPlaceTypeOneUp(int ptIdx);
  int sendPlaceTypeOneDown(int ptIdx);
  int sendPlaceTypeToBottom(int ptIdx);

  static const int kVersionNumber = 4;
};

}
