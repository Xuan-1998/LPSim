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

#include "placeTypeInstances.h"
#include "qstringlist.h"
#include "qmatrix4x4.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace LC {




//===================================================
//===================================================
//===================================================


PlaceType::PlaceType(void) {
  initializePlaceType();
}


PlaceType::~PlaceType(void) {
}

void PlaceType::initializePlaceType(void) {
  this->updateBoundingRectangle();
}

void PlaceType::savePlaceType(QTextStream &stream) {
  stream
      << this->attributes["pt_building_height_deviation"].toFloat() << ","
      << this->attributes["pt_building_height_mean"].toFloat() << ","
      << this->attributes["pt_building_max_frontage"].toFloat() << ","
      << this->attributes["pt_building_max_depth"].toFloat() << ","
      << this->attributes["pt_category"].toInt() << ","
      << this->attributes["pt_cur_edges_count"].toInt() << ","
      << this->attributes["pt_edges_curvature"].toFloat() << ","
      << this->attributes["pt_edges_irregularity"].toFloat() << ","
      << this->attributes["pt_edges_lengthU"].toFloat() << ","
      << this->attributes["pt_edges_lengthV"].toFloat() << ","
      << this->attributes["pt_loc_edges_curvature"].toFloat() << ","
      << this->attributes["pt_loc_edges_irregularity"].toFloat() << ","
      << this->attributes["pt_loc_edges_lengthU"].toFloat() << ","
      << this->attributes["pt_loc_edges_lengthV"].toFloat() << ","
      << this->attributes["pt_edges_width"].toFloat() << ","
      << this->attributes["pt_radius"].toFloat() << ","
      << this->attributes["pt_radius2"].toFloat() << ","
      << this->attributes["pt_is_road_generator"].toFloat() << ","
      << this->attributes["pt_num_departing"].toFloat() << ","
      << this->attributes["pt_orientation"].toFloat() << ","
      << this->attributes["pt_parcel_area_deviation"].toFloat() << ","
      << this->attributes["pt_parcel_area_mean"].toFloat() << ","
      << this->attributes["pt_parcel_setback_front"].toFloat() << ","
      << this->attributes["pt_parcel_setback_rear"].toFloat() << ","
      << this->attributes["pt_parcel_setback_sides"].toFloat() << ","
      << this->attributes["pt_parcel_split_deviation"].toFloat() << ","
      << this->attributes["pt_pt"].value<QVector3D>().x() << ","
      << this->attributes["pt_pt"].value<QVector3D>().x() << ","
      << this->attributes["pt_pt"].value<QVector3D>().x() << ","
      << this->attributes["pt_park_percentage"].toFloat() << endl;
}

bool PlaceType::loadPlaceType(QString &line, int versionNumber) {
  QStringList attList = line.split(",");

  bool correctFormat = false;
  bool ptIsRectangle = false;

  int numExpectedAttributes;
  int numReadAttributes;

  switch (versionNumber) {
  case 0:
    numExpectedAttributes = 25;
    break;

  case 1:
    numExpectedAttributes = 26;
    break;

  case 2:
    numExpectedAttributes = 27;
    break;

  case 3:
    numExpectedAttributes = 28;
    break;

  case 4:
    numExpectedAttributes = 30;
    break;

  case 5:
    numExpectedAttributes = 30;
    break;

  default:
    numExpectedAttributes = 0;
    break;
  }

  numReadAttributes = attList.size();

  if (numExpectedAttributes != numReadAttributes) {
    printf("ERROR: Place type file format is incorrect. File not read\n");
    return false;
  }

  float tmpX, tmpY, tmpZ;

  int pos = 0;

  attributes["pt_building_height_deviation"] = attList.at(pos++).toFloat();
  attributes["pt_building_height_mean"] = attList.at(pos++).toFloat();


  if (versionNumber > 3) {
    attributes["pt_building_max_frontage"] = attList.at(pos++).toFloat();
    attributes["pt_building_max_depth"] = attList.at(pos++).toFloat();

  } else {
    attributes["pt_building_max_frontage"] = 0;
    attributes["pt_building_max_depth"] = 0;
  }

  attributes["pt_category"] = attList.at(pos++).toInt();
  attributes["pt_cur_edges_count"] = attList.at(pos++).toInt();

  attributes["pt_edges_curvature"] = attList.at(pos++).toFloat();
  attributes["pt_edges_irregularity"] = attList.at(pos++).toFloat();

  attributes["pt_edges_lengthU"] = attList.at(pos++).toFloat();
  attributes["pt_edges_lengthV"] = attList.at(pos++).toFloat();

  attributes["pt_loc_edges_curvature"] = attList.at(pos++).toFloat();
  attributes["pt_loc_edges_irregularity"] = attList.at(pos++).toFloat();

  attributes["pt_loc_edges_lengthU"] = attList.at(pos++).toFloat();
  attributes["pt_loc_edges_lengthV"] = attList.at(pos++).toFloat();

  attributes["pt_edges_width"] = attList.at(pos++).toFloat();

  float tmpRad = attList.at(pos++).toFloat();
  attributes["pt_radius"] = attList.at(pos++).toFloat();

  if (versionNumber > 0) {
    attributes["pt_radius2"] = attList.at(pos++).toFloat();
  } else {
    attributes["pt_radius"] = tmpRad;
  }

  if (versionNumber > 2) {
    attributes["pt_is_road_generator"] = attList.at(pos++).toInt();
  } else {
    attributes["pt_is_road_generator"] = 1;
  }

  attributes["pt_num_departing"] = attList.at(pos++).toInt();

  attributes["pt_orientation"] = attList.at(pos++).toFloat();
  attributes["pt_parcel_area_deviation"] = attList.at(pos++).toFloat();
  attributes["pt_parcel_area_mean"] = attList.at(pos++).toFloat();

  attributes["pt_parcel_setback_front"] = attList.at(pos++).toFloat();


  if (versionNumber > 1) {
    attributes["pt_parcel_setback_rear"] = attList.at(pos++).toFloat();
  } else {
    attributes["pt_parcel_setback_rear"] = 0;
  }

  attributes["pt_parcel_setback_sides"] = attList.at(pos++).toFloat();

  attributes["pt_parcel_split_deviation"] = attList.at(pos++).toFloat();

  tmpX = attList.at(pos++).toFloat();
  tmpY = attList.at(pos++).toFloat();
  tmpZ = attList.at(pos++).toFloat();

  attributes["pt_pt"] = QVector3D(tmpX, tmpY, tmpZ);

  attributes["pt_park_percentage"] = attList.at(pos++).toFloat();

  this->initializePlaceType();

  return true;
}

QVector3D PlaceType::getExternalHandlerPos(void) {
  return (QVector3D(
            getQVector3D("pt_pt").x() + getFloat("pt_radius") * cos(
              getFloat("pt_orientation")),
            getQVector3D("pt_pt").y() + getFloat("pt_radius") * sin(
              getFloat("pt_orientation")),
            0.0f));
}

QVector3D PlaceType::getExternalHandler2Pos(void) {
  return (QVector3D(
            getQVector3D("pt_pt").x() + getFloat("pt_radius2") * cos(
              getFloat("pt_orientation") + 0.5f * M_PI),
            getQVector3D("pt_pt").y() + getFloat("pt_radius2") * sin(
              getFloat("pt_orientation") + 0.5f * M_PI),
            0.0f));
}

QVector3D PlaceType::getlengthUHandlerPos(void) {
  return (QVector3D(
            getQVector3D("pt_pt").x() + getFloat("pt_edges_lengthU") * cos(
              getFloat("pt_orientation")),
            getQVector3D("pt_pt").y() + getFloat("pt_edges_lengthU") * sin(
              getFloat("pt_orientation")),
            0.0f));
}

QVector3D PlaceType::getlengthVHandlerPos(void) {
  return (QVector3D(
            getQVector3D("pt_pt").x() - getFloat("pt_edges_lengthV") * sin(
              getFloat("pt_orientation")),
            getQVector3D("pt_pt").y() + getFloat("pt_edges_lengthV") * cos(
              getFloat("pt_orientation")),
            0.0f));
}


void PlaceType::updateBoundingRectangle(void) {
  printf(">>1updateBoundingRectangle\n");
  this->boundingRectangle.clear();

  LC::misctools::Loop3D tmpLoop;

  QMatrix4x4 xformMat;
  xformMat.setToIdentity();
  xformMat.translate(getQVector3D("pt_pt"));
  xformMat.rotate(57.2957795f * (getFloat("pt_orientation")), //rad2degree
                  0.0f, 0.0f, 1.0f);

  float minRad = 1.0f;
  printf(">>2updateBoundingRectangle\n");

  if (getFloat("pt_radius") < minRad) {
    attributes["pt_radius"] = minRad;
  }

  if (getFloat("pt_radius2") < minRad) {
    attributes["pt_radius2"] = minRad;
  }


  QVector3D dir1(getFloat("pt_radius"), 0.0f,  0.0f);
  QVector3D dir2(0.0f, getFloat("pt_radius2"), 0.0f);

  printf(">>3updateBoundingRectangle\n");
  tmpLoop.push_back(QVector3D(-dir1 - dir2));
  tmpLoop.push_back(QVector3D(-dir1 + dir2));
  tmpLoop.push_back(QVector3D(dir1 + dir2));
  tmpLoop.push_back(QVector3D(dir1 - dir2));
  printf(">>4updateBoundingRectangle\n");
  LC::misctools::Polygon3D::transformLoop(tmpLoop, this->boundingRectangle,
                                          xformMat);

  printf(">>51updateBoundingRectangle\n");
  //boost::geometry::assign(this->bg_boudingRectangle, this->boundingRectangle);
  //boost::geometry::correct(this->bg_boudingRectangle);

}

//returns true if bounding rectangle contains testPt
bool PlaceType::containsPoint(QVector3D &testPt) {
  //return isPointWithinLoop(this->bg_boudingRectangle, testPt);
  return true;
}

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
bool PlaceTypesMainClass::savePlaceTypeInstancesToFile(QString filename) {
  QFile placeTypesFile(filename);

  if (!placeTypesFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
    printf("ERROR: Cannot open the file %s for writing\n",
           filename.toLocal8Bit().data());
    return false;
  }

  QTextStream stream(&placeTypesFile);

  //save version number
  stream << "version " << kVersionNumber << endl;

  //save bounding polygon of city
  //first write number of points
  /*int boundingPgonSz = LC::misctools::Global::global()->boundingPolygon.size();
  stream << boundingPgonSz << endl;
  //then write the coordinates of each point
  for(int i=0; i<boundingPgonSz; ++i){
        stream << LC::misctools::Global::global()->boundingPolygon.at(i).x() << ","
                << LC::misctools::Global::global()->boundingPolygon.at(i).y() << ","
                << LC::misctools::Global::global()->boundingPolygon.at(i).z() << endl;
  }*/

  //save landmarks


  //save place types at the end, one place type per line in file
  for (int i = 0; i < G::global().getInt("num_place_types"); ++i) {
    this->myPlaceTypes.at(i).savePlaceType(stream);
  }

  if (placeTypesFile.isOpen()) {
    placeTypesFile.close();
  }

  return true;
}

//**
// Reads place type instance from file.
// Boundary polygon is deprecated. Only used for backwards compatibility
//**
int PlaceTypesMainClass::loadPlaceTypeInstancesFromFile(QString filename) {
  LC::misctools::Polygon3D pgon;
  return loadPlaceTypeInstancesFromFile(filename, pgon);
}

int PlaceTypesMainClass::loadPlaceTypeInstancesFromFile(QString filename,
    LC::misctools::Polygon3D &boundaryPgon) {
  QFile placeTypesFile(filename);

  if (!placeTypesFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
    printf("ERROR: Cannot open the file %s for reading\n",
           filename.toLocal8Bit().data());
    return false;
  }

  QTextStream stream(&placeTypesFile);

  QString line;

  int curPTIdx = 0;


  //first load the file format version number
  int versionNumber;
  line = stream.readLine();

  if (!line.isNull()) {
    QStringList vList = line.split(" ");

    if (vList.size() != 2) {
      printf("ERROR: Invalid Version Number in file %s\n",
             filename.toLocal8Bit().data());;
      return false;
    }

    versionNumber = vList.at(1).toInt();
  } else {
    printf("ERROR: Invalid Version Number in file %s\n",
           filename.toLocal8Bit().data());
    return false;
  }


  //after version 5, boundary polygon is saved separately
  if (versionNumber < 5) {
    //load the bounding polygon of city
    boundaryPgon.contour.clear();

    //load the number of points
    int numBoundingPgonPts = 0;
    line = stream.readLine();

    if (!line.isNull()) {
      numBoundingPgonPts = line.toInt();
    }

    //clear bounding polygon
    for (int i = 0; i < numBoundingPgonPts; ++i) {
      line = stream.readLine();

      if (!line.isNull()) {
        QStringList attList = line.split(",");

        if (attList.size() != 3) {
          continue;
        }

        QVector3D tmpPt;
        tmpPt = QVector3D(attList.at(0).toFloat(),
                          attList.at(1).toFloat(),
                          attList.at(2).toFloat());
        boundaryPgon.contour.push_back(tmpPt);
      }
    }
  }



  //then, read place types ============================
  do {
    line = stream.readLine(); // this reads a line (QString) from the file

    if (!line.isNull()) {
      PlaceType newPt;

      if (newPt.loadPlaceType(line, versionNumber)) {

        if (curPTIdx < myPlaceTypes.size()) {
          myPlaceTypes.at(curPTIdx) = newPt;
        } else {
          myPlaceTypes.push_back(newPt);
        }

        curPTIdx++;
      }
    }
  } while (!line.isNull());

  if (placeTypesFile.isOpen()) {
    placeTypesFile.close();
  }

  return curPTIdx;
}

/**
* Makes a copy of the place type instance at index ptIdx and adds it as a new element
**/
int PlaceTypesMainClass::addPlaceType(int ptIdx) {
  if (myPlaceTypes.size() < 1) {
    return -1;
  }

  std::vector<PlaceType> pTypesCpy;

  if (ptIdx >= 0 && ptIdx < this->myPlaceTypes.size()) {
    //copy selected element
    PlaceType newPt = this->myPlaceTypes.at(ptIdx);
    //copy position and shift it a bit so that it doesn't overlap
    QVector3D copyPos;
    copyPos = myPlaceTypes.at(ptIdx).getQVector3D("pt_pt") + QVector3D(20.f, 20.0f,
              0.0f);
    newPt["pt_pt"] = copyPos;

    //copy elements before
    for (int i = 0; i <= ptIdx; ++i) {
      pTypesCpy.push_back(this->myPlaceTypes.at(i));
    }

    //add copy of selected element
    pTypesCpy.push_back(newPt);

    //copy elements after
    for (int i = ptIdx + 1; i < this->myPlaceTypes.size(); ++i) {
      pTypesCpy.push_back(this->myPlaceTypes.at(i));
    }
  } else {
    //ERROR
  }

  this->myPlaceTypes.clear();
  this->myPlaceTypes = pTypesCpy;

  return (ptIdx + 1);
}

int PlaceTypesMainClass::removePlaceType(int ptIdx) {
  if (myPlaceTypes.size() < 1) {
    return -1;
  }

  std::vector<PlaceType> pTypesCpy;

  if (ptIdx >= 0 && ptIdx < this->myPlaceTypes.size()) {
    //copy all other elements
    for (int i = 0; i < this->myPlaceTypes.size(); ++i) {
      if (i != ptIdx) {
        pTypesCpy.push_back(this->myPlaceTypes.at(i));
      }
    }
  } else {
    //ERROR
  }

  this->myPlaceTypes.clear();
  this->myPlaceTypes = pTypesCpy;

  int retIdx = std::max<float>(0, ptIdx - 1);

  return (retIdx);
}

int PlaceTypesMainClass::sendPlaceTypeToTop(int ptIdx) {
  if (myPlaceTypes.size() < 1) {
    return -1;
  }

  std::vector<PlaceType> pTypesCpy;

  if (ptIdx >= 0 && ptIdx < this->myPlaceTypes.size()) {
    //push element to front
    pTypesCpy.push_back(myPlaceTypes.at(ptIdx));

    //copy all other elements
    for (int i = 0; i < this->myPlaceTypes.size(); ++i) {
      if (i != ptIdx) {
        pTypesCpy.push_back(this->myPlaceTypes.at(i));
      }
    }
  } else {
    //ERROR
  }

  this->myPlaceTypes.clear();
  this->myPlaceTypes = pTypesCpy;

  return 0;
}

int PlaceTypesMainClass::sendPlaceTypeOneUp(int ptIdx) {
  if (myPlaceTypes.size() < 1) {
    return -1;
  }

  PlaceType pTypeCpy;

  if (ptIdx == 0) {
    return 0;
  }

  if (ptIdx > 0 && ptIdx < this->myPlaceTypes.size()) {
    //swap elements
    pTypeCpy = myPlaceTypes.at(ptIdx);
    myPlaceTypes.at(ptIdx) = myPlaceTypes.at(ptIdx - 1);
    myPlaceTypes.at(ptIdx - 1) = pTypeCpy;
  } else {
    //ERROR
  }

  return (ptIdx - 1);
}

int PlaceTypesMainClass::sendPlaceTypeOneDown(int ptIdx) {
  if (myPlaceTypes.size() < 1) {
    return -1;
  }

  PlaceType pTypeCpy;
  int lastIdx = myPlaceTypes.size() - 1;

  if (ptIdx == lastIdx) {
    return lastIdx;
  }

  if (ptIdx >= 0 && ptIdx < lastIdx) {
    //swap elements
    pTypeCpy = myPlaceTypes.at(ptIdx);
    myPlaceTypes.at(ptIdx) = myPlaceTypes.at(ptIdx + 1);
    myPlaceTypes.at(ptIdx + 1) = pTypeCpy;
  } else {
    //ERROR
  }

  return (ptIdx + 1);
}

int PlaceTypesMainClass::sendPlaceTypeToBottom(int ptIdx) {
  if (myPlaceTypes.size() < 1) {
    return -1;
  }

  std::vector<PlaceType> pTypesCpy;

  if (ptIdx >= 0 && ptIdx < this->myPlaceTypes.size()) {
    //copy all other elements
    for (int i = 0; i < this->myPlaceTypes.size(); ++i) {
      if (i != ptIdx) {
        pTypesCpy.push_back(this->myPlaceTypes.at(i));
      }
    }

    //push element to back
    pTypesCpy.push_back(myPlaceTypes.at(ptIdx));
  } else {
    //ERROR
  }

  this->myPlaceTypes.clear();
  this->myPlaceTypes = pTypesCpy;

  return (this->myPlaceTypes.size() - 1);
}


}//

