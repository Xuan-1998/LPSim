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
*		MTC Project - Geometry Project
*
*
*		@desc .
*		@author cvanegas
*
************************************************************************************************/

#include "client_geometry.h"
#ifdef B18_RUN_WITH_GUI
#include "../PM/pmMain.h"
#endif
namespace LC {


/**
* Constructor.
**/
ClientGeometry::ClientGeometry() {

  //setup road default values
  G::global()["arterial_edges_speed"] = 20000.0f; //20k/h
  G::global()["arterial_edges_width"] = 20.0f;

  G::global()["cuda_arterial_numLanes"] = 4;
  G::global()["cuda_road_numLanes"] = 2;
  G::global()["cuda_arterial_edges_speed_ms"] = 14;//~50kph

  selectedPlaceTypeIdx = 0;
}

/**
* Destructor.
**/
ClientGeometry::~ClientGeometry() {
}

void ClientGeometry::createStartingSeeds(int n) {

  printf(">>1createStartingSeeds\n");
  std::vector<PlaceType> *placeTypesPtr = &(geoPlaceTypes.myPlaceTypes);

  placeTypesPtr->clear();

  printf(">>2createStartingSeeds\n");
  PlaceType tmpPlaceType;
  printf(">>20 roads\n");
  //----- place type category ----
  tmpPlaceType["pt_category"] = 0;
  printf(">>21 roads\n");
  //----- roads -----
  printf(">>22 roads\n");
  tmpPlaceType["pt_radius"] = 600.0f;
  tmpPlaceType["pt_radius2"] = 600.0f;
  tmpPlaceType["pt_edges_curvature"] = 0;
  tmpPlaceType["pt_edges_irregularity"] =	0;
  tmpPlaceType["pt_edges_lengthU"] =	350.0f;
  tmpPlaceType["pt_edges_lengthV"] = 200.0f;
  tmpPlaceType["pt_edges_width"] = G::global().getFloat("arterial_edges_width");
  tmpPlaceType["pt_num_departing"] =	4;
  tmpPlaceType["pt_orientation"] = 0;
  printf(">>23 edges\n");

  tmpPlaceType["pt_loc_edges_curvature"] = 0;
  tmpPlaceType["pt_loc_edges_irregularity"] =	0;
  tmpPlaceType["pt_loc_edges_lengthU"] = 0.01f * 50;
  tmpPlaceType["pt_loc_edges_lengthV"] = 0.01f * 50;

  tmpPlaceType["pt_cur_edges_count"] = 0;

  printf(">>3createStartingSeeds\n");
  //----- parcels -----
  tmpPlaceType["pt_parcel_area_mean"] = 3600;
  tmpPlaceType["pt_parcel_area_deviation"] = 49;
  tmpPlaceType["pt_parcel_split_deviation"] = 0.19;
  tmpPlaceType["pt_park_percentage"] = 0.2f;

  //----- buildings -----
  tmpPlaceType["pt_parcel_setback_front"] = 15.0f;
  tmpPlaceType["pt_parcel_setback_sides"] = 2.0f;

  tmpPlaceType["pt_building_height_mean"] = 12;
  tmpPlaceType["pt_building_height_deviation"] =	90;

  tmpPlaceType["pt_building_max_frontage"] = 0;
  tmpPlaceType["pt_parcel_setback_rear"] = 0;
  tmpPlaceType["pt_building_max_depth"] = 0;

  //-------------------

  tmpPlaceType["pt_pt"] = QVector3D(0.0f,    0.0f, 0.0f);
  placeTypesPtr->push_back(tmpPlaceType);
  printf(">>4createStartingSeeds\n");
  //////////////
  G::global()["num_place_types"] = 1;
  printf("-->Initialized placetypes\n");

}//

void ClientGeometry::initBoundingPolygon(void) {
  printf(">>1initBoundingPolygon\n");
  G::boundingPolygon.clear();

  QVector3D tmpPt;
  float sqSideSz = 610.0f;

  tmpPt = QVector3D(-sqSideSz, -sqSideSz, 0.0f);
  G::boundingPolygon.push_back(tmpPt);

  tmpPt = QVector3D(-sqSideSz,  sqSideSz, 0.0f);
  G::boundingPolygon.push_back(tmpPt);

  tmpPt = QVector3D(sqSideSz,  sqSideSz, 0.0f);
  G::boundingPolygon.push_back(tmpPt);

  tmpPt = QVector3D(sqSideSz, -sqSideSz, 0.0f);
  G::boundingPolygon.push_back(tmpPt);
  printf(">>2initBoundingPolygon\n");
  createStartingSeeds(0);
  printf(">>3initBoundingPolygon\n");
}


bool ClientGeometry::exportRoads(QString &filename) {
  std::cout << "Exporting raods to OSM file..." << std::endl;

  QFile exportFile(filename);

  if (!exportFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
    std::cout << "ERROR: Cannot open the file " << filename.toLocal8Bit().data()
              << " for writing\n";
    return false;
  }

  QTextStream stream(&exportFile);

  this->roadGraph.writeRoadNetworkToOSM(stream);

  exportFile.close();

  std::cout << "Done!" << std::endl;

  return true;
}

/*bool ClientGeometry::exportParcels(QString &filename)
{
std::cout << "Exporting parcels to OBJ file..." << std::endl;

QFile exportFile(filename);

if (!exportFile.open(QIODevice::WriteOnly | QIODevice::Text))
{
std::cout << "ERROR: Cannot open the file " << filename.toLocal8Bit().data()
<< " for writing\n";
return false;
}

QTextStream stream( &exportFile );

Block::parcelGraphVertexIter vi, viEnd;
for(int i=0; i < this->geoZone.blocks.size(); ++i){
for(boost::tie(vi, viEnd) = boost::vertices(geoZone.blocks[i].myParcels);
vi != viEnd; ++vi)
{
geoZone.blocks[i].myParcels[*vi].writeParcelContourToOBJ(stream);
}
}

exportFile.close();

std::cout << "Done!" << std::endl;

return true;

}

bool ClientGeometry::exportBuildings(QString &filename)
{
std::cout << "Exporting buildings to OBJ file..." << std::endl;

QFile exportFile(filename);

if (!exportFile.open(QIODevice::WriteOnly | QIODevice::Text))
{
std::cout << "ERROR: Cannot open the file " << filename.toLocal8Bit().data()
<< " for writing\n";
return false;
}

QTextStream stream( &exportFile );

Block::parcelGraphVertexIter vi, viEnd;
for(int i=0; i < this->geoZone.blocks.size(); ++i){
for(boost::tie(vi, viEnd) = boost::vertices(geoZone.blocks[i].myParcels);
vi != viEnd; ++vi)
{
geoZone.blocks[i].myParcels[*vi].writeParcelBuildingToOBJ(stream);
}
}

exportFile.close();

std::cout << "Done!" << std::endl;

return true;
}*/

#ifdef B18_RUN_WITH_GUI
bool ClientGeometry::generateGeometry(int flag) { //, QGLWidget* glWidget){
  bool res;

  //srand(246087); //just some ranomd seed nacho made up

  switch (flag) {
  case kJustRoads:
  case kStartFromRoads:
    res = ProceduralMachine::generateRoadNetwork(
            this->geoPlaceTypes,
            this->roadGraph,
            this->geoZone.blocks,
            this,
            //glWidget,
            flag);
    break;

  case kStartFromBlocks:
    res = ProceduralMachine::generateBlocks(
            this->geoPlaceTypes,
            this->roadGraph,
            this->geoZone.blocks,
            this//,
            //glWidget
          );
    break;

  case kStartFromParcels:
    res = ProceduralMachine::generateParcels(
            this->geoPlaceTypes,
            this->geoZone.blocks,
            this//,
            //glWidget
          );
    break;

  case kStartFromBuildings:
    res = ProceduralMachine::generateBuildings(
            this->geoPlaceTypes,
            this->geoZone.blocks//,
            //glWidget
          );
    break;

  default:
    break;
  }

  /*if(heightOffsetOview_trees_render){
                        res = ProceduralVegetationMachine::generateVegetation(//this->geoTerrain,
                                this->geoPlaceTypes,
                                this->geoZone.blocks,
                                this->geoVegetation);
                }*/

  return res;
}
#endif




void ClientGeometry::adaptToTerrain(void) {

  //std::cout << "undefined \n";
}



}


