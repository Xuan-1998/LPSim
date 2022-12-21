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

#include "VBOVegetation.h"

#include "VBORenderManager.h"


namespace LC {

VBOVegetation::VBOVegetation(void) {
}


VBOVegetation::~VBOVegetation(void) {
}

ModelSpec addTree(QVector3D pos) {
  ModelSpec tree;

  tree.transMatrix.setToIdentity();
  tree.transMatrix.translate(pos);
  tree.transMatrix.rotate(pos.x()*pos.y(), 0.0f, 0.0f, 1.0f); //random
  tree.transMatrix.scale(1.0f + (((0.5f * qrand()) / RAND_MAX)) - 0.5f);
  tree.colors.resize(2);
  //trunk
  tree.colors[1] = QVector3D(0.27f, 0.22f, 0.22f);
  tree.colors[1] = tree.colors[1] + QVector3D(1.0, 1.0,
                   1.0f) * ((0.2f * qrand()) / RAND_MAX - 0.1f);
  //leaves
  int treeColor = qrand() % 100;
  int desvLeaf = qrand() % 40;

  if (treeColor >= 0 && treeColor <= 46) {
    tree.colors[0] = QVector3D(115 - desvLeaf, 169 - desvLeaf,
                               102 - desvLeaf);  //green to dark
  }

  if (treeColor >= 47 && treeColor <= 92) {
    tree.colors[0] = QVector3D(69 - desvLeaf / 2.0f, 109 - desvLeaf / 2.0f,
                               72 - desvLeaf / 2.0f);  //green to dark
  }

  if (treeColor >= 93 && treeColor <= 94) {
    tree.colors[0] = QVector3D(155 - desvLeaf / 3.0f, 124 - desvLeaf / 3.0f,
                               24 - desvLeaf / 3.0f);  //yellow to dark
  }

  if (treeColor >= 95 && treeColor <= 96) {
    tree.colors[0] = QVector3D(96 - desvLeaf / 4.0f, 25 - desvLeaf / 4.0f,
                               33 - desvLeaf / 4.0f);  //red to dark
  }

  if (treeColor >= 97 && treeColor <= 100) {
    tree.colors[0] = QVector3D(97 - desvLeaf / 2.0f, 69 - desvLeaf / 2.0f,
                               58 - desvLeaf / 2.0f);  //grey to dark
  }

  tree.colors[0] /= 255.0f;
  tree.type = 0;

  return tree;
}

ModelSpec addStreetLap(QVector3D pos, QVector3D contourDir) {


  ModelSpec stEl;

  stEl.transMatrix.setToIdentity();
  stEl.transMatrix.translate(pos);
  QVector3D perP = QVector3D::crossProduct(contourDir, QVector3D(0, 0, 1.0f));

  float rotAngle = atan2(perP.y(),
                         perP.x()) * 57.2957795f; //rad to degrees (angle to rotate will be the tan since we compare respect 1,0,0)
  stEl.transMatrix.rotate(rotAngle, 0.0f, 0.0f, 1.0f);

  stEl.colors.resize(1);
  //body xolor
  stEl.colors[0] = QVector3D(0.35f, 0.35f, 0.35f);
  stEl.type = 1;

  return stEl;
}


bool VBOVegetation::generateVegetation(VBORenderManager &rendManager,
                                       PlaceTypesMainClass &placeTypesIn, std::vector< Block > &blocks) {

  std::cout << "Generating vegetation...";

  //for(int i=0; i<blocks.size(); ++i){
  //blocks[i].streetElementInfo.clear();
  //}
  rendManager.removeAllStreetElementName("streetLamp");
  rendManager.removeAllStreetElementName("tree");

  float treesPerSqMeter = 0.002f; //used for trees in parcels
  float distanceBetweenTrees = 31.0f;//23 N 15.0f; //used for trees along streets
  float minTreeHeight = 10.0f;
  float maxTreeHeight = 20.0f;
  float minTreeRadToHeightRatio = 0.3f;
  float maxTreeRadToHeightRatio = 0.5f;

  float distanceBetweenStreetLamps = 43.0f;//30 from google but too close

  int numTreesInParcel;

  QTime tim;
  tim.start();
  float rad, height;
  float parcelBBoxArea;
  QVector3D pos;


  float xmin, xmax, ymin, ymax;
  float xpos, ypos, zpos;

  LC::misctools::BBox parcelBBox;

  Block::parcelGraphVertexIter vi, viEnd;

  //generate trees in parcels
  for (int i = 0; i < blocks.size(); ++i) {

    srand(blocks.at(i).randSeed);

    for (boost::tie(vi, viEnd) = boost::vertices(blocks[i].myParcels);
         vi != viEnd; ++vi) {
      //for(int pN=0;pN<blocks[i].myParcels.size();pN++){
      if ((blocks[i].myParcels[*vi]).parcelType == PAR_PARK) {

        parcelBBox = (blocks[i].myParcels[*vi]).bbox;

        xmin = parcelBBox.minPt.x();
        xmax = parcelBBox.maxPt.x();
        ymin = parcelBBox.minPt.y();
        ymax = parcelBBox.maxPt.y();

        parcelBBoxArea = (xmax - xmin) * (ymax - ymin);

        numTreesInParcel = (int)(parcelBBoxArea * treesPerSqMeter);

        boost::geometry::ring_type<LC::misctools::Polygon3D>::type bg_loop;
        //boost::geometry::point_2d bg_testPt;
        typedef boost::geometry::model::d2::point_xy<double> point_2d;
        point_2d bg_testPt;
        boost::geometry::assign(bg_loop,
                                (blocks[i].myParcels[*vi]).parcelContour.contour);

        for (int i = 0; i < numTreesInParcel; ++i) {
          pos.setX(LC::misctools::genRand(xmin, xmax));
          pos.setY(LC::misctools::genRand(ymin, ymax));
          pos.setZ(0.0f);


          bg_testPt.x(pos.x());
          bg_testPt.y(pos.y());

          //blocks[i].streetElementInfo.push_back(addTree(pos));
          rendManager.addStreetElementModel("tree", addTree(pos));

        }
      }
    }
  }

  //generate trees along streets
  float blockSetback;
  QVector3D ptThis, ptNext;
  QVector3D segmentVector;
  float segmentLength;
  int numTreesAlongSegment;
  std::vector<Vector3D> *contourPtr;

  for (int i = 0; i < blocks.size(); ++i) {

    srand(blocks.at(i).randSeed);


    blockSetback = placeTypesIn.myPlaceTypes.at(
                     0).getFloat("pt_parcel_setback_front");


    blockSetback = 5.0f;

    if (blockSetback >= 4.0f) { //add trees along road only if there's enough space

      contourPtr = &(blocks.at(i).blockContour.contour);

      for (int j = 0; j < contourPtr->size(); ++j) {
        ptThis = contourPtr->at(j);
        ptNext = contourPtr->at((j + 1) % contourPtr->size());
        segmentVector = ptNext - ptThis;
        segmentLength = segmentVector.length();
        segmentVector /= segmentLength;

        QVector3D perpV = QVector3D::crossProduct(segmentVector, QVector3D(0, 0, 1));
        ptThis = ptThis - perpV * 5.5f;

        // Trees
        float distFromSegmentStart = 0.0f;

        while (true) {
          distFromSegmentStart += distanceBetweenTrees * (0.8f + (0.4f * qrand() /
                                  RAND_MAX));

          if (distFromSegmentStart > segmentLength) {
            break;
          }

          pos = ptThis + segmentVector * distFromSegmentStart;

          //blocks[i].streetElementInfo.push_back(addTree(pos));
          rendManager.addStreetElementModel("tree", addTree(pos));

        }

        // StreetLamps
        /*distFromSegmentStart = 0.0f;
        while(true){
        distFromSegmentStart += distanceBetweenStreetLamps;
        if(distFromSegmentStart > segmentLength){
        break;
        }
        pos = ptThis + segmentVector*distFromSegmentStart;
        addStreetLap(blocks[i].streetElementInfo,pos,segmentVector);
        }*/
        int numStreetLamps = ceil((segmentLength - 1.0f) /
                                  distanceBetweenStreetLamps); //-1.0 to leave space at the beginning

        if (numStreetLamps < 2) {
          numStreetLamps = 2;
        }

        float distanceBetweenPosts = (segmentLength - 1.0f) / (numStreetLamps - 1);

        for (int i = 0; i < numStreetLamps - 1;
             i++) { //not in the end (avoid two in the corner)
          pos = ptThis + segmentVector * (0.5f + i * distanceBetweenPosts);

          //blocks[i].streetElementInfo.push_back(addStreetLap(pos,segmentVector));
          rendManager.addStreetElementModel("streetLamp", addStreetLap(pos,
                                            segmentVector));
        }

      }
    }
  }

  std::cout << "\t" << tim.elapsed() << " ms\n";
  return true;
}

}
