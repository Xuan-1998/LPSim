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

#include "LC_GLWidget3D.h"
#include "LC_UrbanMain.h"
#include <QMouseEvent>
#include <QDebug>
#include <QKeyEvent>
#include "qmath.h"
#include "glu.h"

#include "roadGraphDynameqLoader.h"
#include "roadGraphB2018Loader.h"

//test
#include "bTraffic/bGenerateTest.h"
#include "traffic/b18TestSimpleRoadAndOD.h"

#define FAR_DIST 2500.0f

namespace LC {

LCGLWidget3D::LCGLWidget3D(QWidget *parent) : QGLWidget(QGLFormat(
        QGL::SampleBuffers), parent) {
  urbanMain = (LCUrbanMain *)parent;

  myCam = new Camera3D();
  spaceRadius = 5000.0;
  farPlaneToSpaceRadiusFactor = 5.0f; //N 5.0f

  rotationSensitivity = 0.4f;
  zoomSensitivity = 0.6f;

  controlPressed = false;
  shiftPressed = false;
  altPressed = false;
  keyMPressed = false;
  keyLPressed = false;

  shadowEnabled = true;
  keyHold = 0;

}

LCGLWidget3D::~LCGLWidget3D() {
  delete myCam;
}

QSize LCGLWidget3D::minimumSizeHint() const {
  return QSize(200, 200);
}

QSize LCGLWidget3D::sizeHint() const {
  return QSize(400, 400);
}

void LCGLWidget3D::resetMyCam() {
  myCam->resetCamera();
  updateGL();
}

void LCGLWidget3D::mousePressEvent(QMouseEvent *event) {
  this->setFocus();// force to recover focus from sliders
  mouseMoved = false;

  QVector3D mouse3DPos;
  mouseTo3D(event->x(), event->y(), &mouse3DPos);
  //printf("Pos2D %d %d --> Pos3D %f %f %f\n",event->x(), event->y(),mouse3DPos.x(),mouse3DPos.y(),mouse3DPos.z());
  mouse3DPos.setZ(0);
  lastPos = event->pos();

  ///////////////////////////////////////////////////////
  //
  if (keyMPressed == true) {
    // layer edition
    if (urbanMain->ui.layer_enableCheckBox->isChecked() == true) {
      float change = urbanMain->ui.layer_changeSlider->value();

      if (event->buttons() & Qt::RightButton) {
        change = -change;
      }

      int layerNum = urbanMain->ui.layer_numberSpinBox->value();
      vboRenderManager.layers.updateLayer(layerNum, mouse3DPos, change);
      //infoLayers.updateLayer(LC::misctools::Global::global()->render_layer_number,mouse3DPos,LC::misctools::Global::global()->render_layer_value_brush);
      updateGL();
      return;
    }

    float change = urbanMain->ui.terrainPaint_changeSlider->value() * 0.003f;
    float radi = urbanMain->ui.terrainPaint_sizeSlider->value() * 0.01f;

    if (event->buttons() & Qt::RightButton) {
      change = -change;
    }

    if (event->buttons() & Qt::MiddleButton) {
      change = FLT_MAX; //hack: flat terrain
    }

    //mainWin->urbanGeometry->vboRenderManager->addValue(pos.x(), pos.y(), change);
    float xM = 1.0f - (vboRenderManager.side / 2.0f - mouse3DPos.x()) /
               vboRenderManager.side;
    float yM = 1.0f - (vboRenderManager.side / 2.0f - mouse3DPos.y()) /
               vboRenderManager.side;
    vboRenderManager.vboTerrain.updateTerrain(xM, yM, change, radi); //rad,change);
    //vboRenderManager.vboTerrain.updateTerrainNewValue(xM,yM,8.0f,radi);//rad,change);
    shadow.makeShadowMap(this);
    updateGL();


    return;
  }

  ///////////////////////////////////////////////////////
  //
  if ((urbanMain->ui.proceduralModelingCheckBox->isChecked() == true) &&
      //just edit if procedural mode
      (event->buttons() & Qt::LeftButton)) {
    const float mouseThreshold = 25.0f;

    //QVector3D mouse3DPos;
    //mouseTo3D(event->x(), event->y(), &mouse3DPos);
    int closestIdx;
    int closestIdxPt;

    std::vector<QVector3D> placeTypesPositions;
    std::vector<QVector3D> placeTypesExtHandlersPositions;
    std::vector<QVector3D> placeTypesExtHandlers2Positions;
    std::vector<QVector3D> placeTypesUHandlersPositions;
    std::vector<QVector3D> placeTypesVHandlersPositions;

    //printf("pl num %d\n",cg.geoPlaceTypes.myPlaceTypes.size());
    for (int i = 0; i < cg.geoPlaceTypes.myPlaceTypes.size(); ++i) {

      if (i == cg.selectedPlaceTypeIdx) {
        placeTypesExtHandlersPositions.push_back(
          cg.geoPlaceTypes.myPlaceTypes[i].getExternalHandlerPos());
        placeTypesExtHandlers2Positions.push_back(
          cg.geoPlaceTypes.myPlaceTypes[i].getExternalHandler2Pos());
        placeTypesUHandlersPositions.push_back(
          cg.geoPlaceTypes.myPlaceTypes[i].getlengthUHandlerPos());
        placeTypesVHandlersPositions.push_back(
          cg.geoPlaceTypes.myPlaceTypes[i].getlengthVHandlerPos());
      }

      placeTypesPositions.push_back(
        cg.geoPlaceTypes.myPlaceTypes[i].getQVector3D("pt_pt"));
      //qDebug()<<placeTypesPositions.back();

    }

    //snap point to place types
    float minDist = FLT_MAX;
    int snapState = M_NORMAL;

    float distToCtr = LC::misctools::getClosestPointInArrayXY(placeTypesPositions,
                      mouse3DPos, closestIdxPt);

    if (distToCtr < minDist) {
      snapState = M_PTYPE_CENTER_EDITION;
      minDist = distToCtr;
    }

    float distToExtHndlr = LC::misctools::getClosestPointInArrayXY(
                             placeTypesExtHandlersPositions, mouse3DPos, closestIdx);

    if (distToExtHndlr < minDist) {
      snapState = M_PTYPE_EXT_HANDLER_EDITION;
      minDist = distToExtHndlr;
    }

    float distToExtHndlr2 = LC::misctools::getClosestPointInArrayXY(
                              placeTypesExtHandlers2Positions, mouse3DPos, closestIdx);

    if (distToExtHndlr2 < minDist) {
      snapState = M_PTYPE_EXT_HANDLER2_EDITION;
      minDist = distToExtHndlr2;
    }

    float distToUHndlr = LC::misctools::getClosestPointInArrayXY(
                           placeTypesUHandlersPositions, mouse3DPos, closestIdx);

    if (distToUHndlr < minDist) {
      snapState = M_PTYPE_UDIST_HANDLER_EDITION;
      minDist = distToUHndlr;
    }

    float distToVHndlr = LC::misctools::getClosestPointInArrayXY(
                           placeTypesVHandlersPositions, mouse3DPos, closestIdx);

    if (distToVHndlr < minDist) {
      snapState = M_PTYPE_VDIST_HANDLER_EDITION;
      minDist = distToVHndlr;
    }

    /*float distToBoundingPolygon = LC::misctools::getClosestPointInArrayXY(
                terrainVBO.boundingPolygon, mouse3DPos, closestIdx);
            if(distToBoundingPolygon < minDist) {
                snapState = M_BOUNDING_POLYGON_EDITION;
                minDist = distToBoundingPolygon;
            }*/

    if (minDist < mouseThreshold) {
      G::global()["mouseState"] = snapState;

      if (snapState != M_BOUNDING_POLYGON_EDITION) {
        if (snapState == M_PTYPE_CENTER_EDITION) {
          cg.selectedPlaceTypeIdx = closestIdxPt;
          printf("Select other pT\n");
          //clientMain->updatePMWidgetsWithSeedParameters();
        }
      } /*else { //if we're in bounding polygon mode

                        if(controlPressed){

                            mouseState =
                                M_NORMAL;

                            QVector3D tmpMidPt;
                            int boundingPgonSz = terrainVBO.boundingPolygon.size();
                            int prevIdx, nextIdx;
                            prevIdx = (closestIdx - 1 + boundingPgonSz) % boundingPgonSz;
                            tmpMidPt =  0.5f*(
                                terrainVBO.boundingPolygon.at(prevIdx) +
                                terrainVBO.boundingPolygon.at(closestIdx) );

                            terrainVBO.boundingPolygon.insert(
                                terrainVBO.boundingPolygon.begin() + closestIdx,
                                tmpMidPt);
                            terrainVBO.updateTerrain();
                            snapState = M_NORMAL;

                        } else if(altPressed){

                            mouseState =
                                M_NORMAL;

                            terrainVBO.boundingPolygon.erase(
                                terrainVBO.boundingPolygon.begin() + closestIdx);
                            terrainVBO.updateTerrain();
                            closestIdx = qMax( closestIdx - 1, 0);

                        } else {

                            terrainVBO.selectedBoundingPolygonVertexIdx= closestIdx;
                        }

                    }*/
    } else {
      G::global()["mouseState"] = M_NORMAL;
    }

    printf("minDist %f snapState %d\n", minDist, snapState);
  }
}

void LCGLWidget3D::mouseReleaseEvent(QMouseEvent *event) {
  event->ignore();
  int mouseState = G::global().getInt("mouseState");

  if (mouseState == M_PTYPE_CENTER_EDITION ||
      mouseState == M_PTYPE_EXT_HANDLER_EDITION ||
      mouseState == M_PTYPE_EXT_HANDLER2_EDITION ||
      mouseState == M_PTYPE_UDIST_HANDLER_EDITION ||
      mouseState == M_PTYPE_VDIST_HANDLER_EDITION ||
      mouseState == M_BOUNDING_POLYGON_EDITION) {
    mouseState = M_NORMAL;

    setCursor(Qt::WaitCursor);

    if (mouseMoved) {
      generateGeometry(ClientGeometry::kStartFromRoads);
    }
  }

  setCursor(Qt::ArrowCursor);
  updateGL();
}//

void LCGLWidget3D::mouseMoveEvent(QMouseEvent *event) {

  int mouseState = G::global().getInt("mouseState");
  //printf("moseState %d\n",mouseState);
  mouseMoved = true;

  ////////////////////////////////////////////////
  // 2. PLACE TYPE EDITOR
  if (mouseState ==
      M_PTYPE_CENTER_EDITION) {
    QVector3D midPosition;

    if (mouseTo3D(event->x(), event->y(), &midPosition)) {
      if (cg.selectedPlaceTypeIdx >= 0
          &&
          cg.selectedPlaceTypeIdx <
          cg.geoPlaceTypes.myPlaceTypes.size()) {
        midPosition.setZ(0.0f);

        /*//check if target vertex is within bounding polygon
            if( !LC::misctools::isPointWithinLoop(
                terrainVBO.boundingPolygon,
                midPosition) ){
                    return;
            }*/

        cg.geoPlaceTypes.myPlaceTypes[cg.selectedPlaceTypeIdx]["pt_pt"] = midPosition;
        //cg.geoPlaceTypes.myPlaceTypes[cg.selectedPlaceTypeIdx].setAttPt(midPosition);
      }
    }

    cg.geoPlaceTypes.myPlaceTypes[cg.selectedPlaceTypeIdx].updateBoundingRectangle();

    generateGeometry(ClientGeometry::kJustRoads);
    //generateGeometry(ClientGeometry::kStartFromRoads);

    G::global()["view_arterials_edges_render"] = true;
    setCursor(Qt::ArrowCursor);

    updateGL();
    //std::cout << "move2 ";
    return;
  }

  ///////////
  // PLACE TYPE EXTERNAL HANDLER EDITION
  if (mouseState ==
      M_PTYPE_EXT_HANDLER_EDITION
      ||
      mouseState ==
      M_PTYPE_EXT_HANDLER2_EDITION) {
    QVector3D midPosition;

    if (mouseTo3D(event->x(), event->y(), &midPosition)) {
      if (cg.selectedPlaceTypeIdx >= 0
          &&
          cg.selectedPlaceTypeIdx <
          cg.geoPlaceTypes.myPlaceTypes.size()) {
        midPosition.setZ(0.0f);

        QVector3D handlerMinusCenter =
          midPosition -
          cg.geoPlaceTypes.myPlaceTypes[cg.selectedPlaceTypeIdx].getQVector3D("pt_pt");

        float rad = handlerMinusCenter.length();
        float orient = atan2(handlerMinusCenter.y(), handlerMinusCenter.x());

        if (mouseState ==
            M_PTYPE_EXT_HANDLER_EDITION) {

          cg.geoPlaceTypes.myPlaceTypes[cg.selectedPlaceTypeIdx]["pt_radius"] = rad;
          cg.geoPlaceTypes.myPlaceTypes[cg.selectedPlaceTypeIdx]["pt_orientation"] =
            orient;
        } else if (mouseState ==
                   M_PTYPE_EXT_HANDLER2_EDITION) {
          cg.geoPlaceTypes.myPlaceTypes[cg.selectedPlaceTypeIdx]["pt_radius2"] = rad;
          cg.geoPlaceTypes.myPlaceTypes[cg.selectedPlaceTypeIdx]["pt_orientation"] =
            (orient - 0.5f * M_PI);
        }
      }
    }

    cg.geoPlaceTypes.myPlaceTypes[cg.selectedPlaceTypeIdx].updateBoundingRectangle();

    generateGeometry(ClientGeometry::kJustRoads);
    G::global()["view_arterials_edges_render"] = true;
    setCursor(Qt::ArrowCursor);
    updateGL();
    return;
  }


  ///////////
  // PLACE TYPE U/V DIST HANDLER EDITION
  if ((mouseState ==
       M_PTYPE_UDIST_HANDLER_EDITION) ||
      (mouseState ==
       M_PTYPE_VDIST_HANDLER_EDITION)) {
    QVector3D midPosition;

    if (mouseTo3D(event->x(), event->y(), &midPosition)) {
      if (cg.selectedPlaceTypeIdx >= 0
          &&
          cg.selectedPlaceTypeIdx <
          cg.geoPlaceTypes.myPlaceTypes.size()) {
        midPosition.setZ(0.0f);

        QVector3D handlerMinusCenter =
          midPosition -
          cg.geoPlaceTypes.myPlaceTypes[cg.selectedPlaceTypeIdx].getQVector3D("pt_pt");

        float rad = handlerMinusCenter.length();

        if (mouseState ==
            M_PTYPE_UDIST_HANDLER_EDITION) {
          cg.geoPlaceTypes.myPlaceTypes
          [cg.selectedPlaceTypeIdx]["pt_edges_lengthU"] = rad;
        } else if (mouseState ==
                   M_PTYPE_VDIST_HANDLER_EDITION) {
          cg.geoPlaceTypes.myPlaceTypes
          [cg.selectedPlaceTypeIdx]["pt_edges_lengthV"] = rad;
        }
      }
    }

    cg.geoPlaceTypes.myPlaceTypes[cg.selectedPlaceTypeIdx].updateBoundingRectangle();

    generateGeometry(ClientGeometry::kJustRoads);
    G::global()["view_arterials_edges_render"] = true;
    setCursor(Qt::ArrowCursor);

    updateGL();
    return;
  }


  ///////////////////////////////////////////////////////
  //
  QVector3D mouse3DPos;
  mouseTo3D(event->x(), event->y(), &mouse3DPos);
  float dx = (float)(event->x() - lastPos.x());
  float dy = (float)(event->y() - lastPos.y());
  lastPos = event->pos();

  if (keyMPressed == true) { // update renderManager
    vboRenderManager.mousePos3D = mouse3DPos;
    //printf("----------------\n----------\n");
    return;
  }

  myCam->motion(dx, dy,
                keyLPressed); //if M pressed--> light Otherwise-->Move camera

  if (keyLPressed == true) { //update shadow map
    shadow.makeShadowMap(this);
  } else {
    updateCamera();
  }

  updateGL();
}

void LCGLWidget3D::initializeGL() {
  ////////////////////////////////////////
  //---- GLEW extensions ----
  GLenum err = glewInit();

  if (GLEW_OK != err) { // Problem: glewInit failed, something is seriously wrong.
    qDebug() << "Error: " << glewGetErrorString(err);
  }

  qDebug() << "Status: Using GLEW " << glewGetString(GLEW_VERSION);
  //while ((err = glGetError()) != GL_NO_ERROR) qDebug() << "**4ERROR INIT: OpenGL-->" << err << endl;

  if (glewIsSupported("GL_VERSION_4_2")) {
    printf("Ready for OpenGL 4.2\n");
  } else {
    printf("OpenGL 4.2 not supported\n");
    exit(1);
  }

  //const GLubyte *text
  //glGetString(GL_VERSION);
  //printf("VERSION: %s\n", text);
  //while ((err = glGetError()) != GL_NO_ERROR) qDebug() << "**3ERROR INIT: OpenGL-->" << err << endl;

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);

  glEnable(GL_TEXTURE_2D);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);

  glTexGenf(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
  glTexGenf(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
  glDisable(GL_TEXTURE_2D);

  glEnable(GL_TEXTURE_3D);
  glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glDisable(GL_TEXTURE_3D);

  glEnable(GL_TEXTURE_2D_ARRAY);
  glTexParameterf(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER,
                  GL_LINEAR_MIPMAP_LINEAR);
  glTexParameterf(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER,
                  GL_LINEAR_MIPMAP_LINEAR);
  glDisable(GL_TEXTURE_2D_ARRAY);

  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);

  ////////////////////////////////
  printf("vboRenderManager\n");
  vboRenderManager.init();
  updateCamera();

  printf("initBoundingPolygon\n");
  cg.initBoundingPolygon();//init poli

  printf("initShadow\n");
  shadow.initShadow(vboRenderManager.program, this);
  shadow.makeShadowMap(this);

  // load roads
  printf("load roads\n");

  QSettings settings(QApplication::applicationDirPath() +
                     "/command_line_options.ini", QSettings::IniFormat);
  bool useBasicTest = settings.value("USE_BASIC_TEST",
                                     false).toBool(); // false = B2018; true = basic intersection

  if (useBasicTest) {
    const float deltaTime = 0.5f;
    const float startDemandH = 7.30f;
    const float endDemandH = 9.00f;
    B18TestSimpleRoadAndOD::generateTest(cg.roadGraph,
                                         b18TrafficSimulator.trafficPersonVec, startDemandH, endDemandH, this);
    b18TrafficSimulator.initSimulator(deltaTime, &cg.roadGraph, urbanMain);
  } else {
    bool useFullB18Network = settings.value("USE_FULL_B2018_NETWORK",
                                            false).toBool();
    RoadGraphB2018::loadB2018RoadGraph(cg.roadGraph, useFullB18Network);
    // To remove the gl dependency of the loader.
    cg.geoZone.blocks.clear();
    vboRenderManager.removeAllStreetElementName("tree");
    vboRenderManager.removeAllStreetElementName("streetLamp");
    float sqSideSz = G::boundingPolygon[0].x();
    vboRenderManager.changeTerrainDimensions(sqSideSz * 2 + 400.0f, 200);
    QString sfo_path("data/b2018.png");
    vboRenderManager.vboTerrain.loadTerrain(sfo_path);
  }

  printf("Edge\n");

  printf("___ #Edges %d #Vertex %d \n",
         boost::num_edges(cg.roadGraph.myRoadGraph_BI),
         boost::num_vertices(cg.roadGraph.myRoadGraph_BI));
  //test example
  //GenerateTest::generateTest(cg.roadGraph, bTrafficSimulator.people, this);


  // set 2D mode
  printf("Render\n");
  G::global()["render_mode"] = 1;
  changeRender2D3D();//set 2D

  //test
  //VBOText::renderText(vboRenderManager,"a",QVector3D(0,0,0),QVector3D(1,0,0),QVector3D(0,0,1),50);
}

void LCGLWidget3D::resizeGL(int width, int height) {
  updateCamera();
}

void LCGLWidget3D::paintGL() {

  qglClearColor(QColor(0xFF, 0xFF, 0xFF)); //white
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  drawScene(0);
}

void LCGLWidget3D::drawScene(int drawMode) {
  //printf("drawScene\n");
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);


  // LAYER MODE
  if (G::global().getInt("render_mode") == 2) {
    //printf("render_mode 2\n");
    glUniform1i(glGetUniformLocation(vboRenderManager.program, "shadowState"),
                0); //SHADOW: Disable
    vboRenderManager.vboTerrain.render(vboRenderManager);

    if (keyMPressed == true) {
      vboRenderManager.layers.drawDrawingCircle(vboRenderManager);
    }
  }

  // 2D MODE
  if (G::global().getInt("render_mode") == 1) {
    glUniform1i(glGetUniformLocation(vboRenderManager.program, "shadowState"),
                0); //SHADOW: Disable
    //printf("render_mode 1\n");
    vboRenderManager.vboTerrain.render(vboRenderManager);

    if (urbanMain->ui.render_2DlinesCheckBox->isChecked()) {
      // LINES MODE
      glLineWidth(6.0f);
      vboRenderManager.renderStaticGeometry(QString("roads_2D_segments5"));
      glLineWidth(4.0f);
      vboRenderManager.renderStaticGeometry(QString("roads_2D_segments2"));
    } else {
      // STREET MAP
      vboRenderManager.renderStaticGeometry(QString("road_2D_GM"));
      vboRenderManager.renderStaticGeometry(QString("road_2D_GM_int"));
    }

    // road labels
    if (urbanMain->ui.TB_roadLabelButton->isChecked() == true) {
      printf("renderLabels\n");
      VBORoadLabels::renderRoadLabels(vboRenderManager);
    }

    // routes
    if (urbanMain->ui.b18RenderRoutesCheckBox->isChecked() == true) {
      // TODO (igacarid) render routes b18TrafficSimulator.renderRoutes(vboRenderManager);
    }

    ////////////////////////////
    // TRAFFIC
    if (urbanMain->ui.b18RenderSimulationCheckBox->isChecked() == true) {
      //printf(">>GLWidget draw traffic\n");
      b18TrafficSimulator.render(vboRenderManager);
      //printf("<<GLWidget draw traffic\n");
    }

    vboRenderManager.renderStaticGeometry(QString("sky"));
  }

  // 3D MODE
  if (G::global().getInt("render_mode") == 0) {
    //printf("render_mode 0\n");
    // NORMAL
    if (drawMode == 0) {
      glUniform1i(glGetUniformLocation(vboRenderManager.program, "shadowState"),
                  0); //SHADOW: Disable
      vboRenderManager.renderStaticGeometry(QString("sky"));
      vboRenderManager.vboWater.render(vboRenderManager);


      if (shadowEnabled) {
        glUniform1i(glGetUniformLocation(vboRenderManager.program, "shadowState"),
                    1);  //SHADOW: Render Normal with Shadows
      }

      vboRenderManager.vboTerrain.render(vboRenderManager);

      vboRenderManager.renderStaticGeometry(QString("roads"));
      vboRenderManager.renderStaticGeometry(QString("sidewalk"));
      vboRenderManager.renderStaticGeometry(QString("building"));

      //vboRenderManager.renderAllStreetElementName("tree");
      //vboRenderManager.renderAllStreetElementName("streetLamp");

      glUniform1i(glGetUniformLocation(vboRenderManager.program, "shadowState"),
                  0); //SHADOW: Disable
      vboRenderManager.vboGUI.renderVBOGUI(vboRenderManager);
    }

    // SHADOWS
    if (drawMode == 1) {
      glUniform1i(glGetUniformLocation(vboRenderManager.program, "shadowState"),
                  2); // SHADOW: From light

      vboRenderManager.vboTerrain.render(vboRenderManager);

      //vboRenderManager.renderStaticGeometry(QString("roads"));//(to avoid shadows of flying roads)
      vboRenderManager.renderStaticGeometry(QString("sidewalk"));
      vboRenderManager.renderStaticGeometry(QString("building"));

      //vboRenderManager.renderAllStreetElementName("tree");
      //vboRenderManager.renderAllStreetElementName("streetLamp");
    }
  }

  /////////////////////

  //VBOText::renderText(vboRenderManager,"label",QVector3D(0,0,50),QVector3D(1,0,0),QVector3D(0,1,0),400);
  //VBOText::renderText(vboRenderManager,"label1",QVector3D(0,0,50),QVector3D(1,0,0),QVector3D(0,1,0),400,true);
  //VBOText::renderText(vboRenderManager,"a",QVector3D(0,0,54),QVector3D(1,0,0),QVector3D(0,1,0),2);

  //disable depth buffer
  glDepthFunc(GL_ALWAYS);
  glDisable(GL_TEXTURE_2D);
  //printf("<<Flush\n");
  glFlush();
}

void LCGLWidget3D::keyPressEvent(QKeyEvent *e) {
  //printf("k %d\n",keyHold);
  keyHold = std::min<int>(keyHold + 1, 50);
  float factor = (keyHold / 50.0f);
  factor = 1.0f + 8.0f * factor * factor; //quad behavior
  //printf("factor %f\n",factor);
  float sensitivityFactor;

  shiftPressed = false;
  controlPressed = false;
  altPressed = false;
  keyMPressed = false;
  vboRenderManager.editionMode = false;
  keyLPressed = false;

  QTime timer;
  QString name;
  QString numbS;
  int numb;
  int elapsed;//define variables for switch

  switch (e->key()) {
  case Qt::Key_Escape:
    this->parentWidget()->close();
    break;

  case Qt::Key_Shift:
    shiftPressed = true;
    break;

  case Qt::Key_Control:
    controlPressed = true;
    break;

  case Qt::Key_Alt:
    altPressed = true;
    break;

  case Qt::Key_R:
    printf("Reseting camera pose\n");
    myCam->resetCamera();
    break;

  case Qt::Key_W:
    myCam->moveKey(0, factor);
    updateCamera();
    updateGL();
    break;

  case Qt::Key_S:
    myCam->moveKey(1, factor);
    updateCamera();
    updateGL();
    break;

  case Qt::Key_D:
    myCam->moveKey(2, factor);
    updateCamera();
    updateGL();
    break;

  case Qt::Key_A:
    myCam->moveKey(3, factor);
    updateCamera();
    updateGL();
    break;

  case Qt::Key_Q:
    myCam->moveKey(4, factor);
    updateGL();
    break;

  case Qt::Key_Z:
    break;

  case Qt::Key_6:
    printf("Save camera 1\n");
    myCam->printCamera();
    myCam->saveCameraPose(1);
    break;

  case Qt::Key_7:
    printf("Save camera 2\n");
    myCam->saveCameraPose(2);
    break;

  case Qt::Key_8:
    printf("Save camera 3\n");
    myCam->saveCameraPose(3);
    break;

  case Qt::Key_9:
    printf("Save camera 4\n");
    myCam->saveCameraPose(4);
    break;

  case Qt::Key_0:
    printf("Save camera 5\n");
    myCam->saveCameraPose(5);
    break;

  case Qt::Key_1:
    printf("Load Camera1\n");
    myCam->loadCameraPose(1);
    break;

  case Qt::Key_2:
    printf("Load Camera2\n");
    myCam->loadCameraPose(2);
    break;

  case Qt::Key_3:
    printf("Load Camera3\n");
    myCam->loadCameraPose(3);
    break;

  case Qt::Key_4:
    printf("Load Camera4\n");
    myCam->loadCameraPose(4);
    break;

  case Qt::Key_5:
    printf("Load Camera5\n");
    myCam->loadCameraPose(5);
    break;

  case Qt::Key_M:
    //printf("M pressed\n");
    keyMPressed = true;
    vboRenderManager.editionMode = true;
    updateGL();
    setMouseTracking(true);
    break;

  case Qt::Key_L:
    keyLPressed = true;
    printf("L pressed\n");
    break;

  case Qt::Key_K:
    //printf("K pressed\n");
    //shadow.displayDepthTex=!shadow.displayDepthTex;
    break;

  case Qt::Key_T:

    timer.start();

    for (int i = 0; i < 40; i++) {
      updateGL();
    }

    elapsed = timer.elapsed();
    printf("test render 40 %d ms--> %f fps", elapsed,
           40.0f * 1000.0f / ((elapsed + 0.01)));
    break;

  case Qt::Key_C:
    name = "screenshots/" + QDate::currentDate().toString("yyyyMMdd") + "_";
    numb = 0;
    numbS = numbS.sprintf("%02d", numb);

    while (QFile::exists(name + numbS + ".png")) {
      numb++;
      numbS = numbS.sprintf("%02d", numb);
    }

    grabFrameBuffer().save(name + numbS + ".png");
    printf("Saved: %s\n", (name + numbS + ".png").toLatin1().constData());
    break;
  }
}//

void LCGLWidget3D::keyReleaseEvent(QKeyEvent *e) {

  if (e->isAutoRepeat()) {
    e->ignore();
    return;
  }

  //printf("************RELEASE\n");
  keyHold = 0;

  switch (e->key()) {
  case Qt::Key_Escape:
    break;

  case Qt::Key_Shift:
    shiftPressed = false;
    break;

  case Qt::Key_Control:
    controlPressed = false;
    break;

  case Qt::Key_Alt:
    altPressed = false;

  case Qt::Key_M:
    keyMPressed = false;
    setMouseTracking(false);
    vboRenderManager.editionMode = false;
    shadow.makeShadowMap(this);
    updateGL();//remove circle

  default:
    ;
  }
}

bool LCGLWidget3D::mouseTo3D(int x, int y, QVector3D *result) {
  updateCamera();

  GLint viewport[4];

  GLfloat winX, winY, winZ;
  GLdouble posX, posY, posZ;

  glGetIntegerv(GL_VIEWPORT, viewport);

  winX = (float)x;
  winY = (float)viewport[3] - (float)y;
  glReadPixels(x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);

  float *data_matrix = myCam->mvMatrix.data();
  float *p_data_matrix = myCam->pMatrix.data();
  float objectCoordinate[3];
  // Next function has not been fully tested
  glhUnProjectf(winX, winY, winZ, data_matrix, p_data_matrix,
                viewport, &objectCoordinate[0]);

  result->setX(objectCoordinate[0]);
  result->setY(objectCoordinate[1]);
  result->setZ(objectCoordinate[2]);
  return true;
}//

bool LCGLWidget3D::generateGeometry(int flag) {

  //remove all geometry
  vboRenderManager.removeStaticGeometry("roads", false);

  vboRenderManager.removeStaticGeometry("sidewalk", false); //flag no warning
  vboRenderManager.removeStaticGeometry("building", false);

  vboRenderManager.removeStaticGeometry("roads_2D_segments5", false);
  vboRenderManager.removeStaticGeometry("roads_2D_segments2", false);
  vboRenderManager.removeStaticGeometry("road_2D_GM", false);
  vboRenderManager.removeStaticGeometry("road_2D_GM_int", false);

  printf(">> GenerateGeometry\n");
  printf(">> GenerateGeometry: PM\n");
  vboRenderManager.vboGUI.updateVBOGUI(vboRenderManager,
                                       cg.geoPlaceTypes.myPlaceTypes, cg.selectedPlaceTypeIdx);
  cg.generateGeometry(flag);// , this);
  printf(">> GenerateGeometry: VBO Roads\n");
  vboRoadGraph.updateRoadGraph(vboRenderManager, cg.roadGraph);

  if (flag == ClientGeometry::kJustRoads) { //just roads remove buildings
    vboRenderManager.removeStaticGeometry("sidewalk");
    vboRenderManager.removeStaticGeometry("building");
    // vegetation !!!
    vboRenderManager.removeAllStreetElementName("streetLamp");
    vboRenderManager.removeAllStreetElementName("tree");
  } else { //generate buildings and vegetation
    printf(">> GenerateGeometry: VBO Blocks\n");
    VBOBlocks::generateVBOBlocks(vboRenderManager, cg.geoZone.blocks);
    printf(">> GenerateGeometry: VBO veg\n");
    VBOVegetation::generateVegetation(vboRenderManager, cg.geoPlaceTypes,
                                      cg.geoZone.blocks);
  }

  shadow.makeShadowMap(this);
  printf("<< GenerateGeometry\n");
  return true;
}//

void LCGLWidget3D::updateCamera() {
  // update matrices
  int height = this->height() ? this->height() : 1;
  glViewport(0, 0, (GLint)this->width(), (GLint)this->height());
  myCam->updatePerspective(this->width(), height);
  myCam->updateCamMatrix();

  // update uniforms
  float mvpMatrixArray[16];
  float mvMatrixArray[16];

  for (int i = 0; i < 16; i++) {
    mvpMatrixArray[i] = myCam->mvpMatrix.data()[i];
    mvMatrixArray[i] = myCam->mvMatrix.data()[i];
  }

  float normMatrixArray[9];

  for (int i = 0; i < 9; i++) {
    normMatrixArray[i] = myCam->normalMatrix.data()[i];
  }

  //glUniformMatrix4fv(mvpMatrixLoc,  1, false, mvpMatrixArray);
  glUniformMatrix4fv(glGetUniformLocation(vboRenderManager.program, "mvpMatrix"),
                     1, false, mvpMatrixArray);
  glUniformMatrix4fv(glGetUniformLocation(vboRenderManager.program, "mvMatrix"),
                     1, false, mvMatrixArray);
  glUniformMatrix3fv(glGetUniformLocation(vboRenderManager.program,
                                          "normalMatrix"),  1, false, normMatrixArray);

  // light poss
  QVector3D light_dir = myCam->light_dir.toVector3D();
  glUniform3f(glGetUniformLocation(vboRenderManager.program, "lightDir"),
              light_dir.x(), light_dir.y(), light_dir.z());

}//

void LCGLWidget3D::changeRender2D3D() {
  if (G::global().getInt("render_mode") != 0) { //2D Mode
    updateCamera();
    glUniform1i(glGetUniformLocation(vboRenderManager.program, "render_mode"),
                G::global().getInt("render_mode"));//1 --> 2D 2--> Layer
  } else { //3D
    //G::global().getBool("render_mode")==false
    updateCamera();
    shadow.makeShadowMap(this);
    glUniform1i(glGetUniformLocation(vboRenderManager.program, "render_mode"), 0);
  }

  printf("RenderMode %d\n", G::global().getInt("render_mode"));
  printf("___ #Edges %d #Vertex %d \n",
         boost::num_edges(cg.roadGraph.myRoadGraph_BI),
         boost::num_vertices(cg.roadGraph.myRoadGraph_BI));
  vboRoadGraph.updateRoadGraph(vboRenderManager, cg.roadGraph);
}//

} // namespace LC
