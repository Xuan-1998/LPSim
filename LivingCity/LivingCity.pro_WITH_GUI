QT += core gui opengl

# Project build directories
DESTDIR     = $$PWD
OBJECTS_DIR = $$DESTDIR/obj

unix {
    LIBS += -L/opt/local/lib -lopencv_imgcodecs -lopencv_core -lopencv_imgproc -lGLEW
	# -L/Developer/NVIDIA/CUDA-7.5/lib -lcudart -lcublas
    INCLUDEPATH += \
      /usr/include/opencv2/ \
      /opt/local/include/GL/ \
      /opt/local/include/ \ 
      /shared/boost/boost_1_59_0/ \
      $$PWD/glew/include/GL
}
win32{
    # Note: OpenCV uses 2.4.12 since I compile with VS 2013 (vc12)
    LIBS+= \
        $$(OPENCV_BUILD)/x64/vc12/lib/opencv_core2412.lib \ # e.g., OPENCV_BUILD=D:\opencv\build
        $$(OPENCV_BUILD)/x64/vc12/lib/opencv_imgproc2412.lib \
        $$(OPENCV_BUILD)/x64/vc12/lib/opencv_highgui2412.lib \
        $$(OPENCV_BUILD)/x64/vc12/lib/opencv_legacy2412.lib \
        $$(OPENCV_BUILD)/x64/vc12/lib/opencv_ml2412.lib \
        $$(OPENCV_BUILD)/x64/vc12/lib/opencv_photo2412.lib \
        $$(OPENCV_BUILD)/x64/vc12/lib/opencv_video2412.lib \
        $$PWD/glew/lib/glew32.lib \
        $$PWD/glew/lib/OpenGL32.Lib \
        -lglut32


    INCLUDEPATH += \
        $$(OPENCV_BUILD)/include/opencv \ # e.g., OPENCV_BUILD=D:\opencv\build
        $$(OPENCV_BUILD)/include/opencv2 \
        $$(OPENCV_BUILD)/include \
        $$(BOOST_ROOT) \ # e.g., BOOST_ROOT = D:\boost\boost_1_59_0
        $$PWD/glew/include/GL

    CONFIG += console # show printf in terminal
}


RESOURCES += \
    LC_UrbanMain.qrc

FORMS += \
    LC_UrbanMain.ui

HEADERS += \
    Geometry/block.h \
    Geometry/building.h \
    Geometry/client_geometry.h \
    Geometry/parcel.h \
    Geometry/parcelBuildingAttributes.h \
    Geometry/placeTypeInstances.h \
    Geometry/zone.h \
    LC_GLWidget3D.h \
    LC_GLWidget3D_Shadows.h \
    LC_Layer.h \
    LC_UrbanMain.h \
    LC_camera_3d.h \
    PM/pmBlocks.h \
    PM/pmBuildings.h \
    PM/pmMain.h \
    PM/pmParcels.h \
    PM/pmRoads.h \
    RoadGraph/roadGraph.h \
    RoadGraph/roadGraphEdge.h \
    RoadGraph/roadGraphVertex.h \
    VBOBlocks.h \
    VBOBuilding.h \
    VBOGUI.h \
    VBOModel.h \
    VBOModel_StreetElements.h \
    VBOPeopleJobInfoLayer.h \
    VBORenderManager.h \
    VBORoadGraph.h \
    VBORoadLabels.h \
    VBOShader.h \
    VBOSkyBox.h \
    VBOTerrain.h \
    VBOText.h \
    VBOUtil.h \
    VBOVegetation.h \
    VBOWater.h \
    global.h \
    misctools/bounding_box.h \
    misctools/common.h \
    misctools/misctools.h \
    misctools/polygon_3D.h \
    nvModel/nvMath.h \
    nvModel/nvMatrix.h \
    nvModel/nvModel.h \
    nvModel/nvQuaternion.h \
    nvModel/nvVector.h \
    roadGraphB2018Loader.h \
    roadGraphDynameqLoader.h \
    traffic/b18CUDA_trafficSimulator.h \
    traffic/b18CommandLineVersion.h \
    traffic/b18EdgeData.h \
    traffic/b18GridPollution.h \
    traffic/b18TestSimpleRoadAndOD.h \
    traffic/b18TrafficDijkstra.h \
    traffic/b18TrafficJohnson.h \
    traffic/b18TrafficLaneMap.h \
    traffic/b18TrafficOD.h \
    traffic/b18TrafficPerson.h \
    traffic/b18TrafficSimulator.h \
    traffic/cudaEdgeData.h \
    traffic/cudaGridPollution.h \
    traffic/cudaPmTrafficPersonJob.h \
    traffic/cudaTrafficDesigner.h \
    traffic/cudaTrafficLaneMap.h \
    traffic/cudaTrafficMCMC.h \
    traffic/cudaTrafficPerson.h \
    traffic/cudaTrafficPersonPath.h \
    traffic/cudaTrafficPersonShortestPath.h \
    traffic/cudaTrafficRoutes.h \
    traffic/cudaTrafficSimulator.h \
    triangle/triangle.h \
    glu.h

SOURCES += \
    Geometry/block.cpp \
    Geometry/building.cpp \
    Geometry/client_geometry.cpp \
    Geometry/parcel.cpp \
    Geometry/parcelBuildingAttributes.cpp \
    Geometry/placeTypeInstances.cpp \
    Geometry/zone.cpp \
    LC_GLWidget3D.cpp \
    LC_GLWidget3D_Shadows.cpp \
    LC_Layer.cpp \
    LC_UrbanMain.cpp \
    LC_camera_3d.cpp \
    LC_main.cpp \
    PM/pmBlocks.cpp \
    PM/pmBuildings.cpp \
    PM/pmMain.cpp \
    PM/pmParcels.cpp \
    PM/pmRoads.cpp \
    RoadGraph/roadGraph.cpp \
    RoadGraph/roadGraphEdge.cpp \
    RoadGraph/roadGraphVertex.cpp \
    VBOBlocks.cpp \
    VBOBuilding.cpp \
    VBOGUI.cpp \
    VBOModel.cpp \
    VBOModel_StreetElements.cpp \
    VBOPeopleJobInfoLayer.cpp \
    VBORenderManager.cpp \
    VBORoadGraph.cpp \
    VBORoadLabels.cpp \
    VBOShader.cpp \
    VBOSkyBox.cpp \
    VBOTerrain.cpp \
    VBOText.cpp \
    VBOUtil.cpp \
    VBOVegetation.cpp \
    VBOWater.cpp \
    global.cpp \
    misctools/bounding_box.cpp \
    misctools/misctools.cpp \
    misctools/polygon_3D.cpp \
    nvModel/nvModel.cpp \
    nvModel/nvModelObj.cpp \
    nvModel/nvModelQuery.cpp \
    roadGraphB2018Loader.cpp \
    roadGraphDynameqLoader.cpp \
    traffic/b18CommandLineVersion.cpp \
    traffic/b18GridPollution.cpp \
    traffic/b18TestSimpleRoadAndOD.cpp \
    traffic/b18TrafficDijkstra.cpp \
    traffic/b18TrafficJohnson.cpp \
    traffic/b18TrafficLaneMap.cpp \
    traffic/b18TrafficOD.cpp \
    traffic/b18TrafficSimulator.cpp \
    traffic/cudaGridPollution.cpp \
    traffic/cudaPmTrafficPersonJob.cpp \
    traffic/cudaTrafficDesigner.cpp \
    traffic/cudaTrafficLaneMap.cpp \
    traffic/cudaTrafficMCMC.cpp \
    traffic/cudaTrafficPersonPath.cpp \
    traffic/cudaTrafficPersonShortestPath.cpp \
    traffic/cudaTrafficRoutes.cpp \
    traffic/cudaTrafficSimulator.cpp \
    triangle/triangle.c

OTHER_FILES += \
        traffic/b18CUDA_trafficSimulator.cu \
        bTraffic/bCUDA_trafficSimulator.cu




###################################################################
## CUDA
###################################################################
win32{
    # Cuda sources
    CUDA_SOURCES += traffic/b18CUDA_trafficSimulator.cu
    CUDA_SOURCES += bTraffic/bCUDA_trafficSimulator.cu

    # Path to cuda toolkit install
    CUDA_DIR      = "D:/CUDA"

    # Path to header and libs files
    INCLUDEPATH  += $$CUDA_DIR/include
    QMAKE_LIBDIR += $$CUDA_DIR/lib/x64

    SYSTEM_TYPE = 64            # '32' or '64', depending on your system

    # libs used in your code
    LIBS += -lcuda -lcudart
    CUDA_LIBS += -lcuda -lcudart # LIBS

    # GPU architecture
    CUDA_ARCH     = sm_50

    # Here are some NVCC flags I've always used by default.
    NVCCFLAGS     = --use_fast_math


    # Prepare the extra compiler configuration (taken from the nvidia forum - i'm not an expert in this part)
    CUDA_INC = $$join(INCLUDEPATH,'" -I"','-I"','"')


    # MSVCRT link option (static or dynamic, it must be the same with your Qt SDK link option)
    MSVCRT_LINK_FLAG_DEBUG = "/MDd"
    MSVCRT_LINK_FLAG_RELEASE = "/MD"

    QMAKE_EXTRA_COMPILERS += cuda

    # Configuration of the Cuda compiler
    CONFIG(debug, debug|release) {
        # Debug mode
        cuda_d.input = CUDA_SOURCES
        cuda_d.output = $$OBJECTS_DIR/${QMAKE_FILE_BASE}.obj
        cuda_d.commands = $$CUDA_DIR/bin/nvcc.exe -D_DEBUG $$NVCC_OPTIONS $$CUDA_INC $$CUDA_LIBS --machine $$SYSTEM_TYPE \
                         -arch=$$CUDA_ARCH -c -Xcompiler $$MSVCRT_LINK_FLAG_DEBUG -o ${QMAKE_FILE_OUT} ${QMAKE_FILE_NAME}
        cuda_d.dependency_type = TYPE_C
        QMAKE_EXTRA_COMPILERS += cuda_d
    }
    else {
        # Release mode
        cuda.input = CUDA_SOURCES
        cuda.output = $$OBJECTS_DIR/${QMAKE_FILE_BASE}.obj
        cuda.commands = $$CUDA_DIR/bin/nvcc.exe $$NVCC_OPTIONS $$CUDA_INC $$CUDA_LIBS --machine $$SYSTEM_TYPE \
                       -arch=$$CUDA_ARCH -c -Xcompiler $$MSVCRT_LINK_FLAG_RELEASE -o ${QMAKE_FILE_OUT} ${QMAKE_FILE_NAME}
        cuda.dependency_type = TYPE_C
        QMAKE_EXTRA_COMPILERS += cuda
    }
}

unix {
  # Cuda sources
  CUDA_SOURCES += traffic/b18CUDA_trafficSimulator.cu
  #CUDA_SOURCES += bTraffic/bCUDA_trafficSimulator.cu
  # Path to cuda toolkit install
  CUDA_DIR = /usr/local/cuda-9.2
  INCLUDEPATH += $$CUDA_DIR/include
  QMAKE_LIBDIR += $$CUDA_DIR/lib64
  # GPU architecture
  CUDA_ARCH = sm_50
  # NVCC flags
  NVCCFLAGS = --compiler-options -fno-strict-aliasing -use_fast_math --ptxas-options=-v
  # Path to libraries
  LIBS += -lcudart -lcuda
  # join the includes in a line
  CUDA_INC = $$join(INCLUDEPATH,' -I','-I',' ')
  cuda.commands = $$CUDA_DIR/bin/nvcc -m64 -O3 -arch=$$CUDA_ARCH -c $$NVCCFLAGS $$CUDA_INC $$LIBS ${QMAKE_FILE_NAME} -o ${QMAKE_FILE_OUT}
  cuda.dependcy_type = TYPE_C
  cuda.depend_command = $$CUDA_DIR/bin/nvcc -O3 -M $$CUDA_INC $$NVCCFLAGS      ${QMAKE_FILE_NAME}

  cuda.input = CUDA_SOURCES
  cuda.output = ${OBJECTS_DIR}${QMAKE_FILE_BASE}_cuda.o
  # Tell Qt that we want add more stuff to the Makefile
  QMAKE_EXTRA_COMPILERS += cuda
}
