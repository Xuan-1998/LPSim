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

//
// nvModel.h - Model support class
//
// The nvModel class implements an interface for a multipurpose model
// object. This class is useful for loading and formatting meshes
// for use by OpenGL. It can compute face normals, tangents, and
// adjacency information. The class supports the obj file format.
//
// Author: Evan Hart
// Email: sdkfeedback@nvidia.com
//
// Copyright (c) NVIDIA Corporation. All rights reserved.
////////////////////////////////////////////////////////////////////////////////

#ifndef NV_MODEL_H
#define NV_MODEL_H

#include <vector>
#include <assert.h>

#include <GL/glew.h>
#include "nvMath.h"

namespace nv {

    class Model {
    public:

        //
        // Enumeration of primitive types
        //
        //////////////////////////////////////////////////////////////
        enum PrimType {
            eptNone = 0x0,
            eptPoints = 0x1,
            eptEdges = 0x2,
            eptTriangles = 0x4,
            eptTrianglesWithAdjacency = 0x8,
            eptAll = 0xf
        };

        static const int NumPrimTypes = 4;

		 static Model* CreateModel();

         Model();
         virtual ~Model();

        //
        // loadModelFromFile
        //
        //    This function attempts to determine the type of
        //  the filename passed as a parameter. If it understands
        //  that file type, it attempts to parse and load the file
        //  into its raw data structures. If the file type is
        //  recognized and successfully parsed, the function returns
        //  true, otherwise it returns false.
        //
        //////////////////////////////////////////////////////////////
         bool loadModelFromFile( const char* file);

        //
        //  compileModel
        //
        //    This function takes the raw model data in the internal
        //  structures, and attempts to bring it to a format directly
        //  accepted for vertex array style rendering. This means that
        //  a unique compiled vertex will exist for each unique
        //  combination of position, normal, tex coords, etc that are
        //  used in the model. The prim parameter, tells the model
        //  what type of index list to compile. By default it compiles
        //  a simple triangle mesh with no connectivity. 
        //
        //////////////////////////////////////////////////////////////
         void compileModel( PrimType prim = eptTriangles);

		 //
        //  computeBoundingBox
        //
        //    This function returns the points defining the axis-
        //  aligned bounding box containing the model.
        //
        //////////////////////////////////////////////////////////////
         void computeBoundingBox( vec3f &minVal, vec3f &maxVal);


        //
        //  rescale
        //
        //  rescales object based on bounding box
        //
        //////////////////////////////////////////////////////////////
         void rescale( float radius);

		 //
        //  scalePerNumber
        //
        //  rescales object based on factor
        //
        //////////////////////////////////////////////////////////////
         void scalePerNumber( float factor);

		  //
        //  scalePerNumber
        //
        //  rescales object based on factor
        //
        //////////////////////////////////////////////////////////////
         void scalePerNumber( float factorX,float factorY,float factorZ);

		    //
        //  centerAndSetMaxSize igarciad
        //
        //  center object and rescale to fit the maximum size
        //
        //////////////////////////////////////////////////////////////
		 void centerXYzeroZAndSetMaxSize(float maxSize);

        //
        //  buildTangents
        //
        //    This function computes tangents in the s direction on
        //  the model. It operates on the raw data, so it should only
        //  be used before compiling a model into a HW friendly form.
        //
        //////////////////////////////////////////////////////////////
         void computeTangents();

        //
        //  computeNormals
        //
        //    This function computes vertex normals for a model
        //  which did not have them. It computes them on the raw
        //  data, so it should be done before compiling the model
        //  into a HW friendly format.
        //
        //////////////////////////////////////////////////////////////
         void computeNormals();

         void removeDegeneratePrims();

        //
        //general query functions
        //
         bool hasNormals() const;
         bool hasTexCoords() const;
         bool hasTangents() const;
         bool hasColors() const;

         int getPositionSize() const;
         int getNormalSize() const;
         int getTexCoordSize() const;
         int getTangentSize() const;
         int getColorSize() const;

        //
        //  Functions for the management of raw data
        //
         void clearNormals();
         void clearTexCoords();
         void clearTangents();
         void clearColors();

        //
        //raw data access functions
        //  These are to be used to get the raw array data from the file, each array has its own index
        //
         const float* getPositions() const;
         const float* getNormals() const;
         const float* getTexCoords() const;
         const float* getTangents() const;
         const float* getColors() const;

         const GLuint* getPositionIndices() const;
         const GLuint* getNormalIndices() const;
         const GLuint* getTexCoordIndices() const;
         const GLuint* getTangentIndices() const;
         const GLuint* getColorIndices() const;

         int getPositionCount() const;
         int getNormalCount() const;
         int getTexCoordCount() const;
         int getTangentCount() const;
         int getColorCount() const;

         int getIndexCount() const;

        //
        //compiled data access functions
        //
         const float* getCompiledVertices() const;
         const GLuint* getCompiledIndices( PrimType prim = eptTriangles) const;

         int getCompiledPositionOffset() const;
         int getCompiledNormalOffset() const;
         int getCompiledTexCoordOffset() const;
         int getCompiledTangentOffset() const;
         int getCompiledColorOffset() const;

        // returns the size of the merged vertex in # of floats
         int getCompiledVertexSize() const;

         int getCompiledVertexCount() const;
         int getCompiledIndexCount( PrimType prim = eptTriangles) const;

         int getOpenEdgeCount() const;

    protected:

        //Would all this be better done as a channel abstraction to handle more arbitrary data?

        //data structures for model data, not optimized for rendering
        std::vector<float> _positions;
        std::vector<float> _normals;
        std::vector<float> _texCoords;
        std::vector<float> _sTangents;
        std::vector<float> _colors;
        int _posSize;
        int _tcSize;
        int _cSize;

        std::vector<GLuint> _pIndex;
        std::vector<GLuint> _nIndex;
        std::vector<GLuint> _tIndex;
        std::vector<GLuint> _tanIndex;
        std::vector<GLuint> _cIndex;

        //data structures optimized for rendering, compiled model
        std::vector<GLuint> _indices[NumPrimTypes];
        std::vector<float> _vertices;
        int _pOffset;
        int _nOffset;
        int _tcOffset;
        int _sTanOffset;
        int _cOffset;
        int _vtxSize;

        int _openEdges;

        //
        // Static elements used to dispatch to proper sub-readers
        //
        //////////////////////////////////////////////////////////////
        struct FormatInfo {
            const char* extension;
            bool (*reader)( const char* file, Model& i);
        };

        static FormatInfo formatTable[]; 

         static bool loadObjFromFile( const char *file, Model &m);
    };
};


#endif
