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
// nvModelObj.cpp - Model support class
//
// The nvModel class implements an interface for a multipurpose model
// object. This class is useful for loading and formatting meshes
// for use by OpenGL. It can compute face normals, tangents, and
// adjacency information. The class supports the obj file format.
//
// This file implements the obj file parser and translator.
//
// Author: Evan Hart
// Email: sdkfeedback@nvidia.com
//
// Copyright (c) NVIDIA Corporation. All rights reserved.
////////////////////////////////////////////////////////////////////////////////

#include "nvModel.h"

#include <stdio.h>

#define BUF_SIZE 256

using std::vector;

static void skipLine(char * buf, int size, FILE * fp)
{
	do {
		buf[size-1] = '$';
		fgets(buf, size, fp);
	} while (buf[size-1] != '$');
}


namespace nv {

bool Model::loadObjFromFile( const char *file, Model &m) {
    FILE *fp;

    fp = fopen( file, "r");
    if (!fp) {
        return false;
    }

    char buf[BUF_SIZE];
    float val[4];
    int idx[3][3];
    int match;
    bool vtx4Comp = false;
    bool tex3Comp = false;
    bool hasTC = false;
    bool hasNormals = false;

    while ( fscanf( fp, "%s", buf) != EOF ) {

        switch (buf[0]) {
            case '#':
                //comment line, eat the remainder
				skipLine( buf, BUF_SIZE, fp);
                break;

            case 'v':
                switch (buf[1]) {
                
                    case '\0':
                        //vertex, 3 or 4 components
                        val[3] = 1.0f;  //default w coordinate
                        match = fscanf( fp, "%f %f %f %f", &val[0], &val[1], &val[2], &val[3]);
                        m._positions.push_back( val[0]);
                        m._positions.push_back( val[1]);
                        m._positions.push_back( val[2]);
                        m._positions.push_back( val[3]);
                        vtx4Comp |= ( match == 4);
                        assert( match > 2 && match < 5);
                        break;

                    case 'n':
                        //normal, 3 components
                        match = fscanf( fp, "%f %f %f", &val[0], &val[1], &val[2]);
                        m._normals.push_back( val[0]);
                        m._normals.push_back( val[1]);
                        m._normals.push_back( val[2]);
                        assert( match == 3);
                        break;

                    case 't':
                        //texcoord, 2 or 3 components
                        val[2] = 0.0f;  //default r coordinate
                        match = fscanf( fp, "%f %f %f %f", &val[0], &val[1], &val[2]);
                        m._texCoords.push_back( val[0]);
                        m._texCoords.push_back( val[1]);
                        m._texCoords.push_back( val[2]);
                        tex3Comp |= ( match == 3);
                        assert( match > 1 && match < 4);
                        break;
                }
                break;

            case 'f':
                //face
                fscanf( fp, "%s", buf);

                //determine the type, and read the initial vertex, all entries in a face must have the same format
                if ( sscanf( buf, "%d//%d", &idx[0][0], &idx[0][1]) == 2) {
                    //This face has vertex and normal indices

                    //remap them to the right spot
                    idx[0][0] = (idx[0][0] > 0) ? (idx[0][0] - 1) : ((int)m._positions.size() - idx[0][0]);
                    idx[0][1] = (idx[0][1] > 0) ? (idx[0][1] - 1) : ((int)m._normals.size() - idx[0][1]);

                    //grab the second vertex to prime
                    fscanf( fp, "%d//%d", &idx[1][0], &idx[1][1]);

                    //remap them to the right spot
                    idx[1][0] = (idx[1][0] > 0) ? (idx[1][0] - 1) : ((int)m._positions.size() - idx[1][0]);
                    idx[1][1] = (idx[1][1] > 0) ? (idx[1][1] - 1) : ((int)m._normals.size() - idx[1][1]);

                    //create the fan
                    while ( fscanf( fp, "%d//%d", &idx[2][0], &idx[2][1]) == 2) {
                        //remap them to the right spot
                        idx[2][0] = (idx[2][0] > 0) ? (idx[2][0] - 1) : ((int)m._positions.size() - idx[2][0]);
                        idx[2][1] = (idx[2][1] > 0) ? (idx[2][1] - 1) : ((int)m._normals.size() - idx[2][1]);

                        //add the indices
                        for (int ii = 0; ii < 3; ii++) {
                            m._pIndex.push_back( idx[ii][0]);
                            m._nIndex.push_back( idx[ii][1]);
                            m._tIndex.push_back(0); // dummy index, to ensure that the buffers are of identical size
                        }
                        
                        //prepare for the next iteration
                        idx[1][0] = idx[2][0];
                        idx[1][1] = idx[2][1];
                    }
                    hasNormals = true;
                }
                else if ( sscanf( buf, "%d/%d/%d", &idx[0][0], &idx[0][1], &idx[0][2]) == 3) {
                    //This face has vertex, texture coordinate, and normal indices

                    //remap them to the right spot
                    idx[0][0] = (idx[0][0] > 0) ? (idx[0][0] - 1) : ((int)m._positions.size() - idx[0][0]);
                    idx[0][1] = (idx[0][1] > 0) ? (idx[0][1] - 1) : ((int)m._texCoords.size() - idx[0][1]);
                    idx[0][2] = (idx[0][2] > 0) ? (idx[0][2] - 1) : ((int)m._normals.size() - idx[0][2]);

                    //grab the second vertex to prime
                    fscanf( fp, "%d/%d/%d", &idx[1][0], &idx[1][1], &idx[1][2]);

                    //remap them to the right spot
                    idx[1][0] = (idx[1][0] > 0) ? (idx[1][0] - 1) : ((int)m._positions.size() - idx[1][0]);
                    idx[1][1] = (idx[1][1] > 0) ? (idx[1][1] - 1) : ((int)m._texCoords.size() - idx[1][1]);
                    idx[1][2] = (idx[1][2] > 0) ? (idx[1][2] - 1) : ((int)m._normals.size() - idx[1][2]);

                    //create the fan
                    while ( fscanf( fp, "%d/%d/%d", &idx[2][0], &idx[2][1], &idx[2][2]) == 3) {
                        //remap them to the right spot
                        idx[2][0] = (idx[2][0] > 0) ? (idx[2][0] - 1) : ((int)m._positions.size() - idx[2][0]);
                        idx[2][1] = (idx[2][1] > 0) ? (idx[2][1] - 1) : ((int)m._texCoords.size() - idx[2][1]);
                        idx[2][2] = (idx[2][2] > 0) ? (idx[2][2] - 1) : ((int)m._normals.size() - idx[2][2]);

                        //add the indices
                        for (int ii = 0; ii < 3; ii++) {
                            m._pIndex.push_back( idx[ii][0]);
                            m._tIndex.push_back( idx[ii][1]);
                            m._nIndex.push_back( idx[ii][2]);
                        }
                        
                        //prepare for the next iteration
                        idx[1][0] = idx[2][0];
                        idx[1][1] = idx[2][1];
                        idx[1][2] = idx[2][2];
                    }

                    hasTC = true;
                    hasNormals = true;
                }
                else if ( sscanf( buf, "%d/%d", &idx[0][0], &idx[0][1]) == 2) {
                    //This face has vertex and texture coordinate indices

                    //remap them to the right spot
                    idx[0][0] = (idx[0][0] > 0) ? (idx[0][0] - 1) : ((int)m._positions.size() - idx[0][0]);
                    idx[0][1] = (idx[0][1] > 0) ? (idx[0][1] - 1) : ((int)m._texCoords.size() - idx[0][1]);

                    //grab the second vertex to prime
                    fscanf( fp, "%d/%d", &idx[1][0], &idx[1][1]);

                    //remap them to the right spot
                    idx[1][0] = (idx[1][0] > 0) ? (idx[1][0] - 1) : ((int)m._positions.size() - idx[1][0]);
                    idx[1][1] = (idx[1][1] > 0) ? (idx[1][1] - 1) : ((int)m._texCoords.size() - idx[1][1]);

                    //create the fan
                    while ( fscanf( fp, "%d/%d", &idx[2][0], &idx[2][1]) == 2) {
                        //remap them to the right spot
                        idx[2][0] = (idx[2][0] > 0) ? (idx[2][0] - 1) : ((int)m._positions.size() - idx[2][0]);
                        idx[2][1] = (idx[2][1] > 0) ? (idx[2][1] - 1) : ((int)m._texCoords.size() - idx[2][1]);

                        //add the indices
                        for (int ii = 0; ii < 3; ii++) {
                            m._pIndex.push_back( idx[ii][0]);
                            m._tIndex.push_back( idx[ii][1]);
                            m._nIndex.push_back( 0); //dummy normal index to keep everything in synch
                        }
                        
                        //prepare for the next iteration
                        idx[1][0] = idx[2][0];
                        idx[1][1] = idx[2][1];
                    }
                    hasTC = true;
                }
                else if ( sscanf( buf, "%d", &idx[0][0]) == 1) {
                    //This face has only vertex indices

                    //remap them to the right spot
                    idx[0][0] = (idx[0][0] > 0) ? (idx[0][0] - 1) : ((int)m._positions.size() - idx[0][0]);

                    //grab the second vertex to prime
                    fscanf( fp, "%d", &idx[1][0]);

                    //remap them to the right spot
                    idx[1][0] = (idx[1][0] > 0) ? (idx[1][0] - 1) : ((int)m._positions.size() - idx[1][0]);

                    //create the fan
                    while ( fscanf( fp, "%d", &idx[2][0]) == 1) {
                        //remap them to the right spot
                        idx[2][0] = (idx[2][0] > 0) ? (idx[2][0] - 1) : ((int)m._positions.size() - idx[2][0]);

                        //add the indices
                        for (int ii = 0; ii < 3; ii++) {
                            m._pIndex.push_back( idx[ii][0]);
                            m._tIndex.push_back( 0); //dummy index to keep things in synch
                            m._nIndex.push_back( 0); //dummy normal index to keep everything in synch
                        }
                        
                        //prepare for the next iteration
                        idx[1][0] = idx[2][0];
                    }
                }
                else {
                    //bad format
                    assert(0);
                    skipLine( buf, BUF_SIZE, fp);
                }
                break;

            case 's':
            case 'g':
            case 'u':
                //all presently ignored
            default:
				skipLine( buf, BUF_SIZE, fp);

        };
    }

    fclose(fp);

    //post-process data

    //free anything that ended up being unused
    if (!hasNormals) {
        m._normals.clear();
        m._nIndex.clear();
    }

    if (!hasTC) {
        m._texCoords.clear();
        m._tIndex.clear();
    }

    //set the defaults as the worst-case for an obj file
    m._posSize = 4;
    m._tcSize = 3;

    //compact to 3 component vertices if possible
    if (!vtx4Comp) {
        vector<float>::iterator src = m._positions.begin();
        vector<float>::iterator dst = m._positions.begin();

        for ( ; src < m._positions.end(); ) {
            *(dst++) = *(src++);
            *(dst++) = *(src++);
            *(dst++) = *(src++);
            src++;
        }

        m._positions.resize( (m._positions.size() / 4) * 3);

        m._posSize = 3;
    }

    //compact to 2 component tex coords if possible
    if (!tex3Comp) {
        vector<float>::iterator src = m._texCoords.begin();
        vector<float>::iterator dst = m._texCoords.begin();

        for ( ; src < m._texCoords.end(); ) {
            *(dst++) = *(src++);
            *(dst++) = *(src++);
            src++;
        }

        m._texCoords.resize( (m._texCoords.size() / 3) * 2);

        m._tcSize = 2; 
    }

    return true;
}


};

