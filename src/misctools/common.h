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

#ifndef COMMON_ALREADY_INCLUDED
#define COMMON_ALREADY_INCLUDED

#include "GL/glew.h"

#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <limits>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <assert.h>


#include <vector>
#include <stdexcept>
#include <iostream>
#include <QVector2D>
#include <QVector3D>
#include <QMatrix4x4>
#include <QTime>
#include <QCoreApplication>
#ifndef Q_MOC_RUN
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/multi/multi.hpp>
#endif
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <GL/glu.h>

/*
 * Useful defines...
 */
#ifndef TRUE
#define TRUE			      1
#endif

#ifndef FALSE
#define FALSE			      0
#endif

#ifndef ABS
#define ABS(X)			      (((X) < 0) ? -(X) : (X))
#endif
#ifndef MIN
#define MIN(A,B)		      (((A) < (B)) ? (A) : (B))
#define MAX(A,B)		      (((A) > (B)) ? (A) : (B))
#endif
#define SQR(A)			      ((A) * (A))
#define CUBE(A)			   ((A) * (A) * (A))
#define FLOAT_EPSILON		1e-7
#define DOUBLE_EPSILON		1e-16
#define brcase			      break; case
#define brdefault		      break; default
#define MIN3(A,B,C) 		   MIN(MIN(A,B),C)
#define SIGN(A)			   (((A) < 0) ? -1 : 1)
#define CLAMP(X,MIN,MAX)                        \
        if (1) {                                \
           if ((X)<(MIN)) (X) = (MIN);          \
           else if ((X)>(MAX)) (X) = (MAX);     \
        } else
#define SWAP(A,B,T)				\
	if (1) {				\
	   (T) = (A);				\
	   (A) = (B);				\
	   (B) = (T);				\
	} else
#define FEQUAL(A,B)        (ABS((A)-(B)) < 0.000001)
#define INRANGE(X,A,B)     ((X) >= (A) && (X) <= (B))
#define M_TWOPI            6.28318530717958647692
#ifndef M_PI
#define M_PI               3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2             1.57079632679489661923
#endif
#ifndef M_PI_4
#define M_PI_4             0.78539816339744830962
#endif
#define SQRT_2             1.4142135623730950488
#define SQRT_3             1.7320508075688772935

#if defined(_WIN32) || defined(linux) || defined(__CYGWIN__)
#define TAN(X)             (float)tan((float)(X))
#define SIN(X)             (float)sin((float)(X))
#define COS(X)             (float)cos((float)(X))
#define SQRT(X)            (float)sqrt((float)(X))
#define ASIN(X)            (float)asin((float)(X))
#define ACOS(X)            (float)acos((float)(X))
#define ATAN(X)            (float)atan((float)(X))
#define bzero(p,s)         memset(p, 0, s)
#define bcopy(s,d,l)       memcpy(d, s, l)
#else
#define TAN(X)             (float)tanf((float)(X))
#define SIN(X)             (float)sinf((float)(X))
#define COS(X)             (float)cosf((float)(X))
#define SQRT(X)            (float)fsqrt((float)(X))
#define ASIN(X)            (float)asinf((float)(X))
#define ACOS(X)            (float)acosf((float)(X))
#define ATAN(X)            (float)atanf((float)(X))
#endif

#define BZERO(P,S)		   bzero((void *)(P),(S))
#define BCOPY(S,D,L)		   bcopy((void *)(S),(void *)(D),(L));

#ifdef DEBUG
#define IFDEBUG(X)	      if (X)
#else
#define IFDEBUG(X)	      if (0)
#endif

#define MYDELETE(P)			delete (P), (P) = NULL


/*
 * Short hand var types
 */
typedef unsigned int   uint;
typedef unsigned short ushort;
typedef unsigned char  uchar;



/*
 * Use to check that a new of a non data object class succeeded
 */
#ifdef DEBUG
#define ALLOC_VERIFY(P)		   \
	if (!P) {						\
 	   fprintf(stderr,"ERROR: memory allocation returned NULL!\n");	\
   } else
#else
#define ALLOC_VERIFY(P)
#endif


#endif
