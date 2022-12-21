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
*		MTC Project - Client Main Project - ClientGlobalVariables
*
*
*		@desc The global variables for the client.
*		@author igarciad
*
************************************************************************************************/
//based on www.yolinux.com/TUTORIALS/C++Singleton.html

#pragma once

#include <QHash>
#include <QVariant>
#include <QVector3D>
#include <vector>

#include <iostream>

namespace LC {

enum t_land_use {
  kLUNone = 0,
  kLUResS,
  kLUResM,
  kLUResL,
  kLUOfficeS,
  kLUOfficeM,
  kLUOfficeL,
  kLURetail,
  kLUIndS,
  kLUIndM,
  kLUIndL,
  kLUPking,
  kLUInstit,
  kLUPark,
  kLUWater
};

class G {
 public:

  static std::vector<QVector3D> boundingPolygon;

  static G &global();
  static QHash<QString, QVariant> g;

  QVariant operator [](QString i) const    {
    return g[i];
  }
  QVariant &operator [](QString i) {
    return g[i];
  }

  static QVector3D getQVector3D(QString i) {
    if (!g.contains(i)) {
      printf("Global does not contain type %s\n", i.toUtf8().constData());
      return QVector3D();
    }

    return g[i].value<QVector3D>();
  }
  static float getFloat(QString i) {
    if (!g.contains(i)) {
      printf("Global does not contain type %s\n", i.toUtf8().constData());
      return 0;
    }

    return g[i].toFloat();
  }
  static float getDouble(QString i) {
    if (!g.contains(i)) {
      printf("Global does not contain type %s\n", i.toUtf8().constData());
      return 0;
    }

    return g[i].toDouble();
  }
  static int getInt(QString i) {
    if (!g.contains(i)) {
      printf("Global does not contain type %s\n", i.toUtf8().constData());
      return 0;
    }

    return g[i].toInt();
  }
  static bool getBool(QString i) {
    if (!g.contains(i)) {
      printf("Global does not contain type %s\n", i.toUtf8().constData());
      return false;
    }

    return g[i].toBool();
  }

 private:
  G() {}; // Private so that it can  not be called
  G(G const &) {};           // copy constructor is private
  G &operator=(G const &) {}; // assignment operator is private
  //static Global* m_pInstance;


};


}

