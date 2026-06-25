/************************************************************************************************
*
*		LC Project - B18 Edge data
*
*		@author igaciad
*
************************************************************************************************/

#ifndef LC_B18_EDGE_DATA_H
#define LC_B18_EDGE_DATA_H

#include "stdint.h"

#ifndef ushort
#define ushort uint16_t
#endif
#ifndef uint
#define uint uint32_t
#endif
#ifndef uchar
#define uchar uint8_t
#endif

const int kMaxMapWidthM = 1024;
const uint kMaskOutEdge = 0x000000;
const uint kMaskInEdge = 0x800000;
const uint kMaskLaneMap = 0x007FFFFF;

namespace LC {

struct B18EdgeData {
  ushort numLines;
  uint prevInters;
  uint nextInters;
  float length;
  float maxSpeedMperSec;
  uint nextIntersMapped;
  uint prevIntersMapped;
  float curr_cum_vel = 0;
  float curr_iter_num_cars = 0;
};

struct B18IntersectionData {
  ushort state;
  ushort stateLine;
  ushort totalInOutEdges;
  uint edge[28];
  float nextEvent;
  bool isVertiport;

  // Vertiport capacity modeling (only active when isVertiport == true)
  ushort vertiportPadCapacity;    // max simultaneous takeoff/landing pads
  ushort vertiportCurrentOccupancy; // vehicles currently on pads
  ushort vertiportQueueLength;    // vehicles waiting for a pad
  float  vertiportTurnoverTimeSec;  // time per takeoff/landing operation
  float  vertiportLastDepartureTime; // for spacing enforcement
};
}

#endif // LC_B18_EDGE_DATA_H
