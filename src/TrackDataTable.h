#ifndef TRACK_DATA_TABLE_H
#define TRACK_DATA_TABLE_H
// #define CUSTOM_TRACK // <-- Comment this line to use the EHC track data

#include <Arduino.h>
#include "TrackDataStruct.h"

// Left side
extern const TrackData leftDataTable[];

// Right side
extern const TrackData rightDataTable[];

#ifdef CUSTOM_TRACK
constexpr const size_t leftDataTableSize = 2;
constexpr const size_t rightDataTableSize = 2;
#else
constexpr const size_t leftDataTableSize = 28;
constexpr const size_t rightDataTableSize = 28;
#endif
#endif