#ifndef TRACK_DATA_TABLE_H
#define TRACK_DATA_TABLE_H

#include <Arduino.h>
#include "TrackDataStruct.h"

// Left side
extern const TrackData leftDataTable[];
constexpr const size_t leftDataTableSize = 28;

// Right side
extern const TrackData rightDataTable[];
constexpr const size_t rightDataTableSize = 28;

#endif