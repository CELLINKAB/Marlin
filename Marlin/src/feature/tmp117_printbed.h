//copyright cellink 2022 - GPLv3

#pragma once

#include <array>
#include <SoftWire.h>
#include "tmp117/TMP117.h"



double get_tmp117_bed_temp();

constexpr size_t NUM_BED_TEMP_SENSORS = 4;

using BedSensors = std::array<TMP117<SoftWire>, NUM_BED_TEMP_SENSORS>;

BedSensors& bed_sensors();