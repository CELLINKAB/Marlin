//copyright cellink 2022 - GPLv3

#pragma once

#include <array>
#include "tmp117/TMP117.h"

double get_tmp117_bed_temp();

struct BedSensor : public TMP117<TwoWire>
{
    float scalar = 1.0f;
}

using BedSensors = std::array<BedSensor, 4>;

BedSensors& bed_sensors();