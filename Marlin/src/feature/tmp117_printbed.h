//copyright cellink 2022 - GPLv3

#pragma once

#include <array>
#include "tmp117/TMP117.h"

double get_tmp117_bed_temp();

using BedSensors = std::array<TMP117<TwoWire>, 4>;

BedSensors& bed_sensors();