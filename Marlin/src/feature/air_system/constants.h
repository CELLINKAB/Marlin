#pragma once

#include "../../inc/MarlinConfig.h"

#ifndef PRESSURE_VALVE_CLOSE_LEVEL
#    define PRESSURE_VALVE_CLOSE_LEVEL LOW
#endif
#ifndef PRESSURE_VALVE_OPEN_LEVEL
#    define PRESSURE_VALVE_OPEN_LEVEL !PRESSURE_VALVE_CLOSE_LEVEL
#endif