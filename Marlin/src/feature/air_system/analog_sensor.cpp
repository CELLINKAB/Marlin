/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 Cellink [https://github.com/CELLINKAB/Marlin]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#    include "analog_sensor.h"

using namespace pneumatics;

//
// constants
//

static constexpr float TANK_GAIN = 0.183239119;
static constexpr float REGULATOR_GAIN = 0.04;
static constexpr float VACUUM_GAIN = -0.02;

static constexpr float TANK_OFFSET = 150.0f;
static constexpr float VACUUM_OFFSET = -16.5f;
static constexpr float REGULATOR_OFFSET = 0.0f;

//
// statics
//

AnalogPressureSensor<GRIPPER_VACUUM_PIN> pneumatics::gripper_vacuum(VACUUM_GAIN, VACUUM_OFFSET);
AnalogPressureSensor<PRESSURE_TANK_PIN> pneumatics::tank_pressure(TANK_GAIN, TANK_OFFSET);
AnalogPressureSensor<PRESSURE_REGULATOR_SENSE_PIN> pneumatics::regulator_feedback(REGULATOR_GAIN, REGULATOR_OFFSET);

#endif // FESTO_PNEUMATICS