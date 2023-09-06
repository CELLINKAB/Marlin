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

#pragma once

#include "../../inc/MarlinConfig.h"

namespace pneumatics {

template<const pin_t REG>
struct Regulator
{
    inline void set_point(float kPa)
    {
        kPa = constrain(kPa, 0.0, REG_MAX);
        uint32_t value = static_cast<uint32_t>(kPa * DAC_FACTOR);
        analogWrite(REG, value);
        pressure = kPa;
    }
    inline float set_point() const { return pressure; }

private:
    static constexpr float REG_MAX = (3.3 / 5.0) * 100.0;
    static constexpr float DAC_FACTOR = 256.0 / REG_MAX;

    float pressure;
};

extern Regulator<PRESSURE_REGULATOR_PIN> regulator;

} // namespace pneumatics
