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

#include "../inc/MarlinConfig.h"

/**
 * @brief sticky pin reader to handle bouncy switches
 * 
 * @tparam PIN 
 */
template<pin_t PIN>
struct Debounced
{
    const millis_t period;
    const bool inverting;
    millis_t debounce_until;
    bool state;

    /**
     * @brief Construct a new Debounced object
     * 
     * @param debounce_time 
     * @param invert 
     * @param pullup 
     */
    explicit Debounced(millis_t debounce_time, bool invert = false, bool pullup = false)
        : period(debounce_time)
        , inverting(invert)
        , debounce_until(0)
        , state(false)
    {
        if (pullup) {
            SET_INPUT_PULLUP(PIN);
        } else {
            SET_INPUT_PULLDOWN(PIN);
        };
    }

    /**
     * @brief read current raw state accounting for inverting, but without debouncing
     * 
     * @return true 
     * @return false 
     */
    bool read_now() const { return static_cast<bool>(READ(PIN)) ^ inverting; }

    /**
     * @brief read debounced pin state
     * 
     * @return true 
     * @return false 
     */
    bool read() const { return state; }

    /**
     * @brief poll in background to update debouncing timers
     * 
     * @param now 
     */
    void update(millis_t now = millis())
    {
        if (now <= debounce_until)
            return;

        if (debounce_until != 0) {
            state = read_now();
            debounce_until = 0;
        }
        else if (state != read_now()) {
            debounce_until = now + period;
        }
    }


};