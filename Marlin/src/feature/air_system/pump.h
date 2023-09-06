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

#include "analog_sensor.h"
#include "constants.h"

namespace pneumatics {

// instance counter that enables pressure whenever something is using it
template<const pin_t PUMP, const pin_t VALVE, const pin_t SENSE>
struct Pump
{
    explicit Pump(const AnalogPressureSensor<SENSE>& sensor)
        : sensor_(sensor)
        , suction_users(0)
        , pressure_users(0)
    {
        OUT_WRITE(PRESSURE_VALVE_PUMP_OUT_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
        OUT_WRITE(PRESSURE_PUMP_EN_PIN, LOW);
    }

    void update()
    {
        if (suction_users > 0) {
            WRITE(VALVE, PRESSURE_VALVE_CLOSE_LEVEL);
            WRITE(PUMP, HIGH);
            return;
        }

        if (const float current_pressure = sensor_.read_avg();
            current_pressure < TANK_PRESSURE_MAINTAINENCE
            || (pressure_users > 0 && current_pressure < TANK_PRESSURE_TARGET)) {
            WRITE(VALVE, PRESSURE_VALVE_OPEN_LEVEL);
            WRITE(PUMP, HIGH);
        } else {
            if (current_pressure > TANK_PRESSURE_MAX)
                SERIAL_ECHOLN("PRESSURE_MAX_EXCEEDED");

            WRITE(VALVE, PRESSURE_VALVE_CLOSE_LEVEL);
            WRITE(PUMP, LOW);
        }
    }

private:
    static constexpr float TANK_PRESSURE_TARGET = 75.0f;
    static constexpr float TANK_PRESSURE_MAINTAINENCE = 50.0f;
    static constexpr float TANK_PRESSURE_MAX = 200.0f;

    const AnalogPressureSensor<SENSE>& sensor_;

    size_t suction_users;
    size_t pressure_users;

    enum class LockType {
        Suction,
        Pressure,
    };

    void add_user(LockType kind)
    {
        switch (kind) {
        case LockType::Suction:
            ++suction_users;
            break;
        case LockType::Pressure:
            ++pressure_users;
            break;
        }
        update();
    }

    void remove_user(LockType kind)
    {
        switch (kind) {
        case LockType::Suction:
            --suction_users;
            break;
        case LockType::Pressure:
            --pressure_users;
            break;
        }
        update();
    }

    struct [[maybe_unused]] Lock
    {
        explicit Lock(Pump* parent, LockType kind)
            : parent_(*parent)
            , kind_(kind)
        {
            parent_.add_user(kind_);
        }
        ~Lock() { parent_.remove_user(kind_); }
        Lock(const Lock& token) = delete;
        Lock(Lock&& token) = delete;
        Lock& operator=(const Lock& token) = delete;
        Lock& operator=(Lock&& token) = delete;

    private:
        Pump& parent_;
        LockType kind_;
    };

    friend class Lock;

public:
    inline Lock use_pressure() { return Lock(this, LockType::Pressure); }
    inline Lock use_suction() { return Lock(this, LockType::Suction); }
}; // Pump

extern Pump<PRESSURE_PUMP_EN_PIN, PRESSURE_VALVE_PUMP_OUT_PIN, PRESSURE_TANK_PIN> pump;

} // namespace pneumatics