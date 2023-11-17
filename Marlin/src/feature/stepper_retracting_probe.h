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
#include "../module/planner.h"

#include <optional>

#include "simple_TMC_controller.h"

/**
 * @brief Controller for a probe using a stepper motor to deploy and retract
 * 
 */
struct StepperRetractingProbe
{
    static constexpr int32_t SRP_DEPLOY_VELOCITY = -100'000;
    static constexpr int32_t SRP_STOW_VELOCITY = 100'000;
    static constexpr uint8_t SRP_STALL_THRESHOLD = 120;
    static constexpr uint32_t SRP_STEPPER_CURRENT = 1000;
    static constexpr uint32_t SRP_RETRACT_TIME = 7000;
    static constexpr float SRP_STEPPER_RSENSE = 0.11f;

    /**
     * @brief POD configuration object
     * 
     */
    struct Config
    {
        int32_t deploy_velocity{SRP_DEPLOY_VELOCITY};
        int32_t stow_velocity{SRP_STOW_VELOCITY};
        int16_t stall_threshold{SRP_STALL_THRESHOLD};
        uint16_t stepper_current{SRP_STEPPER_CURRENT};
        uint32_t minimum_retract_time{SRP_RETRACT_TIME};
    };

    void deploy();

    void stow();

    constexpr const Config& get_config() const noexcept { return config; }

    inline void set_config(Config conf)
    {
        config = conf;
        reinit_driver();
    }

    void report_config(bool for_replay) const;

    constexpr inline void reset_position() noexcept { state = ProbeState::Unknown; }

    [[nodiscard]] constexpr inline bool is_deployed() const noexcept
    {
        return state == ProbeState::Deployed;
    }

    inline void reinit_driver() noexcept
    { // reset stepper to force reinitialization next use
        _stepper.reset();
    }

private:
    using STMC = SimpleTMC<PROBE_EN_PIN, PROBE_STOP_PIN>;

    Config config{};

    enum class ProbeState {
        Unknown,
        Stowed,
        Deployed
    };

    ProbeState state{ProbeState::Unknown};

    std::optional<STMC> _stepper{};

    void stepper_init();

    inline STMC& stepper()
    {
        if (!_stepper.has_value())
            stepper_init();
        return *_stepper;
    }

    void unstick(int32_t velocity);
};

extern StepperRetractingProbe stepper_probe;