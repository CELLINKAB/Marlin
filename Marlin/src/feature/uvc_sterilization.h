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

#include "cellink_reporter.h"

struct UVCController
{
    static constexpr millis_t DEFAULT_EXPOSURE_SECONDS = 600;
    static constexpr millis_t MAX_EXPOSURE_SECONDS = 3'600;

    millis_t auto_off_time;
    bool safety_override;
    bool send_reports;

    /**
     * @brief One time setup for pin configuration. No-op if called multiple times
     * 
     */
    static void init();

    /**
     * @brief If current sense pins are configured, provide a readout
     * 
     */
    static void report_current_sense();

    /**
     * @brief Returns status of configured overtemp pins unless safety is disabled
     * 
     * @return true warning triggered, device should turn off
     * @return false no pin configured, safety disabled, or warning pin inactive
     */
    bool ot_prewarn();

    /**
     * @brief Returns whether the UVC is currently active based on input time
     * 
     * @param now current system time in millis, defaults to grabbing current millis()
     * @return true UVC active
     * @return false UVC inactive
     */
    inline bool running(millis_t now = millis()) { return now < auto_off_time; }

    /**
     * @brief Start a sterilization cycle
     * 
     * @param intensity PWM value
     */
    void start(uint8_t intensity);

    /**
     * @brief Called regularly to manage sterilization
     * 
     * @param now 
     */
    void update(millis_t now = millis());

    /**
     * @brief Stop an active sterilization now
     * 
     */
    void stop();

private:
    static void write_uvc_switches(bool state);
};

extern UVCController uvc_controller;
