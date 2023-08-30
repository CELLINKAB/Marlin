#pragma once

#include "../inc/MarlinConfig.h"

#include "cellink_reporter.h"

struct UVCController
{
    static constexpr millis_t DEFAULT_EXPOSURE_SECONDS = 300;
    static constexpr millis_t MAX_EXPOSURE_SECONDS = 1'200;

    millis_t auto_off_time;
    bool safety_override;

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
