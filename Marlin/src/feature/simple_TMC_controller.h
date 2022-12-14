// Copyright Cellink 2022 - GPL-v3

#pragma once

#include "../inc/MarlinConfig.h"

#include <type_traits>

#include "tmc_util.h"

/**
 * @brief configuration parameters for TMC2209 steppers
 *
 * @param hw_address serial address of TMC2209 to control, 0-3
 * @param stall_sensitivity stallguard threshold value, higher sensitivity stalls easier, 0 is impossible to stall
 * @param rms_current maximum RMS current draw allowed by the driver in milliamps
 * @param rsense current sense resistor value
 *
 */
struct SimpleTMCConfig
{
    constexpr SimpleTMCConfig(uint8_t address, uint8_t sensitivity)
        : hw_address(address)
        , stall_sensitivity(sensitivity)
        , rms_current(600)
        , rsense(0.11f)
    {}
    constexpr SimpleTMCConfig(uint8_t address, uint8_t sensitivity, uint32_t current, float sense_resistor)
        : hw_address(address)
        , stall_sensitivity(sensitivity)
        , rms_current(current)
        , rsense(sense_resistor)
    {}
    uint8_t hw_address;
    uint8_t stall_sensitivity;
    uint32_t rms_current;
    float rsense;
};

/**
 * @brief TMC controller for quickly integrating a stepper with high level movement methods
 *
 * @tparam EN TMC2209 ENABLE pin number
 * @tparam STOP TMC2209 DIAG pin number
 * @tparam INDEX (optional) TMC2209 index pin number
 */
template<pin_t EN, pin_t STOP, pin_t STEP = -1, pin_t DIR = -1, pin_t INDEX = -1>
struct SimpleTMC
{
    
    SimpleTMC(SimpleTMCConfig config_, Stream* serial)
        : config(config_)
        , driver(serial, config_.rsense, config_.hw_address)
    {
        init_driver();
    }

    SimpleTMC(SimpleTMCConfig config_, pin_t rx_pin, pin_t tx_pin)
        : config(config_)
        , driver(rx_pin, tx_pin, config_.rsense, config_.hw_address)
    {
        init_driver();
    }
    
    /**
     * @brief Move the motor at a velocity with no checking or feedback
     *
     * @param velocity signed step rate, negative values reverse direction
     */
    void raw_move(const int32_t velocity)
    {
        WRITE(EN, LOW);
        driver.VACTUAL(velocity);
    }

    /**
     * @brief begin moving and return asynchronously until stallguard is triggered
     *
     * @param velocity signed step rate to move stepper at, negative values reverse direction
     * @param callback (optional) function to call when stall condition is met
     */
    void move_until_stall(
        const int32_t velocity, callback_function_t callback = [] {})
    {
        auto done = [this, callback] {
            stop();
            if DEBUGGING (INFO)
                SERIAL_ECHOLNPGM("Stepper stalled, SG:", driver.SG_RESULT());
            callback();
        };
        attachInterrupt(STOP, done, RISING);
        raw_move(velocity);
    }

    /**
     * @brief begin moving and return on stall condition
     *
     * @param velocity signed step rate to move stepper at, negative values reverse direction
     * @param timeout unsigned milliseconds after which the motor will stop regardless
     */
    void blocking_move_until_stall(const int32_t velocity, millis_t timeout = 0)
    {
        millis_t end_time = 0;
        if (timeout > 0)
            end_time = millis() + timeout;
        move_until_stall(velocity);
        while (driver.VACTUAL() != 0) {
            delay(1000);
            if DEBUGGING (INFO)
                SERIAL_ECHOLNPGM("Stepper moving, SG:", driver.SG_RESULT());
            idle();
            if (timeout > 0 && millis() > end_time)
                stop();
        }
    }

    /**
     * @brief stop any current moves and disable stepper
     *
     */
    void stop()
    {
        driver.VACTUAL(0);
        WRITE(EN, HIGH);
    }

    /**
     * @brief if index pin is specified, count steps while moving until stall or specified
     *
     * @param velocity signed step rate, negative values reverse direction
     * @param max_steps (optional) threshold value to stop motor regardless of stall state
     * @return uint32_t actual number of steps generated
     */
    template<bool COUNTS = (INDEX > -1), class = std::enable_if_t<COUNTS>>
    uint32_t blocking_move_until_count_or_stall(const int32_t velocity, const uint32_t max_steps = ~0)
    {
        uint32_t count = 0;
        attachInterrupt(
            INDEX,
            [&count, max_steps, this] {
                ++count;
                if (count >= max_steps)
                    stop();
            },
            CHANGE);
        blocking_move_until_stall(velocity);
        detachInterrupt(INDEX);
        return count;
    }

    void rms_current(uint16_t current) { driver.rms_current(current); }

    void stall_threshold(int16_t threshold) { driver.homing_threshold(threshold); }

    void reinit_driver() { init_driver(); }

private:
    SimpleTMCConfig config;
    TMCMarlin<TMC2209Stepper, 'N', '0', AxisEnum::NO_AXIS_ENUM> driver;

    void init_driver()
    {
        OUT_WRITE(EN, HIGH);
        SET_INPUT_PULLUP(STOP);
        if constexpr (INDEX != 0)
            SET_INPUT(INDEX);

        TMC2208_n::GCONF_t gconf{};
        gconf.pdn_disable = true;      // Use UART
        gconf.mstep_reg_select = true; // Select microsteps with UART
        gconf.i_scale_analog = false;  // will be set digitally
        gconf.en_spreadcycle = false;  // use stealthcop
        if constexpr (INDEX != 0)
            gconf.index_step = true; // index will output generated step pulses
        driver.GCONF(gconf.sr);
        driver.stored.stealthChop_enabled = true;

        TMC2208_n::CHOPCONF_t chopconf{};
        chopconf.tbl = 0b01; // blank_time = 24
        chopconf.toff = 4;
        chopconf.intpol = true;
        chopconf.hend = 2 + 3;
        chopconf.hstrt = 1 - 1;
        driver.CHOPCONF(chopconf.sr);

        driver.rms_current(config.rms_current, HOLD_MULTIPLIER);
        driver.microsteps(32);
        driver.iholddelay(10);
        driver.TPOWERDOWN(128); // ~2s until driver lowers to hold current

        driver.homing_threshold(config.stall_sensitivity);
        tmc_enable_stallguard(driver);

        driver.GSTAT(0b111); // Clear
        delay(200);          // ensures serial transfers finish

        uint32_t status = driver.DRV_STATUS();
        if (status == 0xFFFF'FFFF)
            SERIAL_ERROR_MSG("driver bad status");
    }

};
