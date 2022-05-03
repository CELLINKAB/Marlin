// Skyelar Craver @ Cellink 2021

#pragma once

#include "../inc/MarlinConfig.h"
#include "../module/planner.h"
#include "../module/stepper/trinamic.h"

#include "tmc_util.h"

struct LGPins
{
    const pin_t EN;
    const pin_t STOP;
    const pin_t INDEX;
};

struct LidGripper
{
    explicit LidGripper(const LGPins, uint8_t addr);

    // only call once during bootstrapping/setup!
    // sets up TMC driver and pwm/timers
    void init_pins();

    // run the gripper until a stall is triggered
    // counts steps to detect if no lid is present
    const int32_t grip();

    // run the gripper to its openmost position
    void retract();

    // calibrate the full range of the lid gripper for proper lid detection
    const int32_t calibrate();

private:
    const LGPins pins;
    TMCMarlin<TMC2209Stepper, 'L', '0', AxisEnum::NO_AXIS_ENUM> driver;
    bool gripping = true; // assume gripping at startup in case of sudden power loss
    const xyz_pos_t gripper_location LID_GRIPPER_COORDS; // could be const, but operators are not marked const

    // typesafe direction control
    enum class Dir
    {
        RETRACT,
        GRIP
    };

    void move_from_gripper();

    void move_to_gripper();

    const uint32_t move_gripper_until_stall(Dir);
};

#if ENABLED(LID_GRIPPER_STATION)
// #if !PINS_EXIST(LG_STEP, LG_DIR, LG_EN, LG_STOP)
//     #error "The pins LG_STEP, LG_DIR, LG_EN, and LG_STOP must be defined to use a lid gripper!"
// #endif
extern LidGripper lid_gripper;
#endif