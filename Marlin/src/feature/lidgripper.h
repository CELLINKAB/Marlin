// Skyelar Craver @ Cellink 2021

#pragma once

#include "../inc/MarlinConfig.h"

#include "tmc_util.h"

#include "../module/planner.h"

#define LG_GRIP_DIR HIGH
#define LG_RETRACT_DIR !LG_GRIP_DIR

#define LG_TIMER timer_instance[STEP_TIMER_NUM]
#define LG_TIMER_CHANNEL 2
//#define _LG_CHANNEL(n) TIM_CHANNEL_ ## n
#define LG_CHANNEL TIM_CHANNEL_2

template <const pin_t STEP, const pin_t DIR, const pin_t EN, const pin_t STOP>
struct LidGripper
{
    // I want to make these const, but many operator
    // overloads aren't marked as const
    xyz_pos_t gripper_location LID_GRIPPER_COORDS;
    //uint32_t lid_detection_threshold {LID_GRIPPER_DETECTION_THRESHOLD};

    //explicit LidGripper(HardwareSerial * serial);
    LidGripper(HardwareSerial& serial) : driver(&serial, 0.11, LG_SLAVE_ADDRESS) 
    {
        static_assert((STEP != DIR && STEP != EN && STEP != STOP && 
                       DIR != EN && DIR != STOP && EN != STOP),
        "lid gripper pins cannot alias eachother!");
    }

    void init_pins()
    {
        SET_OUTPUT(LG_STEP_PIN);
        SET_OUTPUT(LG_DIR_PIN);
        OUT_WRITE(LG_EN_PIN, LOW);
        SET_INPUT(LG_STOP_PIN);       
        // _serial->begin(115200); 

        static_assert(LID_GRIPPER_STALL_SENSITIVITY >= 0 && LID_GRIPPER_STALL_SENSITIVITY <= 255,
                        "LID_GRIPPER_STALL_SENSITIVITY must be between 0 and 255!");

        //driver.beginSerial(115200);
        driver.begin();
        driver.toff(4);
        driver.blank_time(24);
        driver.rms_current(LID_GRIPPER_CURRENT);
        driver.microsteps(32);
        driver.GSTAT();
        tmc_enable_stallguard(driver);

        //LG_TIMER->attachInterrupt(LG_CHANNEL, [this](){this->step_motor<STEP>();});
        //LG_TIMER->pauseChannel(LG_CHANNEL);
    }

    // run the gripper until a stall is triggered
    // counts steps to detect if no lid is present
    int32_t grip()
    {
        move_to_gripper();
        enable();
        set_dir(LG_GRIP_DIR);

        uint32_t move_steps = move_gripper_until_stall();

        if (DEBUGGING(INFO)) SERIAL_ECHOLNPAIR("Grip stall steps: ", move_steps);

        if (move_steps < LID_GRIPPER_DETECTION_THRESHOLD)
        {
            if (DEBUGGING(INFO)) SERIAL_ECHOLNPAIR("retracting due to grip threshold passed: ", LID_GRIPPER_DETECTION_THRESHOLD);
            retract();
        }
        

        // attachInterrupt(LG_STOP_PIN, [this](){
        //     LG_TIMER->pauseChannel(LG_CHANNEL);
        //     detachInterrupt(LG_STOP_PIN);
        //     const auto drv_status = this->driver.DRV_STATUS();
        //     const auto move_steps = this->current_move_steps;
        //     this->current_move_steps = 0;
        //     if (move_steps > LID_GRIPPER_DETECTION_THRESHOLD)
        //         this->retract();
        //     else
        //         this->move_from_gripper();
        //     // TODO: log this stuff
        // }, CHANGE);
        //LG_TIMER->resumeChannel(LG_CHANNEL);


        return move_steps;
    } 

    // run the gripper to its openmost position
    int32_t retract()
    {
        move_to_gripper();
        set_dir(LG_RETRACT_DIR);
        // attachInterrupt(LG_STOP_PIN, [this](){
        //     LG_TIMER->pauseChannel(LG_CHANNEL);
        //     detachInterrupt(LG_STOP_PIN);
        //     const auto drv_status = this->driver.DRV_STATUS();
        //     this->current_move_steps = 0;
        //     this->move_from_gripper();
        // }, CHANGE);
        // LG_TIMER->resumeChannel(LG_CHANNEL);

        move_gripper_until_stall();

        disable();

        move_from_gripper();
        return 0;
    }

    // calibrate the full range of the lid gripper for proper lid detection
    int32_t calibrate()
    {
        // TODO: check for bed location, maybe check if currently gripping
        set_dir(LG_RETRACT_DIR);
        const int32_t start_steps = move_gripper_until_stall();
        set_dir(LG_GRIP_DIR);
        const int32_t close_steps = move_gripper_until_stall();
        set_dir(LG_RETRACT_DIR);
        const int32_t open_steps = move_gripper_until_stall();

        if (start_steps < 0 || close_steps < 0 || open_steps < 0)
            return -1;

        if (start_steps > close_steps || start_steps > open_steps)
            return -2;

        // Margin of error should be < 0.1%, this approximates the average
        // of open and close operations / 1000 (>> 11 approximates / 2048)
        const int32_t steps_threshold = (close_steps + open_steps) >> 11;
        if (abs(open_steps - close_steps) > steps_threshold)
            return -3;

        calibrated_step_range = static_cast<uint32_t>(close_steps); 

        if (DEBUGGING(INFO)) SERIAL_ECHOLNPAIR("calibrated step range:", calibrated_step_range);

        return close_steps;
    }

 private:
    TMCMarlin<TMC2209Stepper, 'L', '0', AxisEnum::NO_AXIS_ENUM> driver;
    uint32_t calibrated_step_range = 0;

    void move_from_gripper()
    {
        SET_SOFT_ENDSTOP_LOOSE(true);
        do_blocking_move_to_z(10.0);
        SET_SOFT_ENDSTOP_LOOSE(false);
        planner.synchronize();
    }

    void move_to_gripper()
    {
        home_if_needed(true);
        SET_SOFT_ENDSTOP_LOOSE(true);
        do_blocking_move_to_z(10);
        do_blocking_move_to_xy(gripper_location.x, gripper_location.y);
        do_blocking_move_to_z(gripper_location.z);
        SET_SOFT_ENDSTOP_LOOSE(false);

        planner.synchronize();
    }

    uint32_t move_gripper_until_stall()
    {
        uint32_t steps = 0;
        uint32_t idle_after_steps = 800;

        while (/*READ(LG_STOP_PIN) == LOW*/ steps < 8000)
        {
            step_motor();
            ++steps;
            if (steps > idle_after_steps)
            {
                idle_after_steps += 800;
                idle();
                if (DEBUGGING(INFO)) SERIAL_ECHOLNPAIR("LG stall value: ", driver.SG_RESULT());
            }
            delayMicroseconds(100);
        }

        //driver.GSTAT(); // clear diag line

        return steps;
    }


    inline void step_motor()
    {
        WRITE(STEP, HIGH);
        delayMicroseconds(100);
        WRITE(STEP, LOW);
    }

    inline void set_dir(bool value) { WRITE(DIR, value); }

    inline void enable() { WRITE(DIR, LOW); }

    inline void disable() { WRITE(EN, HIGH); }
};

#if ENABLED(LID_GRIPPER_STATION)
  #if !PINS_EXIST(LG_STEP,LG_DIR,LG_EN,LG_STOP)
    #error "The pins LG_STEP, LG_DIR, LG_EN, and LG_STOP must be defined to use a lid gripper!"
  #endif
  extern LidGripper<LG_STEP_PIN,LG_DIR_PIN,LG_EN_PIN,LG_STOP_PIN> lid_gripper;
  extern HardwareSerial LGSerial;
#endif