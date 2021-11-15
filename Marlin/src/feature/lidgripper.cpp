// Skyelar Craver @ Cellink 2021

#include "../inc/MarlinConfig.h"

#if ENABLED(LID_GRIPPER_STATION)

#include "../gcode/gcode.h"

#include "lidgripper.h"

// pin definitions
constexpr LGPins LG1_PINS {LG_EN_PIN, LG_STOP_PIN, LG_INDEX_PIN};

/// public member functions ///

LidGripper::LidGripper(const LGPins pins, uint8_t addr) : pins(pins), driver(&LG_HARDWARE_SERIAL, 0.11, addr)
{
    OUT_WRITE(pins.EN, LOW); // enable on power on in case we were gripping during power cut
    SET_INPUT(pins.STOP);
    SET_INPUT(pins.STOP);
}

void LidGripper::init_pins()
{
    LG_HARDWARE_SERIAL.begin(115200);

    TMC2208_n::GCONF_t gconf{0};
    gconf.pdn_disable = true;      // Use UART
    gconf.mstep_reg_select = true; // Select microsteps with UART
    gconf.i_scale_analog = false;  // will be set digitally
    gconf.en_spreadcycle = false;  // use stealthcop
    gconf.index_step = true;       // index will output generated step pulses
    driver.GCONF(gconf.sr);
    driver.stored.stealthChop_enabled = true;

    TMC2208_n::CHOPCONF_t chopconf{0};
    chopconf.tbl = 0b01; // blank_time = 24
    chopconf.toff = 4;
    chopconf.intpol = true;
    chopconf.hend = 2 + 3;
    chopconf.hstrt = 1 - 1;
    driver.CHOPCONF(chopconf.sr);

    driver.rms_current(LID_GRIPPER_CURRENT, HOLD_MULTIPLIER);
    driver.microsteps(32);
    driver.iholddelay(10);
    driver.TPOWERDOWN(128); // ~2s until driver lowers to hold current

    driver.homing_threshold(constrain(LID_GRIPPER_STALL_SENSITIVITY, 0, 255));
    tmc_enable_stallguard(driver);

    driver.GSTAT(0b111); // Clear
    delay(200);

    uint32_t status = driver.DRV_STATUS();
    if (status == 0xFFFF'FFFF)
        SERIAL_ERROR_MSG("lid gripper bad status: ", status);
}

const int32_t LidGripper::grip()
{
    move_to_gripper();

    const uint32_t move_steps = move_gripper_until_stall(Dir::GRIP);

    if (move_steps > LID_GRIPPER_DETECTION_THRESHOLD)
    {
        if (DEBUGGING(INFO))
            SERIAL_ECHOLNPAIR("retracting due to grip threshold passed: ", LID_GRIPPER_DETECTION_THRESHOLD);
        retract();
    }
    else
    {
        gripping = true;
        move_from_gripper();
    }

    return move_steps;
}

void LidGripper::retract()
{
    move_to_gripper();
    move_gripper_until_stall(Dir::RETRACT);
    gripping = false;
    move_from_gripper();
}

const int32_t LidGripper::calibrate()
{
    // TODO: check for bed location, maybe check if currently gripping
    const int32_t start_steps = move_gripper_until_stall(Dir::RETRACT);
    const int32_t close_steps = move_gripper_until_stall(Dir::GRIP);
    const int32_t open_steps = move_gripper_until_stall(Dir::RETRACT);

    // error was encountered in moves, discard results
    if (start_steps < 0 || close_steps < 0 || open_steps < 0)
        return -1;

    // start steps is worst case == close == open, larger value
    // indicates that the motor failed to stall
    if (start_steps > close_steps || start_steps > open_steps)
        return -2;

    // Margin of error should be < 0.1%, this approximates the average
    // of open and close operations / 1000 (>> 11 approximates / 2048)
    const int32_t steps_threshold = (close_steps + open_steps) >> 11;
    if (abs(open_steps - close_steps) > steps_threshold)
        return -3;

    return close_steps;
}

/// private member functions ///

void LidGripper::move_from_gripper()
{
    SET_SOFT_ENDSTOP_LOOSE(true);
    do_blocking_move_to_z(10.0);
    SET_SOFT_ENDSTOP_LOOSE(false);
}

void LidGripper::move_to_gripper()
{
    home_if_needed(true);
    SET_SOFT_ENDSTOP_LOOSE(true);
    do_blocking_move_to_z(10);
    do_blocking_move_to_xy(gripper_location.x, gripper_location.y);
    do_blocking_move_to_z(gripper_location.z);
    SET_SOFT_ENDSTOP_LOOSE(false);
}

const uint32_t LidGripper::move_gripper_until_stall(Dir direction)
{
    // always make sure gripper is on
    WRITE(pins.EN, LOW);

    uint32_t steps = 0;
    bool stall_triggered = false;

    if DEBUGGING (INFO)
    {
        tmc_print_sgt(driver);
        tmc_print_current(driver);
    }

    auto stop = [this, &stall_triggered, &steps]
    {
        driver.VACTUAL(0);
        stall_triggered = true;
        if (DEBUGGING(INFO))
        {
            SERIAL_ECHOLNPAIR("LG stall value: ", driver.SG_RESULT());
            SERIAL_ECHOLNPAIR("Grip stall steps: ", steps);
        }
        detachInterrupt(pins.STOP);
    };

    attachInterrupt(pins.STOP, stop, RISING);
    attachInterrupt(
        pins.INDEX, [&steps]
        { ++steps; },
        CHANGE);

    int velocity = ((direction == Dir::GRIP) ? LID_GRIPPER_VELOCITY : -(LID_GRIPPER_VELOCITY));
    driver.VACTUAL(velocity);

    // wait until stall, passed threshold, or timeout
    auto timeout = millis() + 5000;
    while (!stall_triggered && steps < LID_GRIPPER_DETECTION_THRESHOLD && millis() < timeout)
    {
        delay(500);
        idle();
    }

    if (!stall_triggered)
    {
        stop();
        SERIAL_ERROR_MSG("Lid Gripper did not stall!");
    }

    // if we've just retracted, no need to keep motor on
    if (direction == Dir::RETRACT)
        WRITE(pins.EN, HIGH);

    return steps;
}

/// extern variable definitions ///

LidGripper lid_gripper(LG1_PINS, LG_SLAVE_ADDRESS);

/// Gcode implementations ///

void GcodeSuite::G500()
{
    lid_gripper.calibrate();
}

void GcodeSuite::G501()
{
    // TODO: add any debugging/printout/parsing logic
    lid_gripper.grip();
}

void GcodeSuite::G502()
{
    // TODO: add any debugging/printout/parsing logic
    lid_gripper.retract();
}

#endif // LID_GRIPPER_STATION