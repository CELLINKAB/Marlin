// Skyelar Craver @ Cellink 2021

#include "../inc/MarlinConfig.h"

#if ENABLED(LID_GRIPPER_STATION)

#include "../gcode/gcode.h"

#include "lidgripper.h"

// #if PIN_EXISTS(LG_RX) && PIN_EXISTS(LG_TX)
//     static HardwareSerial accessory_serial(LG_RX_PIN, LG_TX_PIN);
// #elif PIN_EXISTS(LG_RXTX)
//     static HardwareSerial accessory_serial(LG_RXTX_PIN);
// #else
//     #error "can't construct a serial accessory for lid gripper! \n" \
//         "Please make sure LG_RX and LG_TX, or LG_RXTX pins are defined"
// #endif

HardwareSerial LGSerial(LG_RX_PIN, LG_TX_PIN);
LidGripper<LG_STEP_PIN,LG_DIR_PIN,LG_EN_PIN,LG_STOP_PIN> lid_gripper(LGSerial);

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