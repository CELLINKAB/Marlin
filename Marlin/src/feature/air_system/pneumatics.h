
#pragma once

#include "../../inc/MarlinConfigPre.h"

#define AUTO_REPORT_PNEUMATIC_SENSORS 1
#if ENABLED(AUTO_REPORT_PNEUMATIC_SENSORS)
#    include "../../libs/autoreport.h"
#endif

#include "analog_sensor.h"
#include "pump.h"
#include "regulator.h"

namespace pneumatics {

void init();

void update();

enum class GripperState {
    Close,
    Release,
    Grip,
};

void set_gripper_valves(GripperState state);

void apply_mixing_pressure(uint8_t tool);
void release_mixing_pressure(uint8_t tool);

void report_sensors();

#if ENABLED(AUTO_REPORT_PNEUMATIC_SENSORS)
struct Reporter : AutoReporter<Reporter>
{
    static void report();
};

extern Reporter reporter;
#endif

} // namespace pneumatics