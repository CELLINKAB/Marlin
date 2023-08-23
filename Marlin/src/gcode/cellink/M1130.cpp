// copyright Cellink 2023 - GPLv3

#include "../../feature/cellink_reporter.h"
#include "../gcode.h"

#if PINS_EXIST(CS_24V, CS_BED_24V_CS)

void GcodeSuite::M1130()
{
    pinMode(CS_24V_PIN, INPUT_ANALOG);
    pinMode(CS_BED_24V_CS_PIN, INPUT_ANALOG);

    auto current_1 = analogRead(CS_24V_PIN);
    auto current_2 = analogRead(CS_BED_24V_CS_PIN);

    // TODO: scale these values properly

    cellink::serial_echoln_kv("LOAD_SWITCH_1_CURRENT", current_1, "LOAD_SWITCH_2_CURRENT", current_2);
}

#endif // PINS_EXIST(CS_24V, CS_BED_24V_CS)
