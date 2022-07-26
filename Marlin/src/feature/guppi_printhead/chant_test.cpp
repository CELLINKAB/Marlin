// copyright a thing in a place with a license by some people

#include "../../gcode/gcode.h"
#include "../../gcode/parser.h"
#include "../../inc/MarlinConfig.h"

#define CHANT_MAX_MSG_LEN 128

void debug_echo_cmd(const char* msg)
{
    [[maybe_unused]] static bool bootstrap = []() {
        CHANT_SERIAL.begin();
        return true;
    };
    char buf[CHANT_MAX_MSG_LEN]{};
    const size_t msg_len = strlen(msg);
    SERIAL_ECHOLNPGM_P("sending \"", msg, "\" to chant, ", msg_len, " bytes");
    CHANT_SERIAL.print(msg);
    const size_t new_len = CHANT_SERIAL.readBytes(buf, msg_len + 1);
    SERIAL_ECHOLNPGM_P("received \"", buf, "\" from chant, ", new_len, " bytes");
}

void GcodeSuite::M1069()
{
    const char* msg = parser.value_string();
    debug_echo_cmd(msg);
}
