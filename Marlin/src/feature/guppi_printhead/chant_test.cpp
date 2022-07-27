// copyright a thing in a place with a license by some people

#include "../../gcode/gcode.h"
#include "../../gcode/parser.h"
#include "../../inc/MarlinConfig.h"

#define CHANT_MAX_MSG_LEN 128

void debug_echo_cmd(const char* msg)
{
    [[maybe_unused]] static bool bootstrap = []() {
        //CHANT_SERIAL.setHalfDuplex();
        CHANT_SERIAL.begin(115200);
        OUT_WRITE(CHANT_RTS_PIN, HIGH);
        return true;
    }();
    char buf[CHANT_MAX_MSG_LEN]{};
    const size_t msg_len = strlen(msg);
    SERIAL_ECHOLNPGM_P("sending '", msg, "' to chant, ", msg_len, " bytes");
    CHANT_SERIAL.print(msg);
    WRITE(CHANT_RTS_PIN, LOW);
    const size_t new_len = CHANT_SERIAL.readBytes(buf, msg_len + 1);
    WRITE(CHANT_RTS_PIN, HIGH);
    SERIAL_ECHOLNPGM_P("received '", buf, "' from chant, ", new_len, " bytes");
}

const char* command_switch(uint32_t command)
{
    switch (command) {
    case 0:
        return "null";
    case 1:
        return "one";
    case 20:
        return "extrude 20 units";
    default:
        return "unsupported";
    }
}

void GcodeSuite::M1069()
{
    uint32_t command = parser.ulongval('C');

    debug_echo_cmd(command_switch(command));
}
