// copyright a thing in a place with a license by some people

#include "../../gcode/gcode.h"
#include "../../gcode/parser.h"
#include "../../inc/MarlinConfig.h"
#include "request.h"

#define CHANT_MAX_MSG_LEN 128

namespace printhead
{

void debug_echo_cmd(const char* msg)
{
    [[maybe_unused]] static printhead::Controller ph_1 = []() {
        //CHANT_SERIAL.setHalfDuplex();
        CHANT_SERIAL.begin(115200);
        OUT_WRITE(CHANT_RTS_PIN, HIGH);
        return printhead::Controller(CHANT_SERIAL, printhead::Index::One);
    }();
    char buf[CHANT_MAX_MSG_LEN]{};
    uint8_t crc_buf[sizeof(uint16_t)]{};
    const size_t msg_len = strlen(msg);
    static_assert(sizeof(char) == sizeof(uint8_t), "expected char width to be 8 bits");
    printhead::Packet packet(Index::One, Command::SYRINGEPUMP_20ML_START, msg, msg_len);
    SERIAL_ECHOLNPGM_P("sending '", msg, "' to chant, ", msg_len, " bytes, crc:", packet.crc);
    printhead::send(packet, CHANT_SERIAL);
    WRITE(CHANT_RTS_PIN, LOW);
    Response response = receive(CHANT_SERIAL);
    WRITE(CHANT_RTS_PIN, HIGH);
    if (response.result != Result::OK)
        {SERIAL_ECHO("Packet returned bad!");
        return;}
    SERIAL_ECHOLN("Packet returned OK!");
    SERIAL_ECHOLNPGM("New CRC:", response.packet.crc);
    //SERIAL_ECHOLNPGM_P("received '", (const char*)response.packet.payload, "' from chant, ", response.packet.payload_size, " bytes, crc:", response.packet.crc);
}

constexpr const char* command_switch(uint32_t command)
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

} // namespace printhead

void GcodeSuite::M1069()
{
    uint32_t command = parser.ulongval('C');

    printhead::debug_echo_cmd(printhead::command_switch(command));
}
