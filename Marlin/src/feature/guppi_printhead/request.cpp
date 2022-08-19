

#include "../../inc/MarlinConfig.h"
#include "request.h"

using namespace printhead;

Result printhead::send(const Packet& request, HardwareSerial& serial)
{
    const auto header = request.header_bytes();
    const auto crc = request.crc_bytes();
    serial.write(header.data(), header.size());
    serial.write(request.payload, request.payload_size);
    serial.write(crc.data(), crc.size());

    // should read an ACK after this write to assure complete transaction

    return Result::OK;
}

Response printhead::receive(HardwareSerial& serial)
{
    // this seems bug-prone...
    static uint8_t packet_buffer[64];

    Packet incoming;

    auto bytes_received = serial.readBytes(packet_buffer, 64);

    if (bytes_received < 6)
        return Response{incoming, Result::PACKET_TOO_SHORT};
    memcpy(&incoming.ph_index, &packet_buffer[0], 2);
    memcpy(&incoming.command, &packet_buffer[2], 2);
    memcpy(&incoming.payload_size, &packet_buffer[4], 2);
    if (incoming.payload_size != bytes_received - 8)
        return Response{incoming, Result::BAD_PAYLOAD_SIZE};
    incoming.payload = &packet_buffer[6];
    memcpy(&incoming.crc, &packet_buffer[6 + incoming.payload_size], 2);
    uint16_t crc = crc16_from_bytes(incoming.payload, incoming.payload_size);

    if (crc != incoming.crc)
        return Response{incoming, Result::BAD_CRC};

    return Response{incoming, Result::OK};

    // ACK would go here
}

Response printhead::send_and_receive(Packet packet, HardwareSerial& serial) {
    Response response;
    response.result = send(packet, serial);
    if (response.result != Result::OK) return response;
    return receive(serial);
}

Packet::HeaderBytes Packet::header_bytes() const
{
    static_assert(sizeof(ph_index) + sizeof(command) + sizeof(payload_size) == sizeof(HeaderBytes),
                  "Packet metadata sizing incorrect");
    HeaderBytes bytes;
    auto* ptr = bytes.data();

    memcpy(ptr, &ph_index, 2);
    memcpy(ptr + 2, &command, 2);
    memcpy(ptr + 4, &payload_size, 2);
    return bytes;
}

Packet::CrcBytes Packet::crc_bytes() const
{
    static_assert(sizeof(crc) == sizeof(CrcBytes), "Packet crc sizing incorrect");
    CrcBytes bytes;
    memcpy(bytes.data(), &crc, 2);
    return bytes;
}

Result Controller::set_temperature(Index index, float temperature)
{
    static constexpr size_t PAYLOAD_SIZE = sizeof(uint16_t);
    uint16_t converted_temp = static_cast<uint16_t>((temperature * 100.0f) + 30000.0f);
    uint8_t payload[PAYLOAD_SIZE]{};
    memcpy(payload, &converted_temp, PAYLOAD_SIZE);
    Packet request(index, Command::SET_TEMP, payload, PAYLOAD_SIZE);
    return send(request, bus);
}

Response Controller::get_temperature(Index index) {}

Response Controller::get_info(Index index)
{
    Packet packet(index, Command::GET_DEVICE_INFO);
    return send_and_receive(packet, bus); // TODO: parse response into type
}

Response Controller::get_fw_version(Index index)
{
    Packet packet(index, Command::GET_SW_VERSION);
    return send_and_receive(packet, bus); //TODO: parse result into relevant type
}

Result Controller::set_pid(Index index, float p, float i, float d)
{
    static constexpr uint8_t PAYLOAD_SIZE = 6;
    uint16_t p_ = static_cast<uint16_t>(p * 100);
    uint16_t i_ = static_cast<uint16_t>(i * 100);
    uint16_t d_ = static_cast<uint16_t>(d * 100);

    uint8_t payload[PAYLOAD_SIZE];
    memcpy(payload, &p_, 2);
    memcpy(payload + 2, &i_, 2);
    memcpy(payload + 4, &d_, 2);

    Packet packet(index, Command::SET_PID, payload, PAYLOAD_SIZE);
    return send(packet, bus);
}

Response Controller::get_pid(Index index)
{
    Packet packet(index, Command::GET_PID);
    return send_and_receive(packet, bus); // TODO: parse incoming response and return payload values
}

Result Controller::set_extrusion_speed(Index index, feedRate_t feedrate) {}
Response Controller::get_extrusion_speed(Index index) {
    Packet packet(index, Command::GET_EXTRUSION_SPEED);
    return send_and_receive(packet, bus);
}
Result Controller::set_extruder_stallguard_threshold(Index index, uint8_t threshold) {
    Packet packet(index, Command::SYRINGEPUMP_SET_ESTOP_THRESH, &threshold, 1);
    return send(packet, bus);
}
Response Controller::get_extruder_stallguard_threshold(Index index) {
    Packet packet(index, Command::SYRINGEPUMP_GET_ESTOP_THRESH);
    return send_and_receive(packet, bus);
}
Result Controller::set_extruder_microsteps(Index index, uint8_t microsteps) {
    Packet packet(index, Command::SET_MICROSTEP, &microsteps, 1);
    return send(packet, bus);
}
Response Controller::get_extruder_microsteps(Index index) {
    Packet packet(index, Command::GET_MICROSTEP);
    return send_and_receive(packet, bus);
}
Result Controller::set_extruder_rms_current(Index index, uint16_t mA) {}
Response Controller::get_extruder_rms_current(Index index) {}
Result Controller::set_extruder_hold_current(Index index, uint16_t mA) {}
Result Controller::get_extruder_hold_current(Index index) {}
Result Controller::home_extruder(Index index, ExtruderDirection direction) {
    Packet packet(index, Command::MOVE_TO_HOME_POSITION, &direction, 1);
    return send(packet, bus);
}
Result Controller::start_extruding(Index index) {
    Packet packet(index, Command::SYRINGEPUMP_START);
}
Result Controller::stop_extruding(Index index) {}

Result Controller::set_valve_speed(Index index, feedRate_t feedrate) {}
Response Controller::get_valve_speed(Index index) {}
Result Controller::set_valve_stallguard_threshold(Index index, uint8_t threshold) {}
Response Controller::get_valve_stallguard_threshold(Index index) {}
Result Controller::set_valve_microsteps(Index index, uint8_t microsteps) {}
Response Controller::get_valve_microsteps(Index index) {}
Result Controller::set_valve_rms_current(Index index, uint16_t mA) {}
Response Controller::get_valve_rms_current(Index index) {}
Result Controller::set_valve_hold_current(Index index, uint16_t mA) {}
Result Controller::move_slider_valve(Index index, uint16_t steps) {}