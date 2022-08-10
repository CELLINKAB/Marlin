

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

Result Controller::set_temp(Index index, float temperature)
{
    static constexpr size_t PAYLOAD_SIZE = sizeof(uint16_t);
    uint16_t converted_temp = static_cast<uint16_t>((temperature * 100.0f) + 30000.0f);
    uint8_t payload[PAYLOAD_SIZE]{};
    memcpy(payload, &converted_temp, PAYLOAD_SIZE);
    Packet request(index, Command::SET_TEMP, payload, PAYLOAD_SIZE);
    return send(request, bus);
}

Response Controller::get_info(Index index)
{
    Packet packet(index, Command::GET_DEVICE_INFO);
    Response response;
    if ((response.result = send(packet, bus)) != Result::OK)
        return response;
    return receive(bus); // TODO: parse response into type
}

Response Controller::get_fw_version(Index index)
{
    Packet packet(index, Command::GET_SW_VERSION);
    Response response;
    if ((response.result = send(packet, bus)) != Result::OK)
        return response;
    return receive(bus); //TODO: parse result into relevant type
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
    Response response;
    if ((response.result = send(packet, bus)) != Result::OK)
        return response;
    return receive(bus); // TODO: parse incoming response and return payload values
}