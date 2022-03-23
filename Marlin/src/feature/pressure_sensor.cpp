// Copyright Cellink 2022 - GPLv3

#include "../inc/MarlinConfig.h"

#if ANY_PIN(PRESSURE_SENSOR)

#include "../gcode/gcode.h"
#include "interval_reporter.h"

#include <numeric>

void GcodeSuite::M1111()
{
    static uint32_t pressure_zero_offset = 165;
    static constexpr float KPA_SCALE_FACTOR = 4.1792;
    static auto report_fn = []()
    {
        static constexpr size_t SAMPLES = 3;
        uint32_t sensor_readings[SAMPLES]{};
        for (auto& val : sensor_readings) {
            val = analogRead(PRESSURE_SENSOR_PIN);
            delay(1);
        }
        auto avg_reading = std::accumulate(std::cbegin(sensor_readings), std::cend(sensor_readings), 0) / SAMPLES;
        auto shifted_reading = (avg_reading > pressure_zero_offset) ? (avg_reading - pressure_zero_offset) : 0;
        auto kpa_reading = static_cast<float>(shifted_reading) * KPA_SCALE_FACTOR;
        SERIAL_ECHO_MSG("Pressure (kPa):", kpa_reading);
    };
    static IntervalReporter pressure_sensor(report_fn);

    if (parser.seenval('T'))
        pressure_zero_offset = parser.ulongval('T');
    else if (parser.seen_test('T'){
        if (DEBUGGING(INFO))
            SERIAL_ECHO_MSG("Training pressure sensor...");
        static constexpr size_t SAMPLES = 40;
        uint32_t sensor_values[SAMPLES]{};
        for (auto &val : sensor_values)
        {
            val = analogRead(PRESSURE_SENSOR_PIN);
            delay(1000 / SAMPLES);
        }
        pressure_zero_offset = std::accumulate(std::cbegin(sensor_values), std::cend(sensor_values), 0) / SAMPLES;
        if DEBUGGING (INFO)
            SERIAL_ECHOLNPGM("new sensor offset: ", pressure_zero_offset);
        return;
    }

    if (parser.seen_test('R'))
        SERIAL_ECHO_MSG("Pressure (raw):", analogRead(PRESSURE_SENSOR_PIN));
    else
        report_fn();

    if (int period = parser.intval('P'); period > 0)
        pressure_sensor.set_interval_ms(period);

    if (parser.boolval('S'))
        pressure_sensor.start();
    else
        pressure_sensor.stop();
}

#endif