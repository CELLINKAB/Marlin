// Copyright Cellink 2022 - GPLv3

#include "../inc/MarlinConfig.h"

#if ANY_PIN(PRESSURE_SENSOR)

#include "../gcode/gcode.h"
#include "interval_reporter.h"

void GcodeSuite::M1111()
{
    static constexpr float KPA_SCALE_FACTOR = 4.1792;
    static constexpr float KPA_ZERO_OFFSET = -689.11;
    static constexpr auto pressure_scale = [](uint32_t analog_reading)
    { return (static_cast<float>(analog_reading) * KPA_SCALE_FACTOR) - KPA_ZERO_OFFSET; };
    static auto report_fn = []
    { SERIAL_ECHO_MSG("Pressure (kPa):", pressure_scale(analogRead(PRESSURE_SENSOR_PIN))); };
    static IntervalReporter pressure_sensor(report_fn);

    report_fn();

    if (int period = parser.intval('P'); period > 0)
        pressure_sensor.set_interval_ms(period);

    if (parser.boolval('S'))
        pressure_sensor.start();
    else
        pressure_sensor.stop();
}

#endif