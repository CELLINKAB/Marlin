// Copyright Cellink 2022 - GPLv3

#include "../inc/MarlinConfig.h"

#if ANY_PIN(PRESSURE_SENSOR)

#include "../gcode/gcode.h"
#include "interval_reporter.h"

void GcodeSuite::M1111()
{
    static auto report_fn = []{ SERIAL_ECHO_MSG("Pressure:", analogRead(PRESSURE_SENSOR_PIN)); };
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