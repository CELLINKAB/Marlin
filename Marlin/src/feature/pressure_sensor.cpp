// Copyright Cellink 2022 - GPLv3

#include "../inc/MarlinConfig.h"

#if ENABLED(ANALOG_PRESSURE_SENSOR)

#    if !ANY_PIN(PRESSURE_SENSOR)
#        error "PRESSURE_SENSOR_PIN must be defined for analog pressure sensor!"
#    endif

#    include "../gcode/gcode.h"
#    include "../module/planner.h"

#    include "interval_reporter.h"
#    include "pressure_sensor.h"

void GcodeSuite::M1111()
{
    static AnalogPressureSensor sensor_1(PRESSURE_SENSOR_PIN, 0.5302574309f);
    static AnalogPressureSensor sensor_2(PRESSURE_SENSOR_2_PIN, 0.4188586991f);
    static auto report_fn = []() {
        const float reading_1 = sensor_1.read_avg();
        const float e_pos = current_position.e;
        const auto time = millis();
        const float reading_2 = sensor_2.read_avg();
        SERIAL_ECHOLNPGM("T=", time, ",E=", e_pos, ",P1=", reading_1, ",P2=", reading_2);
    };
    static IntervalReporter pressure_sensor_reporter(report_fn);

    if (parser.seen('F')) {
        float factor = parser.value_float();
        uint8_t sensor_index = parser.byteval('I', 0);
        switch (sensor_index) {
        case 1:
            sensor_1.scalar = factor;
            break;
        case 2:
            sensor_2.scalar = factor;
            break;
        default:
            SERIAL_ECHO_MSG("Need to specify I1/2 (index) for factor change!");
            break;
        }
        return;
    }

    if (parser.seen_test('T'))
    {
        if (DEBUGGING(INFO))
            SERIAL_ECHO_MSG("Training pressure sensor...");
        sensor_1.tare();
        sensor_2.tare();
        if DEBUGGING (INFO) {
            SERIAL_ECHOLNPAIR_F("pressure sensor 1 new offset: ", sensor_1.offset);
            SERIAL_ECHOLNPAIR_F("pressure sensor 2 new offset: ", sensor_2.offset);
        }
        return;
    }

    if (parser.seen_test('R'))
        SERIAL_ECHO_MSG("Pressure (raw) sensor 1:",
                        sensor_1.read_raw(),
                        ", sensor 2:",
                        sensor_2.read_raw());
    else
        report_fn();

    if (int period = parser.intval('P'); period > 0)
        pressure_sensor_reporter.set_interval_ms(period);

    if (parser.boolval('S'))
        pressure_sensor_reporter.start();
    else
        pressure_sensor_reporter.stop();
}

#endif // ANALOG_PRESSURE_SENSOR