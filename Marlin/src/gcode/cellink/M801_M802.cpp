// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ALL(TEMP_SENSOR_BED_IS_TMP117, CELLINK_REPORTING)
#    include "../../feature/tmp117/TMP117.h"
#    include "../../feature/tmp117_printbed.h"
#    include "../../module/temperature.h"
#    include "../gcode.h"

#    include <numeric>

void report_bed_sensors(bool all_sensors)
{
    if (!all_sensors) {
        SERIAL_ECHOLN("PBT:",Temperature::degBed())
    }
    uint8_t sensor_num = 0;
    for (auto& sensor : bed_sensors()) {
        const auto temperature = sensor.getTemperature();
        SERIAL_ECHO("PBT");
        SERIAL_ECHO(sensor_num++);
        SERIAL_CHAR(':');
        if (isnan(temperature))
            SERIAL_ECHO("NAN");
        else
            SERIAL_ECHO_F(temperature);
        SERIAL_CHAR(',');
    }
    SERIAL_EOL();
}

#    if ENABLED(AUTO_REPORT_BED_MULTI_SENSOR)
void BedMultiSensorReporter::report()
{
    report_bed_sensors(all_sensors);
}
BedMultiSensorReporter bed_multi_sensor_reporter;
#    endif

// get bed temp
void GcodeSuite::M802()
{
    const bool all_sensors = parser.seen_test('D');
    report_bed_sensors(all_sensors);
#    if ENABLED(AUTO_REPORT_BED_MULTI_SENSOR)
    bed_multi_sensor_reporter.all_sensors = all_sensors;
    bed_multi_sensor_reporter.set_interval(parser.byteval('S'));
#    endif
}

void GcodeSuite::M801()
{
#    if ENABLED(MYCO_HEATER_DEBUG)
    if (parser.seen('D')) {
        const bool debugging = parser.value_bool();
        Temperature::temp_bed.is_set = !debugging;
        if (!debugging) {
            Temperature::temp_bed.soft_pwm_amount = 0;
            WRITE_HEATER_BED(0);
            return;
        }

        const auto pwm_val = constrain(parser.intval('P', 0), -255, 255);
        const auto frequency = parser.ulongval('F', 10'000);

        analogWriteFrequency(frequency);
        Temperature::temp_bed.soft_pwm_amount = pwm_val;
        WRITE_HEATER_BED(pwm_val);
        return;
    }
#    endif
    M140();
}

#endif // CELLINK_REPORTING