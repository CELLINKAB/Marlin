// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ALL(CELLINK_REPORTING, HAS_HEATED_BED)
#    include "../../feature/cellink_reporter.h"
#    include "../../module/temperature.h"
#    include "../gcode.h"

#    include <numeric>

#    if ENABLED(TEMP_SENSOR_BED_IS_TMP117)
#        include "../../feature/tmp117/TMP117.h"
#        include "../../feature/tmp117_printbed.h"
void report_bed_sensors()
{
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
#    endif

void cellink::Reporter::M802::report()
{
    cellink::serial_echoln_kv("PBT", Temperature::degBed());
}

void GcodeSuite::M800()
{
    thermalManager.setTargetBed(0);
    thermalManager.temp_bed.is_set = false;
    thermalManager.temp_bed.soft_pwm_amount = 0;
#    ifdef BED_FAN_INDEX
    thermalManager.set_fan_speed(BED_FAN_INDEX, 0);
#    endif
    WRITE_HEATER_BED(LOW);
}

// get bed temp
void GcodeSuite::M802()
{
    TERN_(TEMP_SENSOR_BED_IS_TMP117, if (parser.boolval('D')) report_bed_sensors());
    cellink::reporter.m802.set_interval(parser.byteval('S'));
    cellink::reporter.m802.report();
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
        if (parser.seenval('F'))
            analogWriteFrequency(parser.value_ulong());

        Temperature::temp_bed.soft_pwm_amount = pwm_val;
        WRITE_HEATER_BED(pwm_val);
        return;
    }
#    endif
    M140();
}






#endif // CELLINK_REPORTING