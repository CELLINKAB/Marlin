// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ALL(TEMP_SENSOR_BED_IS_TMP117, CELLINK_REPORTING)
#    include "../../feature/cellink_reporter.h"
#    include "../../feature/tmp117/TMP117.h"
#    include "../../feature/tmp117_printbed.h"
#    include "../../module/temperature.h"
#    include "../gcode.h"

#    include <numeric>

void report_bed_sensors(bool all_sensors)
{
    if (!all_sensors) {
        SERIAL_ECHOLNPGM("PBT:", Temperature::degBed());
        return;
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

bool cellink::Reporter::M802::all_sensors = false;
void cellink::Reporter::M802::report()
{
    report_bed_sensors(all_sensors);
}

#    if ENABLED(AUTO_REPORT_BED_MULTI_SENSOR)
bool BedMultiSensorReporter::all_sensors = false;
void BedMultiSensorReporter::report()
{
    report_bed_sensors(all_sensors);
}
BedMultiSensorReporter bed_multi_sensor_reporter;
#    endif

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
    cellink::reporter.m802.set_interval(parser.byteval('S'));
    cellink::reporter.m802.all_sensors = parser.seen_test('D');
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

void GcodeSuite::M772()
{
    cellink::reporter.m772.set_interval(parser.byteval('S'));
    cellink::reporter.m772.report();
}

//SetBedThermisterParams
void GcodeSuite::M1035()
{
    int16_t sensor_index = parser.intval('I', -1);
    float gain = parser.floatval('S', 1.0f);
    float offset = parser.floatval('O');

    auto handle_sensor = [=](auto& sensor) {
        if (offset == 0.0f && gain == 1.0f)
            SERIAL_ECHOLNPGM("SENSOR_ID:",
                             sensor.getDeviceID(),
                             ",OFFSET:",
                             sensor.getOffsetTemperature(),
                             ",GAIN:",
                             sensor.getGain());
        if (offset != 0.0f)
            sensor.setOffsetTemperature(offset);
        if (gain != 1.0f)
            sensor.setGain(gain);
    };
    if (sensor_index == -1) {
        auto& sensors = bed_sensors();
        for (auto& sensor : sensors) {
            handle_sensor(sensor);
        }
    } else {
        auto& sensor
            = bed_sensors()[constrain(sensor_index, 0, static_cast<int16_t>(bed_sensors().size() - 1))];
        handle_sensor(sensor);
    }
}
void GcodeSuite::M1035_report(bool for_replay)
{
    report_heading_etc(for_replay, F("Bed TMP117 Sensors"));
    size_t sensor_num = 0;
    for (auto& sensor : bed_sensors()) {
        SERIAL_ECHOLNPGM("M1035 I",
                         sensor_num++,
                         " O",
                         sensor.getOffsetTemperature(),
                         " S",
                         sensor.getGain());
    }
}

#endif // CELLINK_REPORTING