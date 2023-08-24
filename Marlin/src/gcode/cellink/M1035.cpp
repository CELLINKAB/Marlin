// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ALL(CELLINK_REPORTING, HAS_HEATED_BED, TEMP_SENSOR_BED_IS_TMP117)
#    include "../../feature/cellink_reporter.h"
#    include "../../feature/tmp117/TMP117.h"
#    include "../../feature/tmp117_printbed.h"
#    include "../../module/temperature.h"
#    include "../gcode.h"

#    include <numeric>

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

#endif