// Copyright Cellink 2022 - GPLv3

#include "../inc/MarlinConfig.h"

#if ENABLED(TEMP_SENSOR_BED_IS_TMP117)
#    include "../gcode/gcode.h"
#    include "tmp117/TMP117.h"

#    include <array>
#    include <numeric>

#    include "tmp117_printbed.h"

static std::array<TMP117<TwoWire>, 4>& bed_sensors()
{
    static std::array<TMP117<TwoWire>, 4> sensors{[]() {
        TMP117 sensor_1(TMPAddr::GND);
        sensor_1.init(nullptr);
        TMP117 sensor_2(TMPAddr::SCL);
        sensor_2.init(nullptr);
        TMP117 sensor_3(TMPAddr::SDA);
        sensor_3.init(nullptr);
        TMP117 sensor_4(TMPAddr::VCC);
        sensor_4.init(nullptr);
        return std::array{sensor_1, sensor_2, sensor_3, sensor_4};
    }()};
    return sensors;
}

double get_tmp117_bed_temp()
{
    double total_temps = 0.0;
    for (auto& sensor : bed_sensors()) {
        total_temps += sensor.getTemperature();
    }
    const double avg = total_temps / bed_sensors().size();
    return avg;
}

void GcodeSuite::M802()
{
    uint8_t sensor_num = 0;
    for (auto& sensor : bed_sensors()) {
        SERIAL_ECHO("bed_temperature_");
        SERIAL_ECHO(sensor_num++);
        SERIAL_CHAR(':');
        SERIAL_ECHO_F(sensor.getTemperature());
        SERIAL_CHAR(',');
    }
    SERIAL_EOL();
}

#endif // TEMP_SENSOR_BED_IS_TMP117