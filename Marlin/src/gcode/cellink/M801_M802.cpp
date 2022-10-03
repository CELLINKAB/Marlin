// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ALL(TEMP_SENSOR_BED_IS_TMP117, CELLINK_REPORTING)
#    include "../gcode.h"
#    include "../../feature/tmp117/TMP117.h"

#    include <array>
#    include <numeric>

#    include "../../feature/tmp117_printbed.h"

static std::array<TMP117<TwoWire>, 4>& bed_sensors()
{
    static std::array<TMP117<TwoWire>, 4> sensors{[]() {
        static TwoWire pb_i2c(PRINTBED_TEMP_SDA_PIN, PRINTBED_TEMP_SCL_PIN);
        pb_i2c.begin();
        TMP117 sensor_1(TMPAddr::GND, pb_i2c);
        sensor_1.init(nullptr);
        TMP117 sensor_2(TMPAddr::SCL, pb_i2c);
        sensor_2.init(nullptr);
        TMP117 sensor_3(TMPAddr::SDA, pb_i2c);
        sensor_3.init(nullptr);
        TMP117 sensor_4(TMPAddr::VCC, pb_i2c);
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

// get bed temp
void GcodeSuite::M802()
{
    uint8_t sensor_num = 0;
    for (auto& sensor : bed_sensors()) {
        SERIAL_ECHO("PBT");
        SERIAL_ECHO(sensor_num++);
        SERIAL_CHAR(':');
        SERIAL_ECHO_F(sensor.getTemperature());
        SERIAL_CHAR(',');
    }
    SERIAL_EOL();
}

void GcodeSuite::M801() {}

#endif // CELLINK_REPORTING