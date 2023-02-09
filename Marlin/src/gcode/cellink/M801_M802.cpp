// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ALL(TEMP_SENSOR_BED_IS_TMP117, CELLINK_REPORTING)
#    include "../../feature/tmp117/TMP117.h"
#    include "../../feature/tmp117_printbed.h"
#    include "../../module/temperature.h"
#    include "../gcode.h"

#    include <array>
#    include <numeric>

BedSensors& bed_sensors()
{
    static BedSensors sensors{[]() {
        static TwoWire pb_i2c(PRINTBED_TEMP_SDA_PIN, PRINTBED_TEMP_SCL_PIN);
        pb_i2c.setClock(50'000);
        pb_i2c.begin();
        TMP117<TwoWire> sensor_1(TMPAddr::GND, pb_i2c);
        sensor_1.init(nullptr);
        TMP117<TwoWire> sensor_2(TMPAddr::SCL, pb_i2c);
        sensor_2.init(nullptr);
        TMP117<TwoWire> sensor_3(TMPAddr::SDA, pb_i2c);
        sensor_3.init(nullptr);
        TMP117<TwoWire> sensor_4(TMPAddr::VCC, pb_i2c);
        sensor_4.init(nullptr);
        return std::array{sensor_1, sensor_2, sensor_3, sensor_4};
    }()};
    return sensors;
}

double get_tmp117_bed_temp()
{
    double total_temps = 0.0;
    size_t failed_reads = 0;
    for (auto& sensor : bed_sensors()) {
        const auto temperature = sensor.getTemperature();
        if (!isnan(temperature))
            total_temps += (temperature);
        else
            ++failed_reads;
    }
    if (failed_reads >= bed_sensors().size())
        return -300.0;
    const double avg = total_temps / (bed_sensors().size() - failed_reads);
    return avg;
}

// get bed temp
void GcodeSuite::M802()
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