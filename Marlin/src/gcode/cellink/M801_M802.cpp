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
        static SoftWire pb_i2c(PRINTBED_TEMP_SDA_PIN, PRINTBED_TEMP_SCL_PIN);
        static uint8_t tx_buf[4]{};
        static uint8_t rx_buf[4]{};
        pb_i2c.setTxBuffer(tx_buf, 4);
        pb_i2c.setRxBuffer(rx_buf, 4);
        pb_i2c.setClock(10'000);
        pb_i2c.begin();
        TMP117<SoftWire> sensor_1(TMPAddr::GND, pb_i2c);
        sensor_1.init(nullptr);
        TMP117<SoftWire> sensor_2(TMPAddr::SCL, pb_i2c);
        sensor_2.init(nullptr);
        TMP117<SoftWire> sensor_3(TMPAddr::SDA, pb_i2c);
        sensor_3.init(nullptr);
        TMP117<SoftWire> sensor_4(TMPAddr::VCC, pb_i2c);
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
    static unsigned retry_count = 0;
    if (failed_reads >= bed_sensors().size()) {
        if (retry_count < 3) {
            // i2c_hardware_reset(pb_i2c);
            ++retry_count;
            return get_tmp117_bed_temp();
        } else {
            return -300.0;
        }
    }
    retry_count = 0;
    const double avg = total_temps / (bed_sensors().size() - failed_reads);
    return avg;
}

void report_bed_sensors() {
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

#if ENABLED(AUTO_REPORT_BED_MULTI_SENSOR)
    void BedMultiSensorReporter::report() {
        report_bed_sensors();
    }
    BedMultiSensorReporter bed_multi_sensor_reporter;
#endif

// get bed temp
void GcodeSuite::M802()
{
    report_bed_sensors();
    #if ENABLED(AUTO_REPORT_BED_MULTI_SENSOR)
        bed_multi_sensor_reporter.set_interval(parser.byteval('S'));
    #endif
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