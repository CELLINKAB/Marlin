/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 Cellink [https://github.com/CELLINKAB/Marlin]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(TEMP_SENSOR_BED_IS_TMP117)

#    include "tmp117_printbed.h"

BedKalmanFilter bed_kalman_filter(25, 0);

BedSensors& bed_sensors()
{
    static BedSensors sensors{[]() {
        static SoftWire pb_i2c(PRINTBED_TEMP_SDA_PIN, PRINTBED_TEMP_SCL_PIN);
        static uint8_t tx_buf[4]{};
        static uint8_t rx_buf[4]{};
        pb_i2c.setTxBuffer(tx_buf, 4);
        pb_i2c.setRxBuffer(rx_buf, 4);
        pb_i2c.setClock(17'977); // prime number to avoid resonance
        pb_i2c.begin();
        TMP117<SoftWire> sensor_1(TMPAddr::SCL, pb_i2c);
        sensor_1.init(nullptr);
        TMP117<SoftWire> sensor_2(TMPAddr::SDA, pb_i2c);
        sensor_2.init(nullptr);
        TMP117<SoftWire> sensor_3(TMPAddr::VCC, pb_i2c);
        sensor_3.init(nullptr);
        TMP117<SoftWire> sensor_4(TMPAddr::GND, pb_i2c);
        sensor_4.init(nullptr);
        return std::array{sensor_1, sensor_2, sensor_3, sensor_4};
    }()};
    return sensors;
}

double get_tmp117_bed_temp()
{
    size_t failed_reads = 0;
    size_t temp_index = 0;
    double total_temps = 0.0;
    static double last_avg_temp = NAN;
    std::array<double, 4> individual_temps = {NAN, NAN, NAN, NAN};
    constexpr static float TEMP_TOLERANCE = 3.0f;
    auto is_average_acknowledged = true;
    for (auto& sensor : bed_sensors()) {
        const auto temperature = sensor.getTemperature();
        const bool is_temp_within_range = isnan(last_avg_temp)
                                          || WITHIN(temperature,
                                                    last_avg_temp - TEMP_TOLERANCE,
                                                    last_avg_temp + TEMP_TOLERANCE);
        if (!isnan(temperature) && is_temp_within_range) {
            total_temps += (temperature);
            if (isnan(last_avg_temp)) {// if the first average hasnt been calculated
                individual_temps[temp_index] = temperature;
                ++temp_index;
            }
        }
        else
            ++failed_reads;
    }
    bed_kalman_filter.predict();
    static unsigned retry_count = 0;
    if (failed_reads >= bed_sensors().size()) {
        ++retry_count;
        if (retry_count >= 5) {
            return -300.0; // it's been bad for too long, can no longer rely on kalman predictions
        }
    } else {
        retry_count = 0;
        const double avg = total_temps / (bed_sensors().size() - failed_reads);
        if (isnan(last_avg_temp)) {
            for (auto& individual_temp : individual_temps) {
                if (!isnan(individual_temp) && (std::abs(individual_temp - avg) > TEMP_TOLERANCE))
                    is_average_acknowledged = false;
            }
        }
        if (is_average_acknowledged)
            last_avg_temp = avg;
        bed_kalman_filter.update(avg, 0.01);
    }
    return bed_kalman_filter.surface_temp();
}

BedKalmanFilter::BedKalmanFilter(double initialSurfaceTemp, double initialOffsetTemp)
{
    m_mean(indexST) = initialSurfaceTemp;
    m_mean(indexOT) = initialOffsetTemp;

    m_cov.setIdentity();
}

void BedKalmanFilter::predict()
{
    Eigen::Matrix<double, NUM_BED_STATE_VARS, NUM_BED_STATE_VARS> F_matrix;
    F_matrix.setIdentity();
    F_matrix(indexOT, indexST) = 0.042444990665556376;
    F_matrix(indexOT, indexOT) = 0;

    Eigen::Matrix<double, NUM_BED_STATE_VARS, 1> aux;
    aux(indexST) = 0;
    aux(indexOT) = -0.9055907538332619;

    Eigen::Matrix<double, NUM_BED_STATE_VARS, NUM_BED_STATE_VARS> Q_matrix;
    Q_matrix.setIdentity();

    const auto newX = (F_matrix * m_mean) + aux;
    const auto newP = (F_matrix * m_cov * F_matrix.transpose()) + Q_matrix;

    m_cov = newP;
    m_mean = newX;
}

void BedKalmanFilter::update(double measValue, double measVariance)
{
    Eigen::Matrix<double, 1, 1> z{measValue};
    Eigen::Matrix<double, 1, 1> z_var{measVariance};
    Eigen::Matrix<double, 1, NUM_BED_STATE_VARS> H;
    H.setZero();
    H(0, indexST) = 1;
    H(0, indexOT) = 1;

    const auto y = z - (H * m_mean);
    const auto S = (H * m_cov * H.transpose()) + z_var;
    const auto K = m_cov * H.transpose() * S.inverse();

    const auto newX = m_mean + K * y;
    const auto newP = (Eigen::Matrix<double, NUM_BED_STATE_VARS, NUM_BED_STATE_VARS>::Identity()
                       - K * H)
                      * m_cov;

    m_cov = newP;
    m_mean = newX;
}

Eigen::Matrix<double, NUM_BED_STATE_VARS, NUM_BED_STATE_VARS> BedKalmanFilter::cov() const
{
    return m_cov;
}

Eigen::Matrix<double, NUM_BED_STATE_VARS, 1> BedKalmanFilter::mean() const
{
    return m_mean;
}

double BedKalmanFilter::surface_temp() const
{
    return m_mean(indexST);
}

double BedKalmanFilter::offset_temp() const
{
    return m_mean(indexOT);
}

#endif // TMP117_PRINTBED