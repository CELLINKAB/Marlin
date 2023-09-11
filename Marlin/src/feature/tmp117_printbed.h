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

#pragma once

// DO NOT REORDER
#include <array>
#include <eigen.h>
#include <Eigen/LU>
#include <SoftWire.h>
#include "tmp117/TMP117.h"

double get_tmp117_bed_temp();

constexpr size_t NUM_BED_TEMP_SENSORS = 4;
constexpr size_t NUM_BED_STATE_VARS = 2;

using BedSensors = std::array<TMP117<SoftWire>, NUM_BED_TEMP_SENSORS>;

/**
 * @brief lazily initialize and get access to bed sensors
 * 
 * @return BedSensors& 
 */
BedSensors& bed_sensors();

class BedKalmanFilter
{
public:
    static constexpr int indexST = 0;
    static constexpr int indexOT = 1;

    BedKalmanFilter(double initialSurfaceTemp, double initialOffsetTemp);
    void predict();
    void update(double measValue, double measVariance);
    Eigen::Matrix<double, NUM_BED_STATE_VARS, NUM_BED_STATE_VARS> cov() const;
    Eigen::Matrix<double, NUM_BED_STATE_VARS, 1> mean() const;
    double surface_temp() const;
    double offset_temp() const;

private:
    Eigen::Matrix<double, NUM_BED_STATE_VARS, 1> m_mean;
    Eigen::Matrix<double, NUM_BED_STATE_VARS, NUM_BED_STATE_VARS> m_cov;
};

extern BedKalmanFilter bed_kalman_filter;