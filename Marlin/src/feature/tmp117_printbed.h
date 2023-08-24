//copyright cellink 2022 - GPLv3

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

BedSensors& bed_sensors();

class BedKalmanFilter
{
public:
    static const int indexST = 0;
    static const int indexOT = 1;

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