
#include "../inc/MarlinConfig.h"
#include "tmp117_printbed.h"

BedKalmanFilter bed_kalman_filter(25, 0);

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
    Eigen::Matrix<double, 1, 1> z {measValue};
    Eigen::Matrix<double, 1, 1>  z_var {measVariance};
    Eigen::Matrix<double, 1, NUM_BED_STATE_VARS>  H;
    H.setZero();
    H(0, indexST) = 1;
    H(0, indexOT) = 1;

    const auto y = z - (H * m_mean);
    const auto  S = (H * m_cov * H.transpose()) + z_var;
    const auto  K = m_cov * H.transpose() * S.inverse();

    const auto newX = m_mean + K * y;
    const auto newP = (Eigen::Matrix<double, NUM_BED_STATE_VARS, NUM_BED_STATE_VARS>::Identity() - K * H) * m_cov;

    m_cov = newP;
    m_mean = newX;
}

Eigen::Matrix<double, NUM_BED_STATE_VARS, NUM_BED_STATE_VARS>  BedKalmanFilter::cov() const
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