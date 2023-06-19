
#include "tmp117_printbed.h"

#include "../inc/MarlinConfig.h"

BedKalmanFilter bed_kalman_filter(25, 0);

BedSensors& bed_sensors()
{
    static BedSensors sensors{[]() {
        static SoftWire pb_i2c(PRINTBED_TEMP_SDA_PIN, PRINTBED_TEMP_SCL_PIN);
        static uint8_t tx_buf[4]{};
        static uint8_t rx_buf[4]{};
        pb_i2c.setTxBuffer(tx_buf, 4);
        pb_i2c.setRxBuffer(rx_buf, 4);
        pb_i2c.setClock(20'000);
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
    static double last_temp;
    for (auto& sensor : bed_sensors()) {
        const auto temperature = sensor.getTemperature();
        if (!isnan(temperature))
            total_temps += (temperature);
        else
            ++failed_reads;
    }
    static unsigned retry_count = 0;
    if (failed_reads >= bed_sensors().size()) {
        if (retry_count < 3 && last_temp != 0.0) {
            // i2c_hardware_reset(pb_i2c);
            ++retry_count;
            return last_temp;
        } else {
            return -300.0;
        }
    }
    retry_count = 0;
    const double avg = total_temps / (bed_sensors().size() - failed_reads);
    bed_kalman_filter.predict();
    bed_kalman_filter.update(avg, 0.01);
    last_temp = bed_kalman_filter.surface_temp();
    return last_temp;
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