
#include "../MarlinCore.h"
#include "../gcode/gcode.h"
#include "../module/motion.h"
#include "../module/planner.h"

#include <array>
#include <numeric>

struct OpticalAutocal
{
    inline static constexpr xyz_pos_t END_POSITION_PRINTBED_DELTA{AUTOCAL_PRINTBED_CENTER_DELTA};
    static constexpr uint8_t NUM_CYCLES = 2;
    inline static constexpr xy_pos_t XY_OFFSET_ERR{-1.0f, -1.0f};
    static constexpr float Z_OFFSET_ERR{-1.0f};
    static constexpr pin_t SENSOR_1{OPTICAL_SENSOR_1_PIN};
    static constexpr pin_t SENSOR_2{OPTICAL_SENSOR_2_PIN};

    static xyz_pos_t nozzle_calibration_extra_offset;

    enum class ErrorCode {
        OK,
        POLARITY_MISMATCH,
        CALIBRATION_FAILED,
        NO_NOZZLE_DETECTED,
        SANITY_CHECK_FAILED,
    };

    OpticalAutocal() = default;

    auto full_autocal_routine(const xyz_pos_t start_pos, const feedRate_t feedrate) -> ErrorCode;
    [[nodiscard]] bool is_calibrated(const uint8_t tool) const;
    [[nodiscard]] const xyz_pos_t& offset(const uint8_t tool) const;
    void report_sensors() const;
    void reset(const uint8_t tool);
    void reset_all();
    void test(uint16_t cycles, xyz_pos_t start_pos, feedRate_t feedrate);

    xyz_pos_t tool_change_offset(const uint8_t);

private:
    static constexpr float SHORT_Y_RANGE = 12.0f;
    static constexpr float FULL_Y_RANGE = 24.0f;
    static constexpr float COARSE_Z_INCREMENT = 3.0f;
    static constexpr float MEDIUM_Z_INCREMENT = 0.75f;
    static constexpr float FINE_Z_INCREMENT = 0.125f;
    static constexpr float PRECISE_Z_INCREMENT = 0.025f;

    struct LongSweepCoords
    {
        float sensor_1_forward_y;
        float sensor_2_forward_y;
        float sensor_2_backward_y;
        float sensor_1_backward_y;
        inline constexpr float y1() const
        {
            return (sensor_1_forward_y + sensor_1_backward_y) / 2.0f;
        }
        inline constexpr float y2() const
        {
            return ((sensor_2_backward_y + sensor_1_backward_y) / 2.0f);
        }
        inline constexpr float y_delta() const { return ABS(y1() - y2()); }
        inline constexpr bool has_zeroes() const
        {
            return sensor_1_forward_y == 0.0f || sensor_2_forward_y == 0.0f
                   || sensor_2_backward_y == 0.0f || sensor_1_backward_y == 0.0f;
        }
        void print()
        {
            SERIAL_ECHOLNPGM(" y1: ",
                             retval.sensor_1_forward_y,
                             " y2: ",
                             retval.sensor_2_forward_y,
                             " y3: ",
                             retval.sensor_2_backward_y,
                             " y4: ",
                             retval.sensor_1_backward_y);
        }
    };

    static uint32_t sensor_polarity;

    std::array<xyz_pos_t, EXTRUDERS> offsets;
    xyz_pos_t active_offset;

    [[nodiscard]] auto long_sweep(feedRate_t feedrate_mm_s) -> LongSweepCoords;

    /**
     * @brief Perform a multi-step sweep of optical sensors to find precise tool offset
     *
     * @param z_increment
     * @param feedrate mm/s
     * @param cycles
     */
    [[nodiscard]] auto full_sensor_sweep(const uint8_t tool,
                                         const xyz_pos_t start_pos,
                                         const feedRate_t feedrate_mm_s) -> ErrorCode;

    /**
     * @brief sweep in y direction across both sensors to derive nozzle centerpoint distance between beams
     *        then use that value to calculate the precise offset for both X and Y using sensor geometry
     *
     * @param feedrate mm/s
     * @return const xy_pos_t XY coordinate of the intersection of both optical sensors
     */
    [[nodiscard]] xy_pos_t find_xy_offset(const xy_pos_t start_pos, const feedRate_t feedrate) const;

    [[nodiscard]] float scan_for_tip(float z,
                                     const float inc,
                                     bool& condition,
                                     const feedRate_t feedrate) const;

    [[nodiscard]] float find_z_offset(float z, const feedRate_t feedrate) const;
};

extern OpticalAutocal optical_autocal;
//