
#include "../MarlinCore.h"
#include "../gcode/gcode.h"
#include "../module/motion.h"
#include "../module/planner.h"

#include <array>
#include <numeric>

struct OpticalAutocal
{
    inline static constexpr xyz_pos_t START_POSITION{AUTOCAL_START_POSITION};
    inline static constexpr xyz_pos_t END_POSITION_PRINTBED_DELTA{AUTOCAL_PRINTBED_CENTER_DELTA};
    static constexpr uint8_t NUM_CYCLES = 2;
    inline static constexpr xy_pos_t XY_OFFSET_ERR{-1.0f, -1.0f};
    static constexpr float Z_OFFSET_ERR{-1.0f};
    static constexpr pin_t SENSOR_1{OPTICAL_SENSOR_1_PIN};
    static constexpr pin_t SENSOR_2{OPTICAL_SENSOR_2_PIN};
    using YSweepArray = std::array<float, NUM_CYCLES>;

    OpticalAutocal() = default;

    bool full_autocal_routine(const uint8_t tool, float feedrate);
    [[nodiscard]] bool is_calibrated(const uint8_t tool) const;
    [[nodiscard]] const xyz_pos_t &offset(const uint8_t tool) const;
    void reset(const uint8_t tool);
    void reset_all();

    xyz_pos_t tool_change_offset(const uint8_t);

private:
    static constexpr float SHORT_Y_RANGE = 12.0f;
    static constexpr float FULL_Y_RANGE = 24.0f;
    static constexpr float COARSE_Z_INCREMENT = 4.0f;
    static constexpr float MEDIUM_Z_INCREMENT = 1.0f;
    static constexpr float FINE_Z_INCREMENT = 0.125f;
    static constexpr float PRECISE_Z_INCREMENT = 0.025f;

    static uint32_t sensor_polarity;

    std::array<xyz_pos_t, EXTRUDERS> offsets;
    xyz_pos_t active_offset;

    /**
     * @brief Perform a multi-step sweep of optical sensors to find precise tool offset
     *
     * @param z_increment
     * @param feedrate mm/s
     * @param cycles
     */
    bool full_sensor_sweep(const uint8_t tool, const float feedrate);

    /**
     * @brief sweep in y direction across both sensors to derive nozzle centerpoint distance between beams
     *        then use that value to calculate the precise offset for both X and Y using sensor geometry
     *
     * @param feedrate mm/s
     * @return const xy_pos_t XY coordinate of the intersection of both optical sensors
     */
    [[nodiscard]] xy_pos_t find_xy_offset(const float feedrate) const;

    [[nodiscard]] float scan_for_tip(float z, const float inc, bool& condition, const float feedrate) const;

    [[nodiscard]] float find_z_offset(const float feedrate) const;
};

extern OpticalAutocal optical_autocal;
