#pragma once

#include "../../inc/MarlinConfigPre.h"
namespace pneumatics {

template<const pin_t SENSE>
struct AnalogPressureSensor
{
    AnalogPressureSensor(float scale_factor = 1.0f, float offset_kPa = 0.0f)
        : scalar(scale_factor)
        , offset(offset_kPa)
        , avg_raw(0)
    {
        pinMode(SENSE, INPUT_ANALOG);
    }

    float scalar;
    float offset;

    /**
     * @brief read the sensor and return raw ADC output
     * 
     * @return uint32_t 
     */
    inline uint32_t read_raw() const { return analogRead(SENSE); } // not portable

    inline float read_volts() const
    {
        return (ADC_VREF / static_cast<float>(1 << ADC_RESOLUTION)) * static_cast<float>(read_raw());
    }

    /**
     * @brief read the sensor and return scaled kPa reading
     * 
     * @return float 
     */
    inline float read(bool with_scaling = true, bool with_offset = true) const
    {
        return apply_scaling_leveling(static_cast<float>(read_raw()), with_scaling, with_offset);
    }

    constexpr float read_avg(bool with_scaling = true, bool with_offset = true) const
    {
        return apply_scaling_leveling(static_cast<float>(avg_raw), with_scaling, with_offset);
    }

    inline void tare() { offset = read_avg(true, false); }

    inline void update()
    {
        avg_raw = (avg_raw * OLD_AVERAGE_WEIGHT) + (read_raw() * NEW_VALUE_WEIGHT);
    }

private:
    constexpr static size_t WINDOW_SIZE = 20;
    constexpr static float NEW_VALUE_WEIGHT = 1.0 / static_cast<float>(WINDOW_SIZE);
    constexpr static float OLD_AVERAGE_WEIGHT = static_cast<float>(WINDOW_SIZE - 1)
                                                / static_cast<float>(WINDOW_SIZE);

    uint32_t avg_raw;

    constexpr float apply_scaling_leveling(float reading, bool with_scaling, bool with_offset) const
    {
        return (reading * (static_cast<float>(with_scaling) * scalar))
               - (static_cast<float>(with_offset) * offset);
    }
}; // AnalogPressureSensor

extern AnalogPressureSensor<GRIPPER_VACUUM_PIN> gripper_vacuum;
extern AnalogPressureSensor<PRESSURE_TANK_PIN> tank_pressure;
extern AnalogPressureSensor<PRESSURE_REGULATOR_SENSE_PIN> regulator_feedback;

} // namespace pneumatics