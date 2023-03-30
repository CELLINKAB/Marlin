#pragma once

#include "../../inc/MarlinConfigPre.h"
namespace pneumatics 
{

struct AnalogPressureSensor
{
    AnalogPressureSensor(pin_t sense_pin, float scale_factor = 1.0f, float offset_kPa = 0.0f);

    float scalar;
    float offset;

    /**
     * @brief read the sensor and return raw ADC output
     * 
     * @return uint32_t 
     */
    inline uint32_t read_raw() const { return analogRead(pin); } // not portable

    inline float read_volts() const
    {
        return (ADC_VREF / static_cast<float>(1 << ADC_RESOLUTION))
               * static_cast<float>(analogRead(pin));
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
        avg_raw = avg_raw - (avg_raw / WINDOW_SIZE) + (read_raw() / WINDOW_SIZE);
    }

private:
    constexpr static size_t WINDOW_SIZE = 10;

    pin_t pin;
    uint32_t avg_raw;

    constexpr float apply_scaling_leveling(float reading, bool with_scaling, bool with_offset) const
    {
        return (reading * (static_cast<float>(with_scaling) * scalar))
               - (static_cast<float>(with_offset) * offset);
    }
}; // AnalogPressureSensor

extern AnalogPressureSensor gripper_vacuum;
extern AnalogPressureSensor tank_pressure;
extern AnalogPressureSensor regulator_feedback;

} // namespace pneumatics