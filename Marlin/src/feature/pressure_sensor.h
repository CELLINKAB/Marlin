// Copyright Cellink 2022 - GPLv3

#include "../inc/MarlinConfigPre.h"

#include <numeric>

struct AnalogPressureSensor
{
    AnalogPressureSensor(pin_t sense_pin, float scale_factor = 1.0f, float offset_kPa = 0.0f)
        : scalar(scale_factor)
        , offset(offset_kPa)
        , pin(sense_pin)
    {
        pinMode(sense_pin, INPUT_ANALOG);
    }

    float scalar;
    float offset;

    /**
     * @brief read the sensor and return raw ADC output
     * 
     * @return uint32_t 
     */
    uint32_t read_raw() const { return analogRead(pin); } // not portable

    /**
     * @brief read the sensor and return scaled kPa reading
     * 
     * @return float 
     */
    float read(bool with_scaling = true, bool with_offset = true) const
    {
        return apply_scaling_leveling(static_cast<float>(read_raw()), with_scaling, with_offset);
    }

    float read_avg(bool with_scaling = true, bool with_offset = true) const
    {
        uint32_t samples[40]{};
        for (uint32_t& sample : samples) {
            sample = read_raw();
            delay(2);
        }
        float avg_raw = std::accumulate(std::cbegin(samples), std::cend(samples), 0.0f) / 40.0f;
        return apply_scaling_leveling(avg_raw, with_scaling, with_offset);
    }

    void tare() { offset = read_avg(true, false); }

private:
    pin_t pin;

    float apply_scaling_leveling(float reading, bool with_scaling, bool with_offset) const
    {
        return (reading * (static_cast<float>(with_scaling) * scalar))
               - (static_cast<float>(with_offset) * offset);
    }
};
