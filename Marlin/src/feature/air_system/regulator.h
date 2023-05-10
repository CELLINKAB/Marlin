#pragma once

#include "../../inc/MarlinConfig.h"

namespace pneumatics {

template<const pin_t REG>
struct Regulator
{
    inline void set_point(float kPa)
    {
        kPa = constrain(kPa, 0.0, REG_MAX);
        uint32_t value = static_cast<uint32_t>(kPa * DAC_FACTOR);
        analogWrite(REG, value);
        pressure = kPa;
    }
    inline float set_point() const { return pressure; }

private:
    static constexpr float REG_MAX = (3.3 / 5.0) * 100.0;
    static constexpr float DAC_FACTOR = 256.0 / REG_MAX;

    float pressure;
};

extern Regulator<PRESSURE_REGULATOR_PIN> regulator;

} // namespace pneumatics
