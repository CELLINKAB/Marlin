#pragma once

#include "../../inc/MarlinConfig.h"

namespace pneumatics {

template<const pin_t REG>
struct Regulator
{
    inline void set_point(float kPa)
    {
        uint32_t value = static_cast<uint32_t>(kPa * DAC_FACTOR);
        analogWrite(REG, value);
        pressure = kPa;
    }
    inline float set_point() const { return pressure; }

private:
    // 200kPa regulator 5V analog input clipped to 3.3v, 12 bit DAC
    static constexpr float DAC_FACTOR = 4096.0 / ((3.3 / 5.0) * 200.0);

    float pressure;
};

extern Regulator<PRESSURE_REGULATOR_PIN> regulator;

} // namespace pneumatics