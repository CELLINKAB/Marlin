
#pragma once

#include "../inc/MarlinConfigPre.h"

#include <numeric>
#include <atomic>

#    ifndef PRESSURE_VALVE_CLOSE_LEVEL
#        define PRESSURE_VALVE_CLOSE_LEVEL LOW
#    endif
#    ifndef PRESSURE_VALVE_OPEN_LEVEL
#        define PRESSURE_VALVE_OPEN_LEVEL !PRESSURE_VALVE_CLOSE_LEVEL
#    endif

namespace pneumatics {

void init();

void set_regulator_pressure(float kPa);
float get_regulator_set_pressure();
void pressurize_tank(millis_t timeout_after_ms = 10'000);

void gripper_release();

void apply_mixing_pressure(uint8_t tool);
void release_mixing_pressure(uint8_t tool);

// instance counter that enables pressure whenever something is using it
struct [[maybe_unused]] PressureToken
{
    ~PressureToken();
    PressureToken(const PressureToken& token) = delete;
    PressureToken(PressureToken&& token) = delete;
    PressureToken& operator=(const PressureToken& token) = delete;
    PressureToken& operator=(PressureToken&& token) = delete;
    inline static bool has_users() { return users.load() > 0; }

private:
    PressureToken() { ++users; }
    inline static std::atomic_int users{0};
    inline static void pressure_valves(bool enable)
    {
        WRITE(PRESSURE_PUMP_EN_PIN, enable ? HIGH : LOW);
        WRITE(PRESSURE_VALVE_PUMP_OUT_PIN,
              enable ? PRESSURE_VALVE_OPEN_LEVEL : PRESSURE_VALVE_CLOSE_LEVEL);
    }
    friend PressureToken use_pressure();
    friend void update_tank();
};

[[nodiscard]] PressureToken use_pressure();
void update_tank();

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

    float read_avg(bool with_scaling = true, bool with_offset = true) const;

    inline void tare() { offset = read_avg(true, false); }

private:
    pin_t pin;

    float apply_scaling_leveling(float reading, bool with_scaling, bool with_offset) const;
}; // AnalogPressureSensor

extern AnalogPressureSensor gripper_vacuum;
extern AnalogPressureSensor tank_pressure;
extern AnalogPressureSensor regulator_feedback;

} // namespace pneumatics