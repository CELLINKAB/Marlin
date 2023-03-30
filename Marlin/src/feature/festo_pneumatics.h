
#pragma once

#include "../inc/MarlinConfigPre.h"

#include <atomic>
#include <numeric>

#ifndef PRESSURE_VALVE_CLOSE_LEVEL
#    define PRESSURE_VALVE_CLOSE_LEVEL LOW
#endif
#ifndef PRESSURE_VALVE_OPEN_LEVEL
#    define PRESSURE_VALVE_OPEN_LEVEL !PRESSURE_VALVE_CLOSE_LEVEL
#endif

namespace pneumatics {

void init();

void update();

void set_regulator_pressure(float kPa);
float get_regulator_set_pressure();

enum class GripperState {
    Close,
    Release,
    Grip,
};

void set_gripper_valves(GripperState state);

void apply_mixing_pressure(uint8_t tool);
void release_mixing_pressure(uint8_t tool);

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

// instance counter that enables pressure whenever something is using it
template<const pin_t PUMP, const pin_t VALVE>
struct Pump
{
    void init()
    {
        OUT_WRITE(PRESSURE_VALVE_PUMP_OUT_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
        OUT_WRITE(PRESSURE_PUMP_EN_PIN, LOW);
    }

    void update()
    {
        if (suction_users > 0) {
            WRITE(VALVE, PRESSURE_VALVE_CLOSE_LEVEL);
            WRITE(PUMP, HIGH);
            return;
        }
        const float current_pressure = tank_pressure.read_avg();
        if (current_pressure < TANK_PRESSURE_MAINTAINENCE
            || (pressure_users > 0 && current_pressure < TANK_PRESSURE_TARGET)) {
            WRITE(VALVE, PRESSURE_VALVE_OPEN_LEVEL);
            WRITE(PUMP, HIGH);
        } else {
            WRITE(VALVE, PRESSURE_VALVE_CLOSE_LEVEL);
            WRITE(PUMP, LOW);
        }
    }

private:
    static constexpr float TANK_PRESSURE_TARGET = 75.0f;
    static constexpr float TANK_PRESSURE_MAINTAINENCE = 50.0f;
    static constexpr float TANK_PRESSURE_MAX = 100.0f;

    size_t suction_users;
    size_t pressure_users;

    enum class LockType {
        Suction,
        Pressure,
    };

    void add_user(LockType kind)
    {
        switch (kind) {
        case LockType::Suction:
            ++suction_users;
            break;
        case LockType::Pressure:
            ++pressure_users;
            break;
        }
        update();
    }

    void remove_user(LockType kind)
    {
        switch (kind) {
        case LockType::Suction:
            --suction_users;
            break;
        case LockType::Pressure:
            --pressure_users;
            break;
        }
        update();
    }

    struct [[maybe_unused]] Lock
    {
        explicit Lock(Pump* parent, LockType kind)
            : parent_(*parent)
            , kind_(kind)
        {
            parent_.add_user(kind_);
        }
        ~Lock() { parent_.remove_user(kind_); }
        Lock(const Lock& token) = delete;
        Lock(Lock&& token) = delete;
        Lock& operator=(const Lock& token) = delete;
        Lock& operator=(Lock&& token) = delete;

    private:
        Pump& parent_;
        LockType kind_;
    };

    friend class Lock;

public:
    inline Lock use_pressure() { return Lock(this, LockType::Pressure); }
    inline Lock use_suction() { return Lock(this, LockType::Suction); }
}; // Pump

extern Pump<PRESSURE_PUMP_EN_PIN, PRESSURE_VALVE_PUMP_OUT_PIN> pump;

} // namespace pneumatics