#pragma once

#include "../inc/MarlinConfig.h"

template<pin_t PIN>
struct Debounced
{
    const millis_t period;
    const bool inverting;
    millis_t debounce_until;
    bool state;

    explicit Debounced(millis_t debounce_time, bool invert = false, bool pullup = false)
        : period(debounce_time)
        , inverting(invert)
        , debounce_until(0)
        , state(false)
    {
        if (pullup) {
            SET_INPUT_PULLUP(PIN);
        } else {
            SET_INPUT_PULLDOWN(PIN);
        };
    }

    bool read_now() const { return static_cast<bool>(READ(PIN)) ^ inverting; }

    bool read() const { return state; }

    void update(millis_t now = millis())
    {
        if (now <= debounce_until)
            return;

        if (debounce_until != 0) {
            state = read_now();
            debounce_until = 0;
        }
        else if (state != read_now()) {
            debounce_until = now + period;
        }
    }


};