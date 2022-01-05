#pragma once

#include "../inc/MarlinConfig.h"

#include <array>

struct IntervalReporter
{

    explicit IntervalReporter(callback_function_t reporter);

    ~IntervalReporter();

    // it does not make sense to copy or move a reporter
    IntervalReporter(const IntervalReporter&) = delete;
    IntervalReporter(IntervalReporter&&) = delete;
    IntervalReporter& operator=(const IntervalReporter) = delete;
    IntervalReporter& operator=(IntervalReporter&&) = delete;

    inline void start() { if (channel) channel->enabled = true; }

    inline void stop() { if (channel) channel->enabled = false; }

    inline static uint32_t get_interval_ms() { return get_interval_us() / 1000UL; }

    inline static uint32_t get_interval_us() { 
        return timer.getOverflow(TimerFormat_t::MICROSEC_FORMAT);
    }

    inline static void set_interval_ms(const uint32_t interval_ms) { 
        set_interval_us(interval_ms * 1000);
    }

    static void set_interval_us(const uint32_t interval_us);

private:
    static HardwareTimer timer;

    struct Reporter
    {
        bool valid;
        bool enabled;
        callback_function_t fn;

        Reporter() = default;
        explicit Reporter(callback_function_t fn) : valid(true), enabled(false), fn(fn) {}
    };

    Reporter * channel;

    using ReporterArray = std::array<Reporter, NUM_INTERVAL_REPORTER_SLOTS>;
    static ReporterArray reporters;

    static void report_all();

    static void refresh();
};