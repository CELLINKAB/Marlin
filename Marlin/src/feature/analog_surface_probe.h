// Copyright Cellink 2022

#pragma once

#include "interval_reporter.h"

#include <numeric>

template <typename Impl, pin_t PIN>
struct AnalogProbe
{
    using AnalogProbe_t = AnalogProbe<Impl, PIN>;

        uint32_t get_distance_raw() const
    {
        return analogRead(PIN);
    }

    float get_distance() const {
        return static_cast<const Impl*>(this)->get_distance_impl();
    }

    float get_distance_avg() const {
        float readings[10]{};
        for (auto& val : readings) {
            val = get_distance();
        }
        return std::accumulate(std::cbegin(readings), std::cend(readings), 0.0f);
    }

    bool probe_to_z(const float z, const float fr_mm_s) {
        return static_cast<Impl*>(this)->probe_to_z_impl(z, fr_mm_s);
    }

    bool stow() {
        return static_cast<Impl*>(this)->stow_impl();
    }

    bool deploy() {
        return static_cast<Impl*>(this)->deploy_impl();
    }

    float probe_val() const {
        return static_cast<const Impl*>(this)->probe_val_impl();
    }

        #if ENABLED(GLOBAL_INTERVAL_REPORTER)
        AnalogProbe(callback_function_t reporter_) : reporter(reporter_) {}
        /**
         * @brief reports analog sensor reading at a set interval 
         *        using a hardware timer based interrupt
         * 
         * @param ms milliseconds between reports
         */
        void interval_report(bool start)
        {
            if (start)
                reporter.start();
            else
                reporter.stop();
        }

        IntervalReporter reporter;
    #endif
};

#if ENABLED(RETRACTING_DISPLACEMENT_PROBE)
  #include "retracting_displacement_probe.h"
#elif ENABLED(OPTICAL_SURFACE_PROBE)
  #include "optical_surface_probe.h"
#endif