#pragma once

#include "../inc/MarlinConfig.h"

#include "../module/planner.h"

#if ENABLED(GLOBAL_INTERVAL_REPORTER)
  #include "interval_reporter.h"
#endif

#include <string_view>
#include <numeric>
// using namespace std::string_view_literals;

struct OpticalSurfaceProbe
{
    OpticalSurfaceProbe()
    {
        SET_INPUT(OPT_SURF_IN_PIN);
        SET_INPUT(OPT_SURF_ERR_PIN);
        SET_OUTPUT(OPT_SURF_MFI_PIN);
        OUT_WRITE(OPT_SURF_LED_ON_PIN, HIGH);
    }

    void init();

    uint32_t get_distance_raw() const
    {
        return analogRead(OPT_SURF_IN_PIN);
    }

    float get_distance() const
    {
        float distances[5]{};

        for (float& distance : distances) {
            auto raw = get_distance_raw();
            distance = (raw + 2448) / 145.52f;
            delay(20);
        }
        return std::accumulate(std::cbegin(distances), std::cend(distances), 0.0f) / 5;
    }

    #if ENABLED(GLOBAL_INTERVAL_REPORTER)
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
    #endif

    /**
     * @brief TODO: unimplemented, intended to alter sensor angle
     * 
     */
    void deploy() {}

    /**
     * @brief TODO: unimplemented, intended to alter sensor angle
     * 
     */
    void retract() {}

private:
    constexpr static size_t READ_BUFFER_SIZE{255};

    #if ENABLED(GLOBAL_INTERVAL_REPORTER)
        IntervalReporter reporter{[this]
                                {
                                    const auto position = planner.get_axis_positions_mm();
                                    SERIAL_ECHOLNPAIR("prb:", get_distance(), ",X:", position.x, ",Y:", position.y, ",Z:", position.z);
                                }};
    #endif

    /**
     * @brief Private inner struct for holding serial communication
     *        protocols and other sensor specific implementation details
     * 
     */
    struct
    {
        using str = std::string_view;

        void begin() { serial.begin(115200); }

        // user functions
        enum class User
        {
            User,
            Professional
        };
        void login(const User, const str);
        void set_password(const str);
        User get_user() const;
        void set_default_user(const User);

        enum class Trigger
        {
            None,
            Pulse,
            Edge,
            Software
        };
        void set_trigger(const Trigger);

        enum class MFILevel
        {
            High,
            Low
        };
        void set_mfi_level(const MFILevel);

        enum class OutputMode
        {
            None,
            Uart,
            Analog
        };
        void set_output_mode(const OutputMode);

        void set_analog_scale_default();
        void set_analog_scale(const float min, const float max);

        void get_info() const;
        void get_all_settings() const { serial.println("PRINT"); }
        void set_all_factory_defaults();

    private:
        HardwareSerial &serial{OPT_SURF_HW_SERIAL};

        void read_response()
        {
            char read_buf[READ_BUFFER_SIZE] = {}; // zero initialized buffer
            serial.readBytesUntil('>', read_buf, READ_BUFFER_SIZE);
            str parse_string{read_buf, READ_BUFFER_SIZE};
        }

    } protocol;
};
