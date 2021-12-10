#include "../inc/MarlinConfig.h"


#if ENABLED(GLOBAL_INTERVAL_REPORTER)


    #include "interval_reporter.h"

    IntervalReporter::IntervalReporter(callback_function_t reporter) : channel(nullptr)
    {
        for (auto &reporter_slot : reporters)
        {
            if (!reporter_slot.valid)
            {
                reporter_slot = Reporter(reporter);
                channel = &reporter_slot;
                break;
            }
        }

        if (!channel)
        {
            SERIAL_ERROR_MSG("Too many reporters registered! Some reports will not appear!");
            return;
        }

        refresh();
    }

    IntervalReporter::~IntervalReporter()
    {
        if (channel)
        {
            channel->valid = false;
            channel->enabled = false;
        }
    }

    void IntervalReporter::set_interval_us(const uint32_t interval_us)
    {
        timer.setOverflow(interval_us, TimerFormat_t::MICROSEC_FORMAT);
        refresh();
    }

    void IntervalReporter::report_all()
    {
        for (const auto &reporter : reporters)
            if (reporter.enabled)
                reporter.fn();
    }

    void IntervalReporter::refresh()
    {
        timer.pause();
        timer.refresh();
        timer.resume();
    }

    HardwareTimer IntervalReporter::timer{[]() {
        HardwareTimer timer{INTERVAL_REPORTER_TIMER}; 
        timer.setOverflow(INTERVAL_REPORTER_DEFAULT_INTERVAL, TimerFormat_t::MICROSEC_FORMAT); 
        timer.attachInterrupt(IntervalReporter::report_all);
        return timer;
        }()};
    IntervalReporter::ReporterArray IntervalReporter::reporters{};

#endif // GLOBAL_INTERVAL_REPORTER