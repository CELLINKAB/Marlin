// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ALL(CELLINK_REPORTING, OPTICAL_AUTOCAL)

#    include "../../feature/cellink_reporter.h"
#    include "../gcode.h"

/**
   * @brief Reset nozzle calibration status, aliases G92.1
   * 
   */
void GcodeSuite::M797()
{
    optical_autocal.reset_all();
    process_subcommands_now(F("G510 R"));
}

/**
   * @brief Get current tool calibration status
   * 
   */
void GcodeSuite::M798()
{
    cellink::reporter.m798.set_interval(parser.boolval('S'));
    cellink::reporter.m798.report();
}

/**
   * @brief Get current tool calibration offsets
   * 
   */
void GcodeSuite::M799()
{
    cellink::reporter.m799.set_interval(parser.boolval('S'));
    cellink::reporter.m799.report();
}

#endif // CELLINK_REPORTING && OPTICAL_AUTOCAL