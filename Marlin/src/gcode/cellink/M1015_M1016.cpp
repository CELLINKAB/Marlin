// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ENABLED(CELLINK_REPORTING)

#include "../../feature/cellink_reporter.h"
#    include "../gcode.h"


/**
   * @brief Get current position in cellink protocol format
   * 
   */
void GcodeSuite::M1015()
{
    cellink::reporter.m1015.set_interval(parser.byteval('S'));
    cellink::reporter.m1015.report();
}

/**
   * @brief Get current machine position in cellink protocol format
   * 
   */
void GcodeSuite::M1016()
{
    cellink::reporter.m1016.set_interval(parser.byteval('S'));
    cellink::reporter.m1016.report();
}

#endif // CELLINK_REPORTING