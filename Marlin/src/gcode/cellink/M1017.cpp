// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ENABLED(CELLINK_REPORTING)

#    include "../../feature/cellink_reporter.h"

/**
   * @brief report tool offesets
   * 
   */
void GcodeSuite::M1017()
{
    cellink::reporter.m1017.set_interval(parser.byteval('S'));
    cellink::reporter.m1017.report();
}

#endif // CELLINK_REPORTING