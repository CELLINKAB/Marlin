// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ENABLED(CELLINK_REPORTING)

#    include "../../feature/cellink_reporter.h"
#    include "../gcode.h"

/**
   * @brief report tool offesets
   * 
   */
void GcodeSuite::M1017()
{
    /*output unused in com-module, all status available in M503 report*/
}

#endif // CELLINK_REPORTING