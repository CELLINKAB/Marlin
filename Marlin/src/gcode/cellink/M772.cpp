// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ENABLED(CELLINK_REPORTING)
#    include "../../feature/cellink_reporter.h"
#    include "../../module/temperature.h"
#    include "../gcode.h"

void cellink::Reporter::M772::report()
{
    EXTRUDER_LOOP()
    {
        SERIAL_ECHOPGM(",PH", e, "T:", Temperature::degHotend(e));
    }
    SERIAL_EOL();
}

void GcodeSuite::M772()
{
    cellink::reporter.m772.set_interval(parser.byteval('S'));
    cellink::reporter.m772.report();
}

#endif // CELLINK_REPORTING