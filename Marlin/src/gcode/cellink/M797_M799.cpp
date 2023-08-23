// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ALL(CELLINK_REPORTING, OPTICAL_AUTOCAL)

#    include "../../feature/cellink_reporter.h"
#    include "../../feature/optical_autocal.h"
#    include "../gcode.h"

void Reporter::M798::report()
{
    serial_echoln_kv("AT", active_extruder, "AUTOCAL", optical_autocal.is_calibrated(active_extruder));
}

void Reporter::M799::report()
{
    const auto& offset = optical_autocal.offset(active_extruder);
    serial_echoln_kv("AT",
                     active_extruder,
                     "AUTOCAL_XOFF",
                     offset.x,
                     "AUTOCAL_YOFF",
                     offset.y,
                     "AUTOCAL_ZOFF",
                     offset.z);
}

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

#elif ENABLED(CELLINK_REPORTING)

#    include "../../feature/cellink_reporter.h"
#    include "../gcode.h"

void cellink::Reporter::M798::report() {}
void cellink::Reporter::M799::report() {}

void GcodeSuite::M797() {}
void GcodeSuite::M798() {}
void GcodeSuite::M799() {}
#endif // CELLINK_REPORTING && OPTICAL_AUTOCAL