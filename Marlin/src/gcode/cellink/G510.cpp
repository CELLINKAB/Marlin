#include "../../inc/MarlinConfig.h"

#if ENABLED(OPTICAL_AUTOCAL)

#include "../gcode.h"
#include "../../feature/optical_autocal.h"

void GcodeSuite::G510()
{
  const auto feedrate = parser.feedrateval('F', AUTOCAL_DEFAULT_FEEDRATE);
  if (optical_autocal.full_autocal_routine(feedrate))
  {
    char command[] = "G92 X0 Y0 Z20";
    GcodeSuite::process_subcommands_now(command);
  }
}

#endif