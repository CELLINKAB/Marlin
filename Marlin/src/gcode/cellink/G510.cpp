#include "../../inc/MarlinConfig.h"

#if ENABLED(OPTICAL_AUTOCAL)

#include "../gcode.h"
#include "../../feature/optical_autocal.h"
#include <stdio.h>

void GcodeSuite::G510()
{
  const auto feedrate = parser.feedrateval('F', AUTOCAL_DEFAULT_FEEDRATE);
  if (optical_autocal.full_autocal_routine(feedrate))
  {
    static constexpr size_t SUB_COMMAND_SIZE = 20;
    char command[SUB_COMMAND_SIZE];
    snprintf(command, SUB_COMMAND_SIZE, "G92 X0 Y0 Z%d", POST_AUTOCAL_SAFE_Z_HEIGHT);
    GcodeSuite::process_subcommands_now(command);
  }
}

#endif