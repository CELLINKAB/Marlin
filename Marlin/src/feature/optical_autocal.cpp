
#include "../MarlinCore.h"

#if ENABLED(OPTICAL_AUTOCAL)

  #include "../gcode/gcode.h"
  #include "optical_autocal.h"

  OpticalAutocal optical_autocal;

  void GcodeSuite::G510()
  {
      const float feedrate = parser.floatval('F', AUTOCAL_DEFAULT_FEEDRATE);
      if (optical_autocal.full_autocal_routine(feedrate))
      {
        char command[] = "G92 X0 Y0 Z20";
        GcodeSuite::process_subcommands_now(command);
      }
  }

#endif