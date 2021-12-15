
#include "../MarlinCore.h"

#if ENABLED(OPTICAL_AUTOCAL)

  #include "../gcode/gcode.h"
  #include "optical_autocal.h"

  OpticalAutocal optical_autocal;

  void GcodeSuite::G510()
  {
      const float feedrate = parser.floatval('F', AUTOCAL_DEFAULT_FEEDRATE);
      if (optical_autocal.full_autocal_routine(feedrate))
        GcodeSuite::process_subcommands_now("G92 X0 Y0 Z20");
  }

#endif