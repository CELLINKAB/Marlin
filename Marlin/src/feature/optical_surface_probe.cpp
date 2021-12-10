#include "../inc/MarlinConfig.h"

#if ENABLED(OPTICAL_SURFACE_PROBE)
  #include "optical_surface_probe.h"
  #include "../gcode/gcode.h"
  #include "../gcode/parser.h"

  static OpticalSurfaceProbe probe;

  void GcodeSuite::M1100()
  {
    const auto val = probe.get_distance();
    SERIAL_ECHOLNPAIR("Probe raw reading: ", val);

    #if ENABLED(GLOBAL_INTERVAL_REPORTER)
      const auto ms = parser.intval('P', 0);
      if (ms > 0) IntervalReporter::set_interval_ms(ms);

      const auto start = parser.boolval('S');
      probe.interval_report(start);
    #endif
  }

#endif // OPTICAL_SURFACE_PROBE