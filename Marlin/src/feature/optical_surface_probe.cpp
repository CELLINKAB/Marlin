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

    const auto ms = parser.intval('P', 0);
    probe.interval_report(ms);
  }

#endif // OPTICAL_SURFACE_PROBE