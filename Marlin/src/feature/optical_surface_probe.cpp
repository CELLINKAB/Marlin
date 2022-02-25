#include "../inc/MarlinConfig.h"

#if ENABLED(OPTICAL_SURFACE_PROBE)
  #include "optical_surface_probe.h"
  #include "../gcode/gcode.h"
  #include "../gcode/parser.h"

  static OpticalSurfaceProbe opt_probe;

#endif // OPTICAL_SURFACE_PROBE