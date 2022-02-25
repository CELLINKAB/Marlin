#include "../inc/MarlinConfig.h"

#if ENABLED(HAS_ANALOG_PROBE)
  #include "../gcode/gcode.h"
  #include "analog_surface_probe.h"

  void GcodeSuite::M1100()
  {
    const auto val = analog_probe.get_distance_avg();
    SERIAL_ECHOLNPGM("Probe avg reading: ", val);

    #if ENABLED(GLOBAL_INTERVAL_REPORTER)
      const auto ms = parser.intval('P', 0);
      if (ms > 0) IntervalReporter::set_interval_ms(ms);

      const auto start = parser.boolval('S');
      analog_probe.interval_report(start);
    #endif
  }
#endif // NO_ANALOG_PROBE