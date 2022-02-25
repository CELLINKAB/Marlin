#include "../inc/MarlinConfig.h"

#if ENABLED(RETRACTING_DISPLACEMENT_PROBE)
  
  #include "../gcode/gcode.h"
  #include "../module/planner.h"
  #include "retracting_displacement_probe.h"

  RetractingDisplacementProbe rd_probe;
  RetractingDisplacementProbe::AnalogProbe_t& analog_probe = rd_probe;

  void GcodeSuite::G529()
  {
      if (parser.seen('S'))
        rd_probe.stow();
      else if (parser.seen('D'))
        rd_probe.deploy();
      else if (parser.seen('P')) {
        const auto x = parser.floatval('X', planner.get_axis_position_mm(AxisEnum::X_AXIS));
        const auto y = parser.floatval('Y', planner.get_axis_position_mm(AxisEnum::Y_AXIS));
        const auto z = rd_probe.debug_probe(x, y);
        SERIAL_ECHOLNPGM("X=", x, ",Y=", y, ",P=", z);
      } else
        SERIAL_ECHO_MSG("G529 requires 'S', 'D' or 'P' parameter");
  }

#endif