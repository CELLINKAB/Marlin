
#include "../MarlinCore.h"

#if ENABLED(OPTICAL_AUTOCAL)

  #include "../gcode/gcode.h"
  #include "optical_autocal.h"

  OpticalAutocal<OPTICAL_SENSOR_1_PIN, OPTICAL_SENSOR_2_PIN> optical_autocal;

  void GcodeSuite::G510()
  {
      const float feedrate = parser.floatval('F', AUTOCAL_DEFAULT_FEEDRATE);
      const float z_increment = parser.floatval('Z', AUTOCAL_DEFAULT_Z_INCREMENT);
      const uint8_t cycles = min(parser.byteval('S', AUTOCAL_DEFAULT_CYCLES), 
                                 MAX_AUTOCAL_CYCLES);
      optical_autocal.full_autocal_routine(cycles, z_increment, feedrate);
  }

#endif