// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ENABLED(CELLINK_REPORTING)

  #include "../gcode.h"
  #include "../../module/motion.h"

  /**
   * @brief report homing status in cellink protocol format
   * 
   */
  void GcodeSuite::M821()
  {
    SERIAL_ECHOLNPGM("HOME:", all_axes_homed());
  }

#endif // CELLINK_REPORTING