// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ENABLED(CELLINK_REPORTING)

  #include "../gcode.h"
  #include "../../module/motion.h"

  /**
   * @brief get active tool in cellink protocol format
   * 
   */
  void GcodeSuite::M824()
  {
    SERIAL_ECHOLNPGM("AT:", active_extruder);
  }

#endif // CELLINK_REPORTING