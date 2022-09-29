// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ENABLED(CELLINK_REPORTING)

  #include "../gcode.h"

  /**
   * @brief Reset nozzle calibration status, aliases G92.1
   * 
   */
  void GcodeSuite::M797()
  {
    parser.subcode = 1;
    G92();
  }

#endif // CELLINK_REPORTING