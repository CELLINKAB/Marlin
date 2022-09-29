// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ENABLED(G7_RELATIVE_MOVE)

  #include "../gcode.h"
  #include "../../module/planner.h"

  /**
   * @brief report homing status in cellink protocol format
   * 
   */
  void GcodeSuite::G7()
  {
    // ensure relative mode consistency after move
    axis_bits_t rel_mode = axis_relative;
    set_relative_mode(true);
    G0_G1();
    axis_relative = rel_mode;
  }

#endif // G7_RELATIVE_MOVE