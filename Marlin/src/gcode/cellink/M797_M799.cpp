// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ALL(CELLINK_REPORTING, OPTICAL_AUTOCAL)

  #include "../gcode.h"
  #include "../../feature/optical_autocal.h"
  #include "cellink_reporting.h"

  /**
   * @brief Reset nozzle calibration status, aliases G92.1
   * 
   */
  void GcodeSuite::M797()
  {
    parser.subcode = 1;
    G92();
    optical_autocal.reset();
  }

  /**
   * @brief Get current tool calibration status
   * 
   */
  void GcodeSuite::M798()
  {
    const auto tool = get_target_extruder_from_command();
    // TODO: add multiple tool support
    SERIAL_ECHO_CELLINK_KV("AT", tool);
    SERIAL_ECHOLN_CELLINK_KV("AUTOCAL", optical_autocal.is_calibrated());
  }

  /**
   * @brief Get current tool calibration offsets
   * 
   */
  void GcodeSuite::M799()
  {
    const auto tool = get_target_extruder_from_command();
    // TODO: add multiple tool support
    const auto &offset = optical_autocal.offset();
    SERIAL_ECHO_CELLINK_KV("AT", tool);
    SERIAL_ECHO_CELLINK_KV("AUTOCAL_XOFF", offset.x);
    SERIAL_ECHO_CELLINK_KV("AUTOCAL_YOFF", offset.y);
    SERIAL_ECHOLN_CELLINK_KV("AUTOCAL_ZOFF", offset.z);
  }

#endif // CELLINK_REPORTING && OPTICAL_AUTOCAL