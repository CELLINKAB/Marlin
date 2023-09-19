/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 Cellink [https://github.com/CELLINKAB/Marlin]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */


#include "../../inc/MarlinConfig.h"

#if ENABLED(CELLINK_REPORTING)

  #include "../gcode.h"
  #include "../../module/motion.h"
  #include "../../feature/cellink_reporter.h"

  void cellink::Reporter::M821::report() {
    cellink::serial_echoln_kv("HOME", all_axes_homed());
  }


  /**
   * @brief report homing status in cellink protocol format
   * 
   */
  void GcodeSuite::M821()
  {
    cellink::reporter.m821.report();
    cellink::reporter.m821.set_interval(parser.byteval('S'));
  }

#endif // CELLINK_REPORTING