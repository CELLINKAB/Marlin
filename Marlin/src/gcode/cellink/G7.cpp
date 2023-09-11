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
    G0_G1(TERN_(HAS_FAST_MOVES, true));
    axis_relative = rel_mode;
  }

#endif // G7_RELATIVE_MOVE