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

#if ENABLED(CHANTARELLE_SUPPORT)

#include "request.h"
#include "test.h"

using namespace printhead;

namespace test
{

/**
 * @brief Trigger compiler error if Status size changes, which would invalidate conversions
 * 
 */
void status_fixed_size()
{
    static_assert_eq(sizeof(Status), 2);
}

/**
 * @brief Trigger compiler error if conversion does not match assumptions
 * 
 */
void status_raw_conversions()
{
    static constexpr uint16_t raw = 0b0000'0000'1011'0100;
    static constexpr Status status(raw);
    static_assert(status.enable);
    static_assert(status.is_homing);
    static_assert_eq(raw, status.to_raw());
}

}

#endif // CHANTARELLE_SUPPORT