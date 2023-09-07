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
#include <array>
#include <cstddef>

namespace test {

/**
 * @brief static assert helper to capture type and value as type information
 * 
 * @tparam T 
 * @tparam val 
 */
template<typename T, T val>
struct _static_assert_val{
    constexpr _static_assert_val() = default;
    constexpr _static_assert_val(T v) {};
};

/**
 * @brief templated static assert generating error messages which contain type and value
 * 
 * @tparam T left type
 * @tparam A left value
 * @tparam U right type
 * @tparam B right value
 */
template<typename T, T A, typename U, U B>
constexpr void _static_assert_eq(_static_assert_val<T, A>, _static_assert_val<U, B>){
    static_assert(A == B, "Values not equal!");
}

#define static_assert_eq(A, B) _static_assert_eq(_static_assert_val<decltype(A), A>{}, _static_assert_val<decltype(B), B>{})

/**
 * @brief lexigraphically compare arrays for equality at compile time
 * 
 * @tparam T common array type
 * @tparam N lhs array size
 * @tparam M rhs array size
 * @param a left array
 * @param b right array
 * @return true 
 * @return false 
 */
template<typename T, size_t N, size_t M>
constexpr auto array_compare(const std::array<T,N>& a, const std::array<T,M>& b) -> bool
{
    bool no_mismatches = true;
    for (size_t i = 0; i < M; ++i)
        no_mismatches &= (a[i]==b[i]);
    return no_mismatches;
};

} // namespace test