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
#pragma once

#include "../inc/MarlinConfigPre.h"
#include "../libs/autoreport.h"

namespace cellink {

/**
 * @brief Print key value pair(s) in cellink middleware format
 * 
 * @tparam Key
 * @tparam Value
 * @param key 
 * @param value 
 */
template<typename Key, typename Value>
void serial_echo_kv(Key key, Value value)
{
    SERIAL_CHAR(',');
    SERIAL_ECHO(key);
    SERIAL_CHAR(':');
    SERIAL_ECHO(value);
}

/**
 * @brief Print key value pair(s) in cellink middleware format 
 * 
 * @tparam Key 
 * @tparam Value 
 * @tparam Rest 
 * @param key 
 * @param value 
 * @param rest 
 */
template<typename Key, typename Value, class... Rest>
void serial_echo_kv(Key key, Value value, Rest... rest)
{
    serial_echo_kv(key, value);
    serial_echo_kv(rest...);
}

/**
 * @brief Print key value pair(s) in cellink middleware format and end with newline
 * 
 * @tparam Args variadic key-value types
 * @param args variadic key-value pairs 
 */
template<typename ... Args>
void serial_echoln_kv(Args... args)
{
    serial_echo_kv(args...);
    SERIAL_EOL();
}

/**
 * @brief Unified auto reporter to reduce boilerplate of multiple custom reports
 * 
 */
struct Reporter
{
    struct M119 : AutoReporter<M119>
    {
        static void report();
    } m119;
    struct M772 : AutoReporter<M772>
    {
        static void report();
    } m772;
    struct M798 : AutoReporter<M798>
    {
        static void report();
    } m798;
    struct M799 : AutoReporter<M799>
    {
        static void report();
    } m799;
    struct M802 : AutoReporter<M802>
    {
        static void report();
    } m802;
    struct M814 : AutoReporter<M814>
    {
        static void report();
    } m814;
    struct M816 : AutoReporter<M816>
    {
        static void report();
    } m816;
    struct M821 : AutoReporter<M821>
    {
        static void report();
    } m821;
    struct M825 : AutoReporter<M825>
    {
        static void report();
    } m825;
    struct M1015 : AutoReporter<M1015>
    {
        static void report();
    } m1015;
    struct M1016 : AutoReporter<M1016>
    {
        static void report();
    } m1016;

    /**
     * @brief poll in background for all internal helpers
     * 
     */
    void tick_all();
};

extern Reporter reporter;

} // namespace cellink