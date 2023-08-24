#pragma once

#include "../inc/MarlinConfigPre.h"
#include "../libs/autoreport.h"

namespace cellink {

template<typename K, typename V>
void serial_echo_kv(K key, V value)
{
    SERIAL_CHAR(',');
    SERIAL_ECHO(key);
    SERIAL_CHAR(':');
    SERIAL_ECHO(value);
}

template<typename K, typename V, class... Rest>
void serial_echo_kv(K key, V value, Rest... rest)
{
    serial_echo_kv(key, value);
    serial_echo_kv(rest...);
}

template<typename K, typename V>
void serial_echoln_kv(K key, V value)
{
    serial_echo_kv(key, value);
    SERIAL_EOL();
}

template<typename K, typename V, class... Rest>
void serial_echoln_kv(K key, V value, Rest... rest)
{
    serial_echo_kv(key, value);
    serial_echo_kv(rest...);
    SERIAL_EOL();
}

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
    void tick_all();
};

extern Reporter reporter;

} // namespace cellink