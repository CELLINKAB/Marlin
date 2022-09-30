// Copyright Cellink 2022 - GPLv3

#pragma once

#    define SERIAL_ECHO_CELLINK_KV(K, V) \
        SERIAL_CHAR(','); \
        SERIAL_ECHO(K); \
        SERIAL_CHAR(':'); \
        SERIAL_ECHO(V);
#    define SERIAL_ECHOLN_CELLINK_KV(K, V) \
        SERIAL_ECHO_CELLINK_KV(K, V); \
        SERIAL_EOL();