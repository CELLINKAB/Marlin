/*******************************************************************************
FILE
    M1051.cpp

ORIGINAL AUTHOR
    Milan Ivezic

DESCRIPTION
    This file implements M1051 which is used to report back the firmware code branch,
    version, date etc.

VERSION
    1.0.0

REFERENCES
    None

*******************************************************************************/

#include "../../inc/MarlinConfig.h"


#if ENABLED(CELLINK_REPORTING)

#include "../gcode.h"
#include "src/MarlinCore.h"

void GcodeSuite::M1051()
{
    SERIAL_ECHOLNPGM("TFW_BUILD: ", VER_SEM_VER);
    SERIAL_ECHOLNPGM("TFW_BRANCH: ", VER_BRANCH);
    SERIAL_ECHOLNPGM("TFW_COMMIT: ", VER_CURRENT_COMMIT);
    SERIAL_ECHOLNPGM("TFW_DATE: ", VER_TIMESTAMP);
}

#endif