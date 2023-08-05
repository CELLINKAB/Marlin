// Copyright Cellink 2022 - GPLv3

/*******************************************************************************
FILE
    M801_M802.cpp

ORIGINAL AUTHOR
    Milan Ivezic

DESCRIPTION
    Wrapper M-codes for temperature control to ensure compatibility

VERSION
    1.0.0

REFERENCES
    None

*******************************************************************************/

#include "../../inc/MarlinConfig.h"

#if ENABLED(CELLINK_REPORTING)

#include "../../module/temperature.h"
#include "../gcode.h"

void GcodeSuite::M800()
{
    thermalManager.setTargetBed(0);
    TERN_(PRINTJOB_TIMER_AUTOSTART, thermalManager.auto_job_check_timer(false, true));
    SERIAL_ECHOLNPGM("echo: Bed cooling");
}

void GcodeSuite::M801()
{
    M140();
}

void GcodeSuite::M802()
{
    SERIAL_ECHOLNPGM("PBT:", Temperature::degBed());
}

#endif