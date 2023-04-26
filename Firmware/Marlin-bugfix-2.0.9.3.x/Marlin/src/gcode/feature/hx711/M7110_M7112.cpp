/*******************************************************************************
FILE
    M7110_M7112.cpp

ORIGINAL AUTHOR
    Borislav Cvejic

DESCRIPTION
    This file implements M commands used to manipulate HX711

VERSION
    1.0.0

REFERENCES
    None

*******************************************************************************/

#include "../../../inc/MarlinConfig.h"

#if ENABLED(HX711_WSCALE)

#include "../../gcode.h"
#include "../../../feature/hx_711.h"
#include "src/MarlinCore.h"

/**
 * @brief   Sets the weighting scale threshold value for end stop.
 *          Function is non-waiting. Syntax: M7110 P########
 */
void GcodeSuite::M7110()
{
    if (parser.seenval('P'))
    {
        wScale.setThreshold(parser.value_float());
    }
}

/**
 * @brief Sets the mode and channel of the HX711. Syntax: M7111 P#
 *
 */
void GcodeSuite::M7111()
{
    if (!parser.seenval('P'))
    {
        return;
    }

    const uint8_t chan = parser.value_byte();
    if ((chan >= 1) && (chan <= 3)) // supported values are 1, 2 and 3.
    {
        wScale.setChannel(chan);
    }
}

/**
 * @brief   Prints on the command port the raw value of HX711.
 *          "T" param: Start tare operation of the strain gauge, and enables it.
 *          Syntax: M7112
 *                  M7112 T
 *
 */
void GcodeSuite::M7112()
{
    if (parser.seenval('T'))
    {
        wScale.tare_start();
        while( wScale.tare_ready() == false ) idle();
        wScale.enable_out(true);
        SERIAL_ECHOLNPGM("HX711:Tare done; strain gauge enabled.");
        return;
    }
    // Optimized print to serial.
    SERIAL_ECHOLNPGM("HX711:Raw val: ", wScale.getCurrentVal());
}
#endif