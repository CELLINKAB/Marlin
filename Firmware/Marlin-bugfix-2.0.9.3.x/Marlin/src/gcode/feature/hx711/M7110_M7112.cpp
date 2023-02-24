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

/**
 * @brief   Sets the weighting scale threshold value for end stop.
 *          Function is non-waiting. Syntax: M7110 P########
 */
void GcodeSuite::M7110() {
    if(parser.seenval('P'))
    {
        wScale.setThreshold(parser.value_long());
    }
}

/**
 * @brief Sets the mode and channel of the HX711. Syntax: M7111 P#
 * 
 */
void GcodeSuite::M7111() {
    if(parser.seenval('P'))
    {
        char chan = parser.value_byte();
        if( (chan > 1 )||(chan < 3) )           //supported values are 1, 2 nad 3.
        {
            wScale.setChannel(parser.value_long());
        }
        //otherwise do nothing.
    }
}

/**
 * @brief   Prints on the command port the raw value of HX711.
 *          Syntax: M7112
 * 
 */
void GcodeSuite::M7112() {
    // Optimized print to serial.
    SERIAL_ECHOPGM("HX711 raw val: ");          //print from PGM
    SERIAL_ECHO_F(wScale.getCurrentVal());      //print float
    SERIAL_EOL();                               //print EOL
}

#endif