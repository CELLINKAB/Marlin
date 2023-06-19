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
 * @brief   Sets the weighting scale threshold value for the end stop.
 *          Or/And sets the channel/gain settings for the ADC.
 *          Syntax: M7110 P######## C########
 * NOTE:    After calibration the threshold value is in grams.
 */
void GcodeSuite::M7110()
{
    // Threshold parameter value
    if (parser.seenval('P'))
    {
        wScale.setThreshold(parser.value_float());
    }
    // ADC channel/gain parameter value
    if (parser.seenval('C'))
    {
        const uint8_t chan = parser.value_byte();
        if ((chan >= 1) && (chan <= 3)) // supported values are 1, 2 and 3.
        {
            wScale.setChannel(chan);
        }
    }
    // Scale parameter value
    if (parser.seenval('S'))
    {
        wScale.setScale(parser.value_float());
    }
    // Scale direction parameter
    if (parser.seenval('D'))
    {
        const int8_t dir = parser.value_byte();
        if ((dir == 1) || (dir == -1)) // supported values are -1 and 1.
        {
            wScale.setScaleDir(dir);
        }
    }
}

/**
 * @brief Tare and Calibrate operation command. Syntax: M7111 T0 or M7111 W#####
 *          #### - is known weight used to calibrate.
 *
 */
void GcodeSuite::M7111()
{
    if (parser.seenval('T'))
    {
        float tare_output = 0.0f;
        wScale.tare_start();
        while( wScale.tare_ready(tare_output) == false )
        {
            idle();
        }
        wScale.enable_out(true);
        SERIAL_ECHOLNPGM("HX711:Tare: Done at val: ", tare_output);
        return;
    }
    if (parser.seenval('W'))
    {
        if( wScale.read_enable_out() )
        {
            float calib_output = 0.0f;
            wScale.calibrate_start(parser.value_float());
            while( wScale.calibrate_ready(calib_output) == false )
            {
                idle();
            }
            SERIAL_ECHOLNPGM("HX711:Calibration: Done at val: ",calib_output);
        } else {
            SERIAL_ECHOLNPGM("HX711:Error: Cannot calibrate before tare.");
        }
        return;
    }
}

/**
 * @brief   Prints on the command port the raw value of HX711.
 *          Syntax: M7112
 *
 */
void GcodeSuite::M7112()
{
    // Optimized print to serial.
    SERIAL_ECHOLNPGM("HX711:Raw val: ", wScale.getCurrentVal());
    //debug only
    SERIAL_ECHOLNPGM("HX711:Offset: ", wScale.getOffset());
    SERIAL_ECHOLNPGM("HX711:Scale: ", wScale.getScale());
    SERIAL_ECHOLNPGM("HX711:Threshold: ", wScale.getThreshold());
    SERIAL_ECHOLNPGM("HX711:Channel: ", wScale.getChannel());
}

void GcodeSuite::M7110_report(const bool forReplay/*=true*/) {
  report_heading_etc(forReplay, F(STR_STRAIN_GAUGE));
  SERIAL_ECHOLNPGM_P(
    PSTR("  M7110 P"), wScale.getThreshold(),
    PSTR("  C"), wScale.getChannel(),
    PSTR("  S"), wScale.getScale(),
    PSTR("  D"), wScale.getScaleDir()
    );
}
#endif