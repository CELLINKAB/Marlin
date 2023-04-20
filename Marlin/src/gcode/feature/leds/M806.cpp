/*******************************************************************************
FILE
    M806.cpp

ORIGINAL AUTHOR
    Borislav Cvejic

DESCRIPTION
    This file implements M commands to manipulate UV LED sterilization

VERSION
    1.0.0

REFERENCES
    None

*******************************************************************************/

#include "../../../inc/MarlinConfig.h"

#if ENABLED(UV_LED_STERILIZATION)

#include "../../gcode.h"

/**
 * @brief
 *
 */
void GcodeSuite::M806() {
    if(parser.seenval('I'))
    {
        int32_t input_val= parser.value_long();
        uint8_t set_val = 0u;

        pinMode(UV_LED_OUTPUT_PIN, OUTPUT);

        if(input_val > 255)
        {
            set_val = 255u;
        }else if(input_val < 0)
        {
            set_val = 0u;
        }else
        {
            set_val = (uint8_t)input_val;
        }
        analogWrite(UV_LED_OUTPUT_PIN, set_val);
    }
}

#endif