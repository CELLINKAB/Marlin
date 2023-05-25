/*******************************************************************************
FILE
    hx_711.h

ORIGINAL AUTHOR
    Borislav Cvejic

DESCRIPTION
    This module implements non blocking communication driver for HX711,
    management function and interfaces. The communication is custom type
    for hx711 and is implemented in software.

VERSION
    1.0.0

REFERENCES
    None

*******************************************************************************/
#pragma once

#include "../inc/MarlinConfig.h"
#include "Arduino.h"
#include "inttypes.h"

// This switch enables long delay for clock(1us)
// Use it for very fast processors
// #define HX_711_CLK_DELAY

// The value defined is used for moving average filter depth
// in a retalion: buffer depth = 2^HX_711_ENABLE_FILTER.
#define HX_711_ENABLE_FILTER 3 // 3 makes depth of 8

// The threshold hysteresys
#define HX711_SW_HYSTERESIS 0.1f

// Endstop threshold [in grams if calibrated]
constexpr float HX711_ENDSTOP_THRESHOLD = 500.0f;
// Default calibration input known value if not specified
constexpr float HX711_DEFAULT_CALIBRATION_W = 300.0f;
// Default scale value
constexpr float HX711_DEFAULT_SCALE = 1.0f;
// Default ADC channel value
constexpr uint8_t HX711_DEFAULT_CHANNEL = 1u;

/**
 *   HX_711:   Class thet defines object for HX711 ADC functionalities and
 *             management.
 */
class HX_711
{

public:
    HX_711(const pin_t pinDta, const pin_t pinClk, const pin_t pinInd);

    void begin();

    /**
     * @brief The main task function.
     * 
     */
    void manage_hx_711();

    /**
     * @brief Calculate offset for the scale
     * 
     */
    void tare_start()
    {
        _read_counter = 0u;
        _averaged = 0.0f;
        _offset = 0.0f;
        _avg_start = true;
    };

    /**
     * @brief Calculate a scale(gain) value for a known weight.
     *        It is assumed that the "tare" is done before, and the weight is set in place.
     * 
     * @param avg_value - return value, a raw averaged value.
     * @return true 
     * @return false 
     */
    bool tare_ready(float &avg_value);

    /**
     * @brief Start averaging for the calibration.
     * 
     * @param known_value_in_grams - the weight used to calibrate the scale.
     */
    void calibrate_start(float known_value_in_grams = HX711_DEFAULT_CALIBRATION_W)
    {
        _read_counter = 0u;
        _averaged = 0.0f;
        _scale = 1.0f;
        _calib_weight = known_value_in_grams;
        _avg_start = true;
    };

    /**
     * @brief Start averaging for the tare.
     * 
     * @param avg_value - return value, a raw averaged value.
     * @return true 
     * @return false 
     */
    bool calibrate_ready(float &avg_value);

    int32_t read();
    
    /**
     * @brief Enable/disable the output pin write control. It enables/disables the scale gauge.
     * 
     * @param input_val 
     */
    void enable_out (bool input_val) { _enable = input_val; };

    bool read_enable_out () { return _enable; };


    /* SETTERS and GETTERS */

    /**
     * @brief Set the HX711 Threshold value that will impact status of indication pin.
     *
     * @param thrIn
     */
    inline void  setThreshold(float thrIn) { _th_weigth = thrIn; };
    inline float getThreshold(void) { return _th_weigth; };
    /**
     * @brief Set the HX711 channel/mode.
     *
     * @param chIn
     */
    inline void    setChannel(uint8_t chIn) { _channel = chIn; };
    inline uint8_t getChannel(void) { return _channel; };

    /**
     * @brief Set manually the scale/gain factor for the strain gauge (g/ADC_unit)
     * 
     * @param scaleIn 
     */
    inline void  setScale(float scaleIn) { _scale = scaleIn; };
    inline float getScale(void) { return _scale; };

    /**
     * @brief Get the HX711 filtered raw value
     *
     * @return float
     */
    inline float getCurrentVal() { return _f_val; };

    /**
     * @brief Get the Offset value
     * 
     * @return float 
     */
    inline float getOffset() { return _offset; };

    bool th_init = false;

private:
    // Operational variables
    float _scale = HX711_DEFAULT_SCALE;             // scale parameter
    float _th_weigth = HX711_ENDSTOP_THRESHOLD;     // threshold in grams if calibrated
    float _offset = 0.0f;                           // tare calculated offset
    uint8_t _channel = HX711_DEFAULT_CHANNEL;       // default: channel A; gain 128
    // Internal working variables
    float _averaged = 0.0f; // averaged value over period of time
    bool _enable = false;   // enable the output pin control
    bool _new_val;          // new value indicator
    float _f_val = 0.0f;    // filtered raw value
    int32_t _last_read = 0; // last read raw value (unfiltered)
    uint32_t _read_counter = 0u;
    bool _avg_start = false;
    float _calib_weight = HX711_DEFAULT_CALIBRATION_W;
    // pins variables
    pin_t _pin_dta; // data pin
    pin_t _pin_clk; // clock pin
    pin_t _pin_ind; // indicatior pin

    int32_t _threshold = 0; // scale treshold.

    uint8_t _shiftIn();
};

extern HX_711 wScale; // make global object available.

//  -- END OF FILE --
