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
//#define HX_711_CLK_DELAY

// The value defined is used for moving average filter depth
// in a retalion: buffer depth = 2^HX_711_ENABLE_FILTER.
#define HX_711_ENABLE_FILTER        3

// The threshold hysteresys
#define HX711_SW_HYSTERESIS         3000.0f

// Endstop threshold
#define HX711_ENDSTOP_THRESHOLD     10000.0f

/**
 *   HX_711:   Class thet defines object for HX711 ADC functionalities and
 *             management.
 */
class HX_711
{

public:
  
  HX_711();

  ~HX_711();

  void  init();

  void  begin(const uint8_t pinDta, const uint8_t pinClk, const uint8_t pinInd);

  void  manage_hx_711();

  void  tare_start() { _tare_start = true; _read_counter = 0u; _averaged = 0; };

  bool  tare_ready( float &avg_value );
  bool  tare_ready() { return !_tare_start;};

  int32_t read();

  /**
   * @brief Set the HX711 Threshold value that will impact status of indication pin.
   * 
   * @param thrIn 
   */
  void setThreshold( int32_t thrIn ) { _th_weigth = (float)thrIn; };

  /**
   * @brief Set the HX711 channel/mode.
   * 
   * @param chIn 
   */
  void setChannel( uint8_t chIn ) { _channel = chIn; };

  /**
   * @brief Get the HX711 filtered raw value
   * 
   * @return float 
   */
  float getCurrentVal() { return _f_val; };

  bool      th_init      = false;

private:

  bool      _new_val        = false;    // new value indicator
  float     _f_val          = 0.0f;     // filtered raw value
  int32_t   _last_read      = 0;        // last read raw value (unfiltered)
  float     _averaged       = 0.0f;     // tare averaged
  uint32_t  _read_counter   = 0u;
  bool      _tare_start     = false;
  float     _th_weigth      = HX711_ENDSTOP_THRESHOLD;

  uint8_t   _pin_dta;                   // data pin
  uint8_t   _pin_clk;                   // clock pin
  uint8_t   _pin_ind;                   // indicatior pin

  uint8_t   _channel        = 1;        // default: channel A; gain 128
  int32_t   _threshold      = 0;        // scale treshold.

  uint8_t  _shiftIn();
};

extern HX_711 wScale;                 // make global object available.

//  -- END OF FILE --

