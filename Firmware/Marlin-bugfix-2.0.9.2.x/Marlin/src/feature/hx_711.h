#pragma once
//
//    FILE: hx_711.h
//  AUTHOR: Borislav Cvejic
// VERSION: 0.0.1
// PURPOSE: Basic library for HX711 with non blocking communication
//
//
// HISTORY:
//
//  NOTES
//  

#include "Arduino.h"
#include "inttypes.h"

#define hx_711_LIB_VERSION               (F("0.0.1"))

// This switch enables long delay for clock(1us)
// Use it for very fast processors
//#define HX_711_CLK_DELAY

// The value defined is used for moving average filter depth
// in a retalion: buffer depth = 2^HX_711_ENABLE_FILTER.
#define HX_711_ENABLE_FILTER  3


class HX_711
{

public:
  
  HX_711();

  ~HX_711();

  void  init();

  void begin(const uint8_t pinDta, const uint8_t pinClk, const uint8_t pinInd);

  void  manage_hx_711();

  float get_val() { return _f_val; };

  int32_t read();

private:

  bool      _new_val;
  float     _f_val        = 0.0f;
  int32_t   _last_read;

  uint8_t   _pin_dta;
  uint8_t   _pin_clk;
  uint8_t   _pin_ind;

  uint8_t   _gain         = 128;     // default channel A gain 128
  long      _offset       = 0;
  uint8_t   _channel      = 1;       // default channel A gain 128
  uint8_t   _mode         = 0;

  uint8_t  _shiftIn();
};

//extern HX_711 test_scale;
//  -- END OF FILE --

