/*******************************************************************************
FILE
    hx_711.cpp

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
#include "../inc/MarlinConfigPre.h"

#if ENABLED(HX711_WSCALE)

#include "hx_711.h"

HX_711 wScale;

HX_711::HX_711()
{
  init();
}

HX_711::~HX_711() {}

/**
 * @brief Assigns and initiate physical pin states for HX711 communication.
 * 
 * @param pinDta Data in pin
 * @param pinClk Clock out pin
 * @param pinInd Indication out pin
 */
void HX_711::begin(const uint8_t pinDta, const uint8_t pinClk, const uint8_t pinInd)
{
  _pin_dta = pinDta;
  _pin_clk = pinClk;
  _pin_ind = pinInd;
  // If there is no pull-up on PIN, external must be added!
  pinMode(_pin_dta, INPUT_PULLUP);  //must have pull-up!
  pinMode(_pin_clk, OUTPUT);
  pinMode(_pin_ind, INPUT_PULLUP); //this is output pin, but for the safety reasons set to 1 with pullup
  digitalWrite(_pin_clk, LOW);
}

/**
 * @brief Initializes internal variables.
 * 
 */
void HX_711::init()
{
  _new_val  = false;
  _channel  = 1;
}

/**
 * @brief Non blocking READ function that will update _new_val status and 
 *        return 32bit signed value that is read from ADC as 24bit val. 
 *        If data is not ready it will return 0, and update _new_val to false.
 * 
 * @return int32_t 
 */
int32_t HX_711::read()
{
  union
  {
    int32_t value = 0;
    uint8_t data[4];
  } v;
//  When output data is not ready for retrieval,
//       digital output pin DOUT is HIGH.
  if(digitalRead(_pin_dta) == LOW)
  {
//    noInterrupts();
//  When DOUT goes to LOW, it indicates data is ready for retrieval.

    //  Pulse the clock pin 24 times to read the data.
    //  v.data[2] = shiftIn(_dataPin, _clockPin, MSBFIRST);
    //  v.data[1] = shiftIn(_dataPin, _clockPin, MSBFIRST);
    //  v.data[0] = shiftIn(_dataPin, _clockPin, MSBFIRST);
    v.data[2] = _shiftIn();
    v.data[1] = _shiftIn();
    v.data[0] = _shiftIn();

    //  CLOCK      CHANNEL      GAIN      m
    //  ------------------------------------
    //   25           A         128       1    //  default
    //   26           B          32       2
    //   27           A          64       3

    uint8_t m = _channel;

    while (m > 0)
    {
      //  delayMicroSeconds(1) needed for fast processors?
      digitalWrite(_pin_clk, HIGH);
      digitalWrite(_pin_clk, LOW);
      m--;
    }

//    interrupts();

    //  Sign extending
    if (v.data[2] & 0x80) v.data[3] = 0xFF;
    
    _new_val = true;
  }else{
    _new_val = false;
    v.value = 0;
  }
  return v.value;
}

/**
 * @brief This function is non blocking management function for
 *        HX771 base scale wight. This function can be called in 
 *        management loop. It manages all functionality related 
 *        to weight scale.
 */ 
void HX_711::manage_hx_711()
{
  _last_read = read();          //shift in data from ADC

  if(_new_val)                  //if there are data available, process it.
  {
    #ifdef HX_711_ENABLE_FILTER
      #define FILTER_DEPTH (uint8_t)(1u << HX_711_ENABLE_FILTER)
      #define FILTER_MASK  (uint8_t)( FILTER_DEPTH - 1u )

      static uint8_t cnt_fil_buff = 0;
      static int32_t fbuff[FILTER_DEPTH] = {};

      fbuff[cnt_fil_buff] = _last_read;

      cnt_fil_buff = (cnt_fil_buff + 1u) & FILTER_MASK;

      float f_summ = 0;
      for(uint8_t i = 0; i< FILTER_DEPTH; i++) f_summ += (float)fbuff[i];
      
      _f_val = f_summ / (float)FILTER_DEPTH;
    #else
      _f_val = (float)_last_read;
    #endif

  if(_f_val > _threshold)      //if threshold reached set-up output pin.
  { 
    // Drive the pin low that is logic 0
    pinMode(_pin_ind, OUTPUT);
    digitalWrite(_pin_ind, LOW);
  }
  else{
    // Set pin to high impedance with pullup that is logic 1
    pinMode(_pin_ind, INPUT_PULLUP);
  }

  }
}
/**
 * @brief Shift In 8 bits function MSB_FIRST
 * 
 * @return uint8_t 
 */
uint8_t HX_711::_shiftIn()
{
  uint8_t shiftValue = 0;
  uint8_t mask  = 0x80;

  while (mask > 0)
  {
    digitalWrite(_pin_clk, HIGH);
    #ifdef HX_711_CLK_DELAY
      delayMicroseconds(1);   //  T2  >= 0.2 us
    #else    
      digitalWrite(_pin_clk, HIGH);    //using as delay before reading the data
      digitalWrite(_pin_clk, HIGH);
    #endif
    if (digitalRead(_pin_dta) == HIGH)
    {
      shiftValue |= mask;
    }
    digitalWrite(_pin_clk, LOW);
    #ifdef HX_711_CLK_DELAY
      delayMicroseconds(1);   //  keep duty cycle ~50%
    #else
      digitalWrite(_pin_clk, LOW);
      digitalWrite(_pin_clk, LOW);
    #endif
    mask >>= 1;
  }
  return shiftValue;
}

#endif

// -- END OF FILE --

