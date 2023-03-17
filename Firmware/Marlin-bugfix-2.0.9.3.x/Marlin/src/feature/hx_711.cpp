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

HX_711 wScale(HX711_DATA_PIN, HX711_CLCK_PIN, HX711_INDICATION_PIN);

HX_711::HX_711(const pin_t pinDta, const pin_t pinClk, const pin_t pinInd) : _new_val(false), _channel(1), _pin_dta(pinDta), _pin_clk(pinClk), _pin_ind(pinInd) {}

/**
 * @brief Assigns and initiate physical pin states for HX711 communication.
 *
 * @param pinDta Data in pin
 * @param pinClk Clock out pin
 * @param pinInd Indication out pin
 */
void HX_711::begin()
{
  // If there is no pull-up on PIN, external must be added!
  SET_INPUT_PULLUP(_pin_dta); // must have pull-up!
  OUT_WRITE(_pin_clk, LOW);
  SET_INPUT_PULLUP(_pin_ind); // this is output pin, but for the safety reasons set to 1 with pullup
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
  uint8_t data[4]{};
  int32_t value = 0;
  //  When output data is not ready for retrieval,
  //       digital output pin DOUT is HIGH.
  if (READ(_pin_dta) == LOW)
  {
    noInterrupts();
    //  When DOUT goes to LOW, it indicates data is ready for retrieval.

    //  Pulse the clock pin 24 times to read the data.
    //  v.data[2] = shiftIn(_dataPin, _clockPin, MSBFIRST);
    //  v.data[1] = shiftIn(_dataPin, _clockPin, MSBFIRST);
    //  v.data[0] = shiftIn(_dataPin, _clockPin, MSBFIRST);
    data[2] = _shiftIn();
    data[1] = _shiftIn();
    data[0] = _shiftIn();

    //  CLOCK      CHANNEL      GAIN      m
    //  ------------------------------------
    //   25           A         128       1    //  default
    //   26           B          32       2
    //   27           A          64       3

    uint8_t m = _channel;

    while (m > 0)
    {
      delayMicroseconds(1);
      WRITE(_pin_clk, HIGH);
      delayMicroseconds(1);
      WRITE(_pin_clk, LOW);
      --m;
    }

    interrupts();

    //  Sign extending
    if (data[2] & 0x80)
      data[3] = 0xFF;

    // safely transmute bytes into value
    memcpy(&value, data, sizeof(value));

    _new_val = true;
  }
  else
  {
    _new_val = false;
  }
  return value;
}

/**
 * @brief This function is non blocking management function for
 *        HX771 base scale wight. This function can be called in
 *        management loop. It manages all functionality related
 *        to weight scale.
 */
void HX_711::manage_hx_711()
{
  _last_read = read(); // shift in data from ADC

  if (_new_val) // if there are data available, process it.
  {

// MOVING AVERAGE MANAGEMENT
#ifdef HX_711_ENABLE_FILTER
    constexpr static uint8_t FILTER_DEPTH = 1u << HX_711_ENABLE_FILTER;
    constexpr static uint8_t FILTER_MASK = FILTER_DEPTH - 1u;

    static uint8_t cnt_fil_buff = 0;
    static int32_t fbuff[FILTER_DEPTH] = {};

    fbuff[cnt_fil_buff] = _last_read;

    cnt_fil_buff = (cnt_fil_buff + 1u) & FILTER_MASK;

    float f_summ = 0;
    for (uint8_t i = 0; i < FILTER_DEPTH; i++)
      f_summ += (float)fbuff[i];
    _f_val = f_summ / (float)FILTER_DEPTH;

#else
    _f_val = (float)_last_read;
#endif

    // THRESHOLD HYSTERESIS MANAGEMENT
    static bool ind_pin_state = true;
#ifndef HX711_SW_HYSTERESIS
  #define HX711_SW_HYSTERESIS 0.0f
#endif
    float th_hysteresis = _th_weigth * HX711_SW_HYSTERESIS;
    if (ind_pin_state)
    {
            // if threshold reached set-up output pin.
      if (abs(_f_val - _averaged) > (_th_weigth + th_hysteresis))
        ind_pin_state = false;
    } else {
            // if threshold reached set-up output pin.
      if (abs(_f_val - _averaged) < (_th_weigth - th_hysteresis))
        ind_pin_state = true;
    }

    // PIN OPERATION MANAGEMENT
    if (!ind_pin_state) // set-up output pin.
    {
      // Drive the pin low that is logic 0
      OUT_WRITE(_pin_ind, LOW);
    }
    else
    {
      // Set pin to high impedance with pullup that is logic 1
      SET_INPUT_PULLUP(_pin_ind);
    }

    // TARE MANAGEMENT
    constexpr static float TARE_AVG_CNT = 200.0f;

    if (_tare_start == true)
    {
      _averaged += _f_val;
      if (_read_counter >= TARE_AVG_CNT)
      {
        _averaged = _averaged / TARE_AVG_CNT;
        _tare_start = false;
      }
    }

    _read_counter++;
  }
}

bool HX_711::tare_ready(float &avg_value)
{
  if (_tare_start == false)
  {
    avg_value = _averaged;
    return true;
  }
  else
  {
    return false;
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
  uint8_t mask = 0x80;

  while (mask > 0)
  {
    WRITE(_pin_clk, HIGH);
    delayMicroseconds(1); //  T2  >= 0.2 us

    if (READ(_pin_dta) == HIGH)
    {
      shiftValue |= mask;
    }

    WRITE(_pin_clk, LOW);
    delayMicroseconds(1); //  keep duty cycle ~50%

    mask >>= 1;
  }
  return shiftValue;
}

#endif

// -- END OF FILE --
