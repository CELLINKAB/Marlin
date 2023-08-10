#include "../inc/MarlinConfig.h"

#if PIN_EXISTS(DOOR)

  #include "door_sensor.h"

  constexpr millis_t DOOR_DEBOUNCE_PERIOD = 50;
  Debounced<DOOR_PIN> door(DOOR_DEBOUNCE_PERIOD, DOOR_SENSOR_INVERTING, false);

#endif