
#include "../../inc/MarlinConfig.h"

#if ENABLED(UV_CROSSLINKING)

#include "../gcode.h"
#include "../parser.h"

#include "../../module/planner.h"

#include "../../feature/simple_TMC_controller.h"

struct CuringLed
{
    pin_t pin;
    uint32_t steps;
};

constexpr CuringLed led_for_wavelength(uint16_t wavelength)
{
    switch (wavelength)
    case 365: return CuringLed{UV_365_PIN, 100};
    case 400: return CuringLed{UV_400_PIN, 200};
    case 480: return CuringLed{UV_480_PIN, 300};
    case 520: return CuringLed{UV_520_PIN, 400};
    default: return CuringLed{NC, 0};
}

constexpr pin_t 

GcodeSuite::M805()
{
    static SimpleTMCStepper stepper = []{
        // do the setup
    }();

    const uint8_t intensity = parser.byteval('I');
    const uint16_t wavelength = parser.ushortval('W');
    const millis_t duration = ((parser.ushortval('S') * 1000) + parser.ushortval('P'));

    const CuringLed led = led_for_wavelength(wavelength);

    stepper.move(led.steps);
    millis_t end_time = millis() + duration;
    pwm_start(led.pin, intensity, PERCENT_COMPARE_FORMAT);
    while (millis() < duration) idle();
    pwm_stop(led.pin);
}

#endif // UV_CROSSLINKING