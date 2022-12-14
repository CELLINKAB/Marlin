
#include "../../inc/MarlinConfig.h"

#if ENABLED(EXOCYTE_UV_CROSSLINKING)

#    include "../../feature/simple_TMC_controller.h"
#    include "../../module/planner.h"
#    include "../gcode.h"
#    include "../parser.h"

struct CuringLed
{
    pin_t pin;
    uint32_t steps;
};

constexpr CuringLed led_for_wavelength(uint16_t wavelength)
{
    switch (wavelength) {
    case 400:
        [[fallthrough]];
    case 405:
        return CuringLed{PC_400_PIN, 200};
    case 480:
        [[fallthrough]];
    case 485:
        return CuringLed{PC_480_PIN, 300};
    case 520:
        return CuringLed{PC_520_PIN, 400};
    case 360:
        [[fallthrough]];
    case 365:
        return CuringLed{PC_365_PIN, 100};
    default:
        return CuringLed{static_cast<pin_t>(NC), 0};
    }
}

using Stepper = SimpleTMC<PC_ENABLE_PIN, PC_STOP_PIN, PC_STEP_PIN, PC_DIR_PIN>;

void move_rainbow(Stepper& stepper, CuringLed led)
{
    static int32_t rainbow_position = [&]() {
        stepper.move_until_stall(100);
        return 0;
    }();
    if (led.steps == rainbow_position)
        return;
    stepper.move_steps(led.steps - rainbow_position, 200, [](){return READ(PC_STOP_PIN);});

    // TODO: some error checking here
}

void GcodeSuite::M805()
{
    static auto stepper = [] {
        OUT_WRITE(PC_365_PIN, LOW);
        OUT_WRITE(PC_400_PIN, LOW);
        OUT_WRITE(PC_480_PIN, LOW);
        OUT_WRITE(PC_520_PIN, LOW);
        pinMode(PC_PWM_PIN, PWM);

        return Stepper(SimpleTMCConfig(PC_SLAVE_ADDRESS, 100, 400, 0.15f), PC_SERIAL_RX_PIN, PC_SERIAL_TX_PIN);
    }();
    const uint8_t intensity = parser.byteval('I');
    const uint16_t wavelength = parser.ushortval('W');
    const millis_t duration = ((parser.ulongval('S') * 1000) + parser.ushortval('P'));

    const CuringLed led = led_for_wavelength(wavelength);

    if (led.pin == -1)
        SERIAL_ERROR_MSG("bad wavelength argument! \n Must be one of 365,400,480, or 520\n got:",
                         wavelength);

    move_rainbow(stepper, led);
    const millis_t end_time = millis() + duration;
    WRITE(led.pin, HIGH);
    analogWrite(PC_PWM_PIN, intensity);
    do {
        idle();
        delay(100);
    } while (static_cast<int32_t>(end_time - millis()) > 200);
    if (millis_t now = millis(); end_time > now)
        delay(end_time - now);
    analogWrite(PC_PWM_PIN, 0);
    WRITE(led.pin, LOW);
}

#endif // EXOCYTE_UV_CROSSLINKING