
#include "../../inc/MarlinConfig.h"

#if ENABLED(EXOCYTE_UV_CROSSLINKING)

#    include "../../feature/simple_TMC_controller.h"
#    include "../../module/planner.h"
#    include "../gcode.h"
#    include "../parser.h"

struct CuringLed
{
    pin_t pin;
    float deg;
};

constexpr static uint8_t PC_MICROSTEPS = 32;
constexpr static float PC_DEG_PER_STEP = 1.8f;
constexpr static uint32_t PC_VELOCITY = 1000;

constexpr int32_t deg_to_steps(float degs)
{
    return static_cast<int32_t>((degs / PC_DEG_PER_STEP) * PC_MICROSTEPS);
}

constexpr static float PC_365_DEG = 0.0f;
constexpr static float PC_400_DEG = PC_365_DEG + 15.0f;
constexpr static float PC_480_DEG = PC_400_DEG + 15.0f;
constexpr static float PC_520_DEG = PC_480_DEG + 15.0f;

static constexpr pin_t BAD_PIN = static_cast<pin_t>(NC);

constexpr CuringLed led_for_wavelength(uint16_t wavelength)
{
    switch (wavelength) {
    case 400:
        [[fallthrough]];
    case 405:
        return CuringLed{PC_400_PIN, PC_400_DEG};
    case 480:
        [[fallthrough]];
    case 485:
        return CuringLed{PC_480_PIN, PC_480_DEG};
    case 520:
        return CuringLed{PC_520_PIN, PC_520_DEG};
    case 360:
        [[fallthrough]];
    case 365:
        return CuringLed{PC_365_PIN, PC_365_DEG};
    default:
        return CuringLed{BAD_PIN, 0.0f};
    }
}

static constexpr pin_t PC_STOP_PIN = PC_ENDSTOP_PIN;
using Stepper = SimpleTMC<PC_ENABLE_PIN, PC_STOP_PIN, PC_STEP_PIN, PC_DIR_PIN>;

inline void move_degs(Stepper& stepper, float degs)
{
    int32_t steps = -deg_to_steps(degs);
    stepper.move_steps(steps, PC_VELOCITY);
}

void home_rainbow(Stepper& stepper)
{
    stepper.raw_move(-PC_VELOCITY);
    while (READ(PC_STOP_PIN) == LOW)
        safe_delay(0);
    stepper.stop();
}

void move_rainbow(Stepper& stepper, CuringLed led)
{
    home_rainbow(stepper);
    move_degs(stepper, led.deg);
}

void GcodeSuite::M805()
{
    static auto stepper = [] {
        OUT_WRITE(PC_365_PIN, LOW);
        OUT_WRITE(PC_400_PIN, LOW);
        OUT_WRITE(PC_480_PIN, LOW);
        OUT_WRITE(PC_520_PIN, LOW);
        pinMode(PC_PWM_PIN, PWM);

        return Stepper(SimpleTMCConfig(PC_SLAVE_ADDRESS, 50, 800, 0.15f),
                       PC_SERIAL_RX_PIN,
                       PC_SERIAL_TX_PIN);
    }();
    const uint8_t intensity = parser.byteval('I');
    const uint16_t wavelength = parser.ushortval('W');
    const millis_t duration = (SEC_TO_MS(parser.ulongval('S')) + parser.ushortval('P'));

    const CuringLed led = led_for_wavelength(wavelength);

    if (led.pin == BAD_PIN) {
        SERIAL_ERROR_MSG("bad wavelength argument! \n Must be one of 365,400,480, or 520\n got:",
                         wavelength);
        return;
    }

    stepper.set_hold(true);
    if (parser.seenval('D'))
        move_rainbow(stepper, CuringLed{led.pin, parser.value_float()});
    else
        move_rainbow(stepper, led);
    WRITE(led.pin, HIGH);
    analogWrite(PC_PWM_PIN, intensity);
    safe_delay(duration);
    analogWrite(PC_PWM_PIN, 0);
    WRITE(led.pin, LOW);
    stepper.set_hold(false);
    stepper.stop();
}

#endif // EXOCYTE_UV_CROSSLINKING