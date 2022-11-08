
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
        return CuringLed{PC_400_PIN, 200};
    case 480:
        return CuringLed{PC_480_PIN, 300};
    case 520:
        return CuringLed{PC_520_PIN, 400};
    case 365:
        return CuringLed{PC_365_PIN, 100};
    default:
        return CuringLed{static_cast<pin_t>(NC), 0};
    }
}

void single_step()
{
    static uint32_t idle_counter = 0;
    WRITE(PC_STEP_PIN, HIGH);
    delayMicroseconds(1);
    WRITE(PC_STEP_PIN, LOW);
    delayMicroseconds(1);
    idle_counter += 1;
    if (idle_counter >= 10000) {
        idle();
        idle_counter = 0;
    }

}

void move_rainbow(CuringLed led)
{
    static constexpr uint8_t stop_dir = LOW;
    static uint32_t rainbow_position = []{
        OUT_WRITE(PC_DIR_PIN, stop_dir);
        OUT_WRITE(PC_STEP_PIN, LOW);
        uint32_t max_steps = 100'000;
        while ( max_steps-- && READ(PC_STOP_PIN) == LOW) {
            single_step();
        }
        return 0;
    }();
    if (led.steps == rainbow_position) return;
    //if (led.steps < rainbow_position)
}

void GcodeSuite::M805()
{
    static auto stepper = [] {
        using TMC = SimpleTMC<PC_ENABLE_PIN, PC_STOP_PIN>;
        SimpleTMCConfig config(PC_SLAVE_ADDRESS, 100, 250);
        return TMC::init(config);
    }();

    const uint8_t intensity = parser.byteval('I');
    const uint16_t wavelength = parser.ushortval('W');
    const millis_t duration = ((parser.ushortval('S') * 1000) + parser.ushortval('P'));

    const CuringLed led = led_for_wavelength(wavelength);

    // move rainbow to correct position for selected LED
    millis_t end_time = millis() + duration;
    analogWrite(led.pin, intensity);
    while (millis() < duration)
        idle();
    analogWrite(led.pin, 0);
}

#endif // EXOCYTE_UV_CROSSLINKING