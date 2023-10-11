/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 Cellink [https://github.com/CELLINKAB/Marlin]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */


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

constexpr static uint8_t PC_MICROSTEPS = 4;
constexpr static uint32_t PC_RMS_CURRENT = 800;
constexpr static float PC_DEG_PER_STEP = 1.8f;
constexpr static uint32_t PC_VELOCITY = 500;

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

struct Rainbow
{
    static constexpr pin_t PC_STOP_PIN = PC_ENDSTOP_PIN;
    using Stepper = SimpleTMC<PC_ENABLE_PIN, PC_STOP_PIN, PC_STEP_PIN, PC_DIR_PIN>;

    Stepper& stepper;
    uint32_t velocity;

    constexpr explicit Rainbow(Stepper& stepper_)
        : stepper(stepper_)
        , velocity(PC_VELOCITY)
    {}

    void move_degs(float degs)
    {
        int32_t steps = -deg_to_steps(degs);
        stepper.move_steps(steps, velocity);
    }

    void home()
    {
        stepper.raw_move(-velocity);
        millis_t timeout = millis() + 1000;
        while (READ(PC_STOP_PIN) == LOW && millis() < timeout)
            idle();
        stepper.stop();
    }

    void move(CuringLed led)
    {
        home();
        move_degs(led.deg);
    }

    void set_hold(bool hold)
    {
        stepper.set_hold(hold);
        if (hold == false)
            stepper.stop();
    }
};

void GcodeSuite::M805()
{
    static auto rainbow = [] {
        OUT_WRITE(PC_365_PIN, LOW);
        OUT_WRITE(PC_400_PIN, LOW);
        OUT_WRITE(PC_480_PIN, LOW);
        OUT_WRITE(PC_520_PIN, LOW);
        pinMode(PC_PWM_PIN, PWM);
        constexpr auto PC_FREQ = TERN(PC_PWM_FREQUENCY, PC_PWM_FREQUENCY, 20'000);
        hal.set_pwm_frequency(PC_PWM_PIN, PC_FREQ);

        static auto st = Rainbow::Stepper(SimpleTMCConfig(PC_SLAVE_ADDRESS, 50, PC_RMS_CURRENT, 0.15f),
                                          PC_SERIAL_RX_PIN,
                                          PC_SERIAL_TX_PIN);
        auto driver = st.get_driver();
        driver.microsteps(PC_MICROSTEPS);
        driver.ihold(31); // hold current == run current for maximum torque
        return Rainbow(st);
    }();
    const uint8_t intensity = parser.byteval('I');
    const uint16_t wavelength = parser.ushortval('W');
    const millis_t duration = (SEC_TO_MS(parser.ulongval('S')) + parser.ushortval('P'));

    if (parser.seenval('F')) {
        const feedRate_t feedrate_deg_s = parser.value_feedrate();
        const uint32_t step_velocity = deg_to_steps(feedrate_deg_s);
        rainbow.velocity = step_velocity;
    }

    const CuringLed led = led_for_wavelength(wavelength);

    if (led.pin == BAD_PIN) {
        SERIAL_ERROR_MSG("bad wavelength argument! \n Must be one of 365,400,480, or 520\n got:",
                         wavelength);
        return;
    }

    planner.synchronize();
    planner.quick_pause();

    rainbow.set_hold(true);
    if (parser.seenval('D'))
        rainbow.move(CuringLed{led.pin, parser.value_float()});
    else
        rainbow.move(led);
    WRITE(led.pin, HIGH);
    analogWrite(PC_PWM_PIN, intensity);
    safe_delay(duration);
    analogWrite(PC_PWM_PIN, 0);
    WRITE(led.pin, LOW);
    rainbow.set_hold(false);

    planner.quick_resume();
}

#endif // EXOCYTE_UV_CROSSLINKING