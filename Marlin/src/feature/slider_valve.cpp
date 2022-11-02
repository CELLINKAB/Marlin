// Copyright Cellink 2022 - GPL-v3

#include "../gcode/gcode.h"
#include "../module/planner.h"
#include "../module/servo.h"

#if ENABLED(SLIDER_VALVE)

#include "extruder_bottomout.h"

#define SLIDER_SERVO servo[0]

namespace cartridge {

enum class SliderPosition : uint8_t {
    Extrude = 0,
    MaterialMix = 1,
    CellMix = 2,
    NeutralizerMix = 3,
};

constexpr int slider_map(SliderPosition position)
{
    switch (position) {
    case SliderPosition::Extrude:
        return 0;
    case SliderPosition::MaterialMix:
        return 36;
    case SliderPosition::CellMix:
        return 52;
    case SliderPosition::NeutralizerMix:
        return 66;
    }

    return 0; // unreachable
}

static SliderPosition current_slider_pos = SliderPosition::Extrude;

} // namespace cartridge

void GcodeSuite::M1112()
{
    static bool homed [[maybe_unused]] = []() {
        bottomout_extruder(E1_STOP_PIN);
        return true;
    }();

    active_extruder = 1;

    if (parser.seen('H')) {
        bottomout_extruder(E1_STOP_PIN);
        return;
    }

    if (parser.seen('A')) {
        const auto absolute_position = parser.value_float();
        planner.synchronize();
        auto pos = current_position.copy();
        pos.e = absolute_position;
        planner.buffer_line(pos, 1.0);
        planner.synchronize();
        return;
    }

    const auto slider_position = parser.intval('P', -1);
    if (slider_position < 0 || slider_position > 3) {
        SERIAL_ECHO_MSG("P value must be between 0 and 3");
        return;
    }
}

#endif // SLIDER_VALVE