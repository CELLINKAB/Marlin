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

void pneumatic_assisted_move(abce_pos_t pos, feedRate_t feedrate)
{
    planner.synchronize();
    WRITE(PRESSURE_VALVE_1_PIN, HIGH);
    WRITE(PRESSURE_VALVE_2_PIN, LOW);
    delay(100);
    planner.buffer_segment(pos, feedrate);
    planner.synchronize();
    WRITE(PRESSURE_VALVE_1_PIN, LOW);
    WRITE(PRESSURE_VALVE_2_PIN, HIGH);
}

void GcodeSuite::M1113()
{
    // if (cartridge::current_slider_pos == cartridge::SliderPosition::Extrude) {
    //     SERIAL_ERROR_MSG("Slider valve shouldn't be in Extrude position for mixing!");
    //     return;
    // }

    const auto volume = parser.axisunitsval('E', AxisEnum::E_AXIS);
    const auto feedrate = parser.feedrateval('F');

    const auto pos = current_position.copy();
    const auto retract_pos = pos - abce_pos_t{0, 0, 0, volume};

    for (auto cycles = parser.ushortval('P'); cycles > 0; --cycles) {
        pneumatic_assisted_move(retract_pos, feedrate);
        planner.buffer_segment(pos, feedrate);
    }

    if (parser.seen('L')) {
        pneumatic_assisted_move(retract_pos, feedrate);
    }

    sync_plan_position_e();
}

#endif // SLIDER_VALVE