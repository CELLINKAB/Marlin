// Copyright Cellink 2022 - GPL-v3

#include "../gcode/gcode.h"
#include "../module/planner.h"
#include "../module/servo.h"

#define SLIDER_SERVO servo[0]

namespace cartridge {

enum class SliderPosition : uint8_t
{
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
    const auto slider_position = parser.intval('P', -1);
    if (slider_position < 0 || slider_position > 3) {
        SERIAL_ECHO_MSG("P value must be between 0 and 3");
        return;
    }
    const auto enum_slider_val = static_cast<cartridge::SliderPosition>(slider_position);
    SLIDER_SERVO.move(cartridge::slider_map(enum_slider_val));
    cartridge::current_slider_pos = enum_slider_val;
}

void GcodeSuite::M1113()
{
    if (cartridge::current_slider_pos == cartridge::SliderPosition::Extrude) {
        SERIAL_ERROR_MSG("Slider valve shouldn't be in Extrude position for mixing!");
        return;
    }

    const auto volume = parser.axisunitsval('E', AxisEnum::E_AXIS);
    const auto feedrate = parser.feedrateval('F');

    planner.synchronize();
    const auto pos = current_position.copy();
    const auto retract_pos = pos - abce_pos_t{0, 0, 0, volume};

    for (auto cycles = parser.ushortval('P'); cycles > 0; --cycles) {
        planner.buffer_segment(retract_pos, feedrate);
        planner.buffer_segment(pos, feedrate);
    }

    if (parser.seen('L'))
        planner.buffer_segment(retract_pos, feedrate);

    planner.synchronize();
    sync_plan_position_e();
}