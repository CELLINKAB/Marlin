#include "../../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#    include "analog_sensor.h"

using namespace pneumatics;

//
// constants
//

static constexpr float TANK_GAIN = 0.183239119;
static constexpr float REGULATOR_GAIN = 0.04;
static constexpr float VACUUM_GAIN = -0.02;

static constexpr float TANK_OFFSET = 150.0f;
static constexpr float VACUUM_OFFSET = -16.5f;
static constexpr float REGULATOR_OFFSET = 0.0f;

//
// implementations
//

AnalogPressureSensor::AnalogPressureSensor(pin_t sense_pin, float scale_factor, float offset_kPa)
    : scalar(scale_factor)
    , offset(offset_kPa)
    , pin(sense_pin)
    , avg_raw(0)
{
    pinMode(sense_pin, INPUT_ANALOG);
}

//
// statics
//

AnalogPressureSensor pneumatics::gripper_vacuum(GRIPPER_VACUUM_PIN, VACUUM_GAIN, VACUUM_OFFSET);
AnalogPressureSensor pneumatics::tank_pressure(PRESSURE_TANK_PIN, TANK_GAIN, TANK_OFFSET);
AnalogPressureSensor pneumatics::regulator_feedback(PRESSURE_REGULATOR_SENSE_PIN, REGULATOR_GAIN, REGULATOR_OFFSET);

#endif // FESTO_PNEUMATICS