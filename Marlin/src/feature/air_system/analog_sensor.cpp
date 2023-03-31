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
// statics
//

AnalogPressureSensor<GRIPPER_VACUUM_PIN> pneumatics::gripper_vacuum(VACUUM_GAIN, VACUUM_OFFSET);
AnalogPressureSensor<PRESSURE_TANK_PIN> pneumatics::tank_pressure(TANK_GAIN, TANK_OFFSET);
AnalogPressureSensor<PRESSURE_REGULATOR_SENSE_PIN> pneumatics::regulator_feedback(REGULATOR_GAIN, REGULATOR_OFFSET);

#endif // FESTO_PNEUMATICS