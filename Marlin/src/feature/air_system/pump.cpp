#include "../../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#include "pump.h"


//
// statics
//

pneumatics::Pump<PRESSURE_PUMP_EN_PIN, PRESSURE_VALVE_PUMP_OUT_PIN, PRESSURE_TANK_PIN> pneumatics::pump(pneumatics::tank_pressure);

#endif  // FESTO_PNEUMATICS