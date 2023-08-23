#include "../../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#include "regulator.h"

//
// statics
//

pneumatics::Regulator<PRESSURE_REGULATOR_PIN> pneumatics::regulator;

#endif  // FESTO_PNEUMATICS