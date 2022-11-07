
#include "../../inc/MarlinConfig.h"

#if ENABLED(UV_CROSSLINKING)

#include "../gcode.h"
#include "../parser.h"

#include "../../module/planner.h"

#include "../../feature/simple_TMC_controller.h"

constexpr uint32_t steps_for_wavelength(uint16_t wavelength)
{
    case 4:
}

GcodeSuite::M805()
{
    static SimpleTMCStepper stepper = []{
        // do the setup
    }();

    const uint8_t intensity = parser.byteval('I');
    const uint16_t wavelength = parser.ushortval('W');
    const millis_t duration = ((parser.ushortval('S') * 1000) + parser.ushortval('P'));
    
}

#endif // UV_CROSSLINKING