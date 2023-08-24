
#include "../inc/MarlinConfig.h"
#if ENABLED(CELLINK_REPORTING)

#    include "../module/endstops.h"
#    include "../module/planner.h"

#    include "cellink_reporter.h"

using namespace cellink;

void Reporter::M119::report()
{
    endstops.report_states();
}
void Reporter::M814::report() {}
void Reporter::M816::report() {}
void Reporter::M821::report() {}
void Reporter::M825::report() {}
void Reporter::M1015::report()
{
    const auto pos = current_position.asLogical();
    serial_echoln_kv("XPOS", pos.x, "YPOS", pos.y, "ZPOS", pos.z);
}
void Reporter::M1016::report()
{
    const auto pos = planner.get_axis_positions_mm();
    serial_echoln_kv("XMPOS", pos.x, "YMPOS", pos.y, "ZMPOS", pos.z);
}

void Reporter::tick_all()
{
    m119.tick();
    m772.tick();
    m798.tick();
    m799.tick();
    m802.tick();
    m821.tick();
    m825.tick();
    m1015.tick();
    m1016.tick();
}

Reporter cellink::reporter;
#endif