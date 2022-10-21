
#include "../../inc/MarlinConfig.h"

#include "../gcode.h"

#define TEM_INA_PIN PD14
#define TEM_INB_PIN PD15
#define TEM_EN_PIN PF12
#define TEM_PWM_PIN PB3

void GcodeSuite::M1115()
{
    static bool init [[maybe_unused]] = []{
        OUT_WRITE(TEM_INA_PIN, HIGH);
        OUT_WRITE(TEM_INB_PIN, LOW);
        OUT_WRITE(TEM_EN_PIN, HIGH);
        SET_OUTPUT(TEM_PWM_PIN);
        return true;
    }();

    static auto dir = [](bool d){
        WRITE(TEM_INA_PIN, d);
        WRITE(TEM_INB_PIN, !d);
    };

    if (parser.seen('C')) dir(parser.value_bool());

    if (parser.seen('E')) WRITE(TEM_EN_PIN, parser.value_bool());

    if (parser.seen('B')) pwm_start(digitalPinToPinName(TEM_PWM_PIN), 10000, parser.value_ulong(), TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
}