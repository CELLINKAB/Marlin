// copyright Cellink 2022 GPLv3

#include "../../gcode/gcode.h"
#include "../../gcode/parser.h"
#include "../../module/planner.h"

#include "chantarelle.h"

#if ENABLED(CHANTARELLE_SUPPORT)

printhead::Controller ph_controller(CHANT_SERIAL);

//
// auto reporting
//

#    if ENABLED(AUTO_REPORT_CHANTARELLE)

void printhead::reporters::State::report()
{
    ph_controller.report_states();
}

static printhead::reporters::State printhead_reporter;

void printhead::reporters::tick_all()
{
    printhead_reporter.tick();
}
#    endif

//
// Helper functions
//

inline printhead::Index get_ph_index()
{
    uint8_t tool = GcodeSuite::get_target_extruder_from_command();
    switch (tool) {
    case 0:
        [[fallthrough]];
    case 1:
        [[fallthrough]];
    case 2:
        [[fallthrough]];
    case 0xff:
        return static_cast<printhead::Index>(tool);
    default:
        return printhead::Index::None;
    }
}

constexpr printhead::ExtruderDirection extrude_dir_from_bool(bool dir)
{
    return dir ? printhead::ExtruderDirection::Retract : printhead::ExtruderDirection::Extrude;
}

//
// Helper Macros
//

// this will declare a variable in scope, be careful!
#    define BIND_INDEX_OR_RETURN(index_var_name) \
        const printhead::Index index_var_name = get_ph_index(); \
        if (index_var_name == printhead::Index::None) \
        return

//
// extruder bottomout
//

//
// extruder bottomout
//

/**
 * @brief Home extruder
 * 
 */
void GcodeSuite::G511()
{
    BIND_INDEX_OR_RETURN(index);
    printhead::ExtruderDirection dir = parser.seen('U') ? printhead::ExtruderDirection::Retract
                                                        : printhead::ExtruderDirection::Extrude;
    ph_controller.home_extruder(index, dir);

    if (DEBUGGING(LEVELING))
        SERIAL_ECHO_MSG("extruder home started");
}

/**
 * @brief Home slider valve
 * 
 */
void GcodeSuite::G512()
{
    BIND_INDEX_OR_RETURN(index);
    ph_controller.home_slider_valve(index, printhead::SliderDirection::Pull);

    if (DEBUGGING(LEVELING))
        SERIAL_ECHO_MSG("slider valve home started");
}

/**
 * @brief Slider valve move
 * 
 */
void GcodeSuite::G513()
{
    static auto steps_from_mm = [](float mm) {
        return static_cast<int32_t>(mm / printhead::MM_PER_MICRO_STEP);
    };
    BIND_INDEX_OR_RETURN(index);
    if (!parser.seenval('P'))
        return;
    float position = parser.value_float();
    planner.synchronize();
    ph_controller.move_slider_valve(index, steps_from_mm(position));

    if (DEBUGGING(LEVELING))
        SERIAL_ECHO_MSG("slider valve move started");
}

//
// Common printhead commands
//

// debug arbitrary command, super unsafe
void GcodeSuite::M1069()
{
    static uint8_t cmd_buf[128]{};
    const uint16_t index = (get_target_extruder_from_command());
    const uint16_t command = (parser.ushortval('C'));
    memcpy(cmd_buf, &index, 2);
    memcpy(cmd_buf + 2, &command, 2);
    const char* payload = parser.string_arg;
    uint16_t cmd_size = 0;
    if (payload != nullptr) {
        if (DEBUGGING(INFO))
            SERIAL_ECHOPGM("input: ", payload, " parsed: ");
        if (payload[0] == 'P')
            payload += 1;
        if (payload[0] == '0' && payload[1] == 'x')
            payload += 2;
        while (isHexadecimalDigit(payload[0]) && isHexadecimalDigit(payload[1]) && cmd_size < 125) {
            cmd_buf[cmd_size + 6] = (HEXCHR(payload[0]) << 4) + HEXCHR(payload[1]);
            if (DEBUGGING(INFO)) {
                SERIAL_PRINT(cmd_buf[cmd_size], PrintBase::Hex);
                SERIAL_CHAR(' ');
            }
            ++cmd_size;
            payload += 2;
        }
        if (DEBUGGING(INFO))
            SERIAL_EOL();
    }
    memcpy(cmd_buf + 4, &cmd_size, 2);
    uint16_t crc = printhead::crc16_from_bytes(cmd_buf, cmd_size);
    memcpy(cmd_buf + 6 + cmd_size, &crc, 2);
    SERIAL_ECHO("Sending: Packet { index: ");
    SERIAL_PRINT(cmd_buf[0], PrintBase::Hex);
    SERIAL_CHAR(' ');
    SERIAL_PRINT(cmd_buf[1], PrintBase::Hex);
    SERIAL_ECHO(", command: ");
    SERIAL_PRINT(cmd_buf[2], PrintBase::Hex);
    SERIAL_CHAR(' ');

    SERIAL_PRINT(cmd_buf[3], PrintBase::Hex);
    SERIAL_ECHO(", size: ");
    SERIAL_PRINT(cmd_buf[4], PrintBase::Hex);
    SERIAL_CHAR(' ');

    SERIAL_PRINT(cmd_buf[5], PrintBase::Hex);
    SERIAL_ECHO(", payload: ");
    for (size_t i = 0; i < cmd_size; ++i) {
        SERIAL_PRINT(cmd_buf[6 + i], PrintBase::Hex);
        SERIAL_CHAR(' ');
    }
    SERIAL_ECHO(", crc: ");
    SERIAL_PRINT(cmd_buf[6 + cmd_size], PrintBase::Hex);
    SERIAL_PRINT(cmd_buf[6 + cmd_size + 1], PrintBase::Hex);
    SERIAL_ECHOLN(" }");
    printhead::flush_rx(CHANT_SERIAL);
    WRITE(CHANT_RTS_PIN, HIGH);
    auto written = CHANT_SERIAL.write(cmd_buf, cmd_size + 8);
    CHANT_SERIAL.flush();
    WRITE(CHANT_RTS_PIN, LOW);
    SERIAL_ECHOLNPGM("Sent ", written, " bytes");
    if (written != cmd_size + 8U) {
        SERIAL_ECHOLNPGM("Serial error, code: ", CHANT_SERIAL.getWriteError());
    }
    CHANT_SERIAL.setTimeout(500);
    size_t read_bytes = CHANT_SERIAL.readBytes(cmd_buf, 128);
    SERIAL_ECHOLNPGM("Received ", read_bytes, " bytes");
    SERIAL_ECHO("Response: [ ");
    for (size_t i = 0; i < read_bytes; ++i) {
        SERIAL_PRINT(cmd_buf[i], PrintBase::Hex);
        SERIAL_CHAR(' ');
    }
    SERIAL_ECHOLN("]");
}

//StartActuatingPrinthead
void GcodeSuite::M750() {}
//StopActuatingPrinthead
void GcodeSuite::M751() {}
//StartActuatingCoaxial
void GcodeSuite::M752() {}
//StopActuatingCoaxial
void GcodeSuite::M753() {}
//EnumeratePrintheads
void GcodeSuite::M770() {}
//SetCurrentPrintheadTemp
void GcodeSuite::M771()
{
    BIND_INDEX_OR_RETURN(index);
    if (parser.seen('D')) { // debug, set PWM directly
        printhead::TemTemps both_tems_pwm;
        const uint16_t tem_pwm = min(parser.ulongval('S'), 4095UL);
        if (parser.seen('I')) {
            const uint8_t tem_index = constrain(parser.value_byte(),
                                                0,
                                                printhead::constants::CS_FANS - 1);
            both_tems_pwm[tem_index] = tem_pwm;
        } else {
            for (auto& tem : both_tems_pwm)
                tem = tem_pwm;
        }
        ph_controller.set_tem_debug(index, both_tems_pwm);

        return;
    }
    else if (parser.seenval('P')) {
    const int16_t temperature = parser.celsiusval('P');
    ph_controller.set_temperature(index, temperature);
    } else { SERIAL_ECHOLN("BAD_TEMPERATURE_CONTROL_PARAMS");}
}

//GetAllPrintheadsTemps
// M772 - MOVED
//SetPrintheadPressure
void GcodeSuite::M773() {}
//GetPrintheadPressure
void GcodeSuite::M774() {}
//SetPrintHdHeaterPIDparams
void GcodeSuite::M777()
{
    BIND_INDEX_OR_RETURN(index);
    const float p = parser.floatval('P');
    const float i = parser.floatval('I');
    const float d = parser.floatval('D');
    ph_controller.set_pid(index, p, i, d);
}
//GetPrintHdHeaterPIDparams
void GcodeSuite::M778()
{
    BIND_INDEX_OR_RETURN(index);
    auto res = ph_controller.get_pid(index);
    if (res.result != printhead::Result::OK)
        return;
    float kP = static_cast<float>(res.packet.payload[0]) / 100.0f;
    float kI = static_cast<float>(res.packet.payload[1]) / 100.0f;
    float kD = static_cast<float>(res.packet.payload[2]) / 100.0f;
    SERIAL_ECHOLNPGM("TOOL:", static_cast<uint16_t>(index), ",kP:", kP, ",kI:", kI, ",kD:", kD);
}
//SetPrintheadTempFilterParams
void GcodeSuite::M779() {}
//GetPrintheadTempFilterParams
void GcodeSuite::M780() {}
//SetPrintHdHeaterMaxPow
void GcodeSuite::M781() {}
//GetPrintHeadProbedStat
void GcodeSuite::M782() {}
//GetPrintheadType
void GcodeSuite::M783() {}
//GetPrintheadMounted
void GcodeSuite::M784() {}
//GetPrintheadUUID
void GcodeSuite::M785()
{
    BIND_INDEX_OR_RETURN(index);
    ph_controller.get_uuid(index);
}
//SetPrintheadLED
void GcodeSuite::M786() {}
//GetPrintheadLEDDetails
void GcodeSuite::M787() {}
//SetPrintheadRoomTemp
void GcodeSuite::M788() {}
//SetPrintheadFeedrateMultiplier
void GcodeSuite::M790() {}
//GetPrintheadFeedrateMultiplier
void GcodeSuite::M791() {}
/**
 * @brief SetFanSpeed sets fan PWM on one or both fans on an extruder
 * 
*/
void GcodeSuite::M792()
{
    BIND_INDEX_OR_RETURN(index);
    printhead::FanSpeeds both_fans_pwm;
    const uint16_t fan_pwm = min(parser.ulongval('S'), 4095UL);
    if (parser.seen('I')) {
        const uint8_t fan_index = constrain(parser.value_byte(), 0, printhead::constants::CS_FANS - 1);
        both_fans_pwm[fan_index] = fan_pwm;
    } else {
        for (auto& fan : both_fans_pwm)
            fan = fan_pwm;
    }

    ph_controller.set_fan_speed(index, both_fans_pwm);
}
/**
 * @brief GetFanSpeed returns set speed and tech output for com-module
 * 
*/
void GcodeSuite::M793()
{
    BIND_INDEX_OR_RETURN(index);
    ph_controller.get_fan_speed(index);
    // TODO: custom print format
}
//SetPHAmpCorrParams
void GcodeSuite::M794() {}
//GetPHAmpCorrParams
void GcodeSuite::M795() {}
//GetPHSWVersion
void GcodeSuite::M796()
{
    BIND_INDEX_OR_RETURN(index);
    auto res = ph_controller.get_fw_version(index);

    if (res.result == printhead::Result::OK) {
        SERIAL_ECHOLNPGM("PH:", static_cast<uint8_t>(index), ",FW_VERSION:", res.packet.payload.data());
    }
}
//SetPHNotCalibrated
// void GcodeSuite::M797() {}
// //SetPrintheadExternalPWM
// void GcodeSuite::M798() {}
// //GetPrintheadExternalPWM
// void GcodeSuite::M799() {}
//DisableBedTempController
void GcodeSuite::M800() {}
//SetBedTempController
// void GcodeSuite::M801() {}
// //GetBedTempControllerInfo
// void GcodeSuite::M802() {}
//SetPeltierOutputLimit
void GcodeSuite::M803() {}
//SetThermistorUnifiedParams
void GcodeSuite::M804() {}
//SetUVCrosslinkingLEDs
// void GcodeSuite::M805() {}
//SetUVSterilization
//void GcodeSuite::M806() {}
//SetCleanchamberFan
void GcodeSuite::M807()
{
    //TODO: alias marlin command
}
//SetAirValves
void GcodeSuite::M808() {}
//ControlRGBLED
void GcodeSuite::M810()
{
    M150();
}
//ControlPHVerticalMove
void GcodeSuite::M811() {}
//GetZ3EndstopStatus
void GcodeSuite::M814() {}
//GetTankNRegulatorPressure
void GcodeSuite::M816() {}
//MovePHsVertically
void GcodeSuite::M817() {}
//ReadDoorStatus
void GcodeSuite::M818()
{
    SERIAL_ECHOLNPGM("DO:", (READ(DOOR_PIN) ^ DOOR_SENSOR_INVERTING), ",INTERLOCK_24V:", (READ(FREEZE_PIN) ^ FREEZE_STATE));
}
//SetBedCoolingFans
void GcodeSuite::M819()
{
    //TODO: alias marlin command
}
//PrinterHomingStatus
// void GcodeSuite::M821() {}
//ControlAirPumpStatus
void GcodeSuite::M822() {}
//SafePark
void GcodeSuite::M823() {}
//GetCurrActiveTool
// void GcodeSuite::M824() {}
//GetCurrPhotocuringParams
void GcodeSuite::M825() {}
//SetHomeDirection
void GcodeSuite::M826() {}
//SetDarkMode
void GcodeSuite::M830() {}
//SetMotorCurrent
void GcodeSuite::M842() {}
//ControlPin
void GcodeSuite::M848()
{
    M42();
}
//GetSystemTime
void GcodeSuite::M849()
{
    SERIAL_ECHOLNPGM("uptime:", millis());
}
//GetToolMachineOffset
void GcodeSuite::M855() {}
//ProbeSurface
void GcodeSuite::M856() {}
//SetZProbePlanePosition
void GcodeSuite::M857() {}
//GetIsAutoBedLevelingPointsSet
void GcodeSuite::M858() {}
//PrintToolOffset
void GcodeSuite::M860()
{
    for (uint32_t tool = 0; tool < EXTRUDERS; ++tool)
        SERIAL_ECHOLNPGM("TOOL:",
                         tool,
                         ",X:",
                         hotend_offset[tool].x,
                         ",Y:",
                         hotend_offset[tool].y,
                         ",Z:",
                         hotend_offset[tool].z);
}
//SetDropSegments
// void GcodeSuite::M900() {} CONFLICT - linear advance
//DigitalTrimpotControl
void GcodeSuite::M908() {}
//RetrieveHardwareRevision
void GcodeSuite::M910() {}
//GetPHHWConnectionStatus
void GcodeSuite::M1001() {}
//DbgTrig
void GcodeSuite::M1002()
{
#    if PIN_EXISTS(CHANT_IRQ1)
    if (parser.seenval('S'))
        WRITE(CHANT_IRQ1_PIN, parser.value_bool());
    else
        SERIAL_ECHOLNPGM("TRIG_PIN:", READ(CHANT_IRQ1_PIN));
#    endif
}
//ControlPHActuateStatus
void GcodeSuite::M1004() {}
//GetPHMountedStatus
void GcodeSuite::M1005() {}
//SendCustomCommandToPH
void GcodeSuite::M1006()
{
    M1069();
}
//ResetAwaitingResponse
void GcodeSuite::M1008() {}
//SetPrintheadHeaterValue
void GcodeSuite::M1012() {}
//PrintCurrentPosition
// void GcodeSuite::M1015() {}
// //PrintCurrentMechanicalPos
// void GcodeSuite::M1016() {}
// //PrintCurrentToolOffset
// void GcodeSuite::M1017() {}
//DbgPhCom
void GcodeSuite::M1018()
{
    M1069();
}
//ReadExternalGPIOs
void GcodeSuite::M1020()
{
    const uint32_t pin = parser.ulongval('P');
    SERIAL_ECHOLNPGM("PINNAME:", digitalPinToPinName(pin), ",STATE:", READ(pin));
}
//GetAllPHTempStatus
void GcodeSuite::M1023()
{
    BIND_INDEX_OR_RETURN(index);
    auto res = ph_controller.debug_get_temperature(index);
    if (res.result == printhead::Result::OK) {
        float temp1 = static_cast<float>(res.packet.payload[0]) / 100.0f;
        float temp2 = static_cast<float>(res.packet.payload[1]) / 100.0f;
        SERIAL_ECHOLNPGM("TOOL:", static_cast<uint16_t>(index), ",TEMP_1:", temp1, ",TEMP_2:", temp2);
    } else
        SERIAL_ECHOLN("ERROR");
}
//ActivateToolDBG
void GcodeSuite::M1024() {}
//BedTempDebug
void GcodeSuite::M1025()
{
    M801();
}
//SetPeltierValue
void GcodeSuite::M1028() {}
//SetBedTempContrlParams
void GcodeSuite::M1034() {}
//SetRegulatorPressure
//void GcodeSuite::M1036() {} - MOVED
//GetAllAxesMotorCurrent
void GcodeSuite::M1037() {}
//GetPrintbedProbesStatus
void GcodeSuite::M1038() {}
//SetPeltierPolarity
void GcodeSuite::M1039() {}
//EnableDisableBuck
void GcodeSuite::M1040() {}
//StartStopPump
void GcodeSuite::M1042() {}
//GetAllPositions
void GcodeSuite::M1045() {}
//GetAllToolOffsets
void GcodeSuite::M1046() {}
//EnableDisableHeartbeat
void GcodeSuite::M1047() {}
//GetPrintbedPeltierConfig
void GcodeSuite::M1048() {}
//ChangeSerialBaudrate
void GcodeSuite::M1050() {}
//CheckCurrentCompileVersion
void GcodeSuite::M1051() {}
//GetPressureAndExtrusionData
//void GcodeSuite::M1062() {} - MOVED
//MovePrintheadsUpDownDBG
void GcodeSuite::M1063() {}
//IgnoreCurrentPrinthead
void GcodeSuite::M1064() {}
//SetPrintheadsTemperatureVerboseMode
void GcodeSuite::M1065() {}
//SetPhotocuringModuleVerboseMode
void GcodeSuite::M1066() {}
//DBGRetrieveCurrentExtADC
void GcodeSuite::M1070() {}

//
// Syringe pump codes
//

//DebugAddSteps
void GcodeSuite::M2020()
{
    BIND_INDEX_OR_RETURN(index);
    const int32_t steps = parser.longval('S');
    ph_controller.add_raw_extruder_steps(index, steps);
}
//SetPHExtrusionSpeed
void GcodeSuite::M2030()
{
    BIND_INDEX_OR_RETURN(index);
    const float feedrate_ul_s = parser.floatval('F');
    const uint32_t feedrate_pl_s = static_cast<uint32_t>(feedrate_ul_s * 1'000'000);
    if (feedrate_ul_s == 0.0f)
        return;
    // TODO: maybe need to convert from uL/s to mm/m
    ph_controller.set_extrusion_speed(index, feedrate_pl_s);
    feedrate_mm_s = feedrate_ul_s
                    / ((DEFAULT_NOMINAL_FILAMENT_DIA / 2.0f) * (DEFAULT_NOMINAL_FILAMENT_DIA / 2.0f)
                       * PI);
}
//GetPHExtrusionSpeed
void GcodeSuite::M2031()
{
    BIND_INDEX_OR_RETURN(index);
    ph_controller.get_extrusion_speed(index);
}
//SetPHIntExtrusionSpeed
void GcodeSuite::M2032() {}
//GetPHIntExtrusionSpeed
void GcodeSuite::M2033() {}
//SetPHExtrusionStepVol
void GcodeSuite::M2034()
{
    BIND_INDEX_OR_RETURN(index);
    if (!parser.seenval('V'))
        return;
    const uint32_t picoliters = static_cast<uint32_t>(parser.value_float() * 1000);
    ph_controller.set_step_volume(index, picoliters);
}
//GetPHExtrusionStepVol
void GcodeSuite::M2035()
{
    BIND_INDEX_OR_RETURN(index);
    ph_controller.get_step_volume(index);
}
//SetPHFullstepExtrusionVol
void GcodeSuite::M2036()
{
    BIND_INDEX_OR_RETURN(index);
    const uint32_t pL_volume = parser.ulongval('V');
    ph_controller.set_volume_per_fullstep(index, pL_volume);
}
//GetPHFullstepExtrusionVol
void GcodeSuite::M2037() {}
//SetPHMicrostep
void GcodeSuite::M2038()
{
    BIND_INDEX_OR_RETURN(index);
    if (!parser.seen('S'))
        return;
    const uint8_t microsteps = parser.value_byte();
    ph_controller.set_extruder_microsteps(index, microsteps);
}
//GetPHMicrostep
void GcodeSuite::M2039()
{
    BIND_INDEX_OR_RETURN(index);
    ph_controller.get_extruder_microsteps(index);
}
//GetPHStatusByte
void GcodeSuite::M2040()
{
    BIND_INDEX_OR_RETURN(index);
    auto res = ph_controller.get_status(index);
    if (res.result == printhead::Result::OK) {
        SERIAL_ECHO("STATUS:");
        print_bin(res.packet.payload.to_raw());
        SERIAL_EOL();
    } else
        SERIAL_ECHOLNPGM("ERROR:", printhead::string_from_result_code(res.result));
}
//SetPHEndStopThreshold
void GcodeSuite::M2041()
{
    BIND_INDEX_OR_RETURN(index);
    if (!parser.seen('S'))
        return;
    const uint8_t sg_threshold = parser.value_byte();
    ph_controller.set_extruder_stallguard_threshold(index, sg_threshold);
}
//GetPHEndStopThreshold
void GcodeSuite::M2042()
{
    BIND_INDEX_OR_RETURN(index);
    ph_controller.get_extruder_stallguard_threshold(index);
}
//GetPPHEndstopValues
void GcodeSuite::M2043() {}
//GetPHReadableStatus
void GcodeSuite::M2044() {}
//SetPHExtrusionDirection
void GcodeSuite::M2045() {}
//ControlPHMotorStatus
void GcodeSuite::M2046() {}
//HomePrinthead
void GcodeSuite::M2047()
{
    BIND_INDEX_OR_RETURN(index);
    const auto direction = extrude_dir_from_bool(parser.boolval('D'));
    ph_controller.home_extruder(index, direction);
}
//PHCurrHomingStatus
void GcodeSuite::M2048() {}
//DebugPHMicrosteps
void GcodeSuite::M2049() {}
//DebugExtrudeDirection
void GcodeSuite::M2050() {}
//ControlSyringeExtrusionVol
void GcodeSuite::M2051() {}
//HeaterSelfTest
void GcodeSuite::M2052() {}
//TurnOffHeater
void GcodeSuite::M2053()
{
    BIND_INDEX_OR_RETURN(index);
    ph_controller.disable_heating(index);
}
//GetTempControlCurrent
void GcodeSuite::M2054() {}
//GetPHInternalTemp
void GcodeSuite::M2055() {}
//GetBuckMeasOutputVoltage
void GcodeSuite::M2056() {}
//SetMaxCoolingVoltage
void GcodeSuite::M2057() {}
//GetMaxCoolingPower
void GcodeSuite::M2058() {}
//SetPeltierPolarity
void GcodeSuite::M2059() {}
//SetTempContBuckCalib
void GcodeSuite::M2060() {}
//GetTempContPHBuckCalibVal
void GcodeSuite::M2061() {}
//SetPHPeakTime
void GcodeSuite::M2063() {}
//GetPHPeakTime
void GcodeSuite::M2064() {}
//SetPHOpenTime
void GcodeSuite::M2065() {}
//GetPHOpenTime
void GcodeSuite::M2066() {}
//SetPHCycleTime
void GcodeSuite::M2067() {}
//GetPHCycleTime
void GcodeSuite::M2068() {}
//SetPHPeakCurrent
void GcodeSuite::M2069()
{
    BIND_INDEX_OR_RETURN(index);
    const uint16_t current = parser.ushortval('C');
    ph_controller.set_extruder_rms_current(index, current);
}
//GetPHPeakCurrent
void GcodeSuite::M2070() {}
//SetPHHoldCurrent
void GcodeSuite::M2071() {}
//GetPHHoldCurrent
void GcodeSuite::M2072() {}
//ControlPHAirSupply
void GcodeSuite::M2073() {}
//StartInkjetSelfTest
void GcodeSuite::M2075() {}
//GetInkjetSelfTestResult
void GcodeSuite::M2076() {}
//SetPHLEDOutputMode
void GcodeSuite::M2080() {}
//SetPHPhotocuringPow
void GcodeSuite::M2081() {}
//GetPHPhotocuringPow
void GcodeSuite::M2082() {}
//GetPHLEDWaveLength
void GcodeSuite::M2083() {}
//GetPHLEDPhotoFeedback
void GcodeSuite::M2084() {}
//StartPHExtLEDCalibNSelfTest
void GcodeSuite::M2085() {}
//GetPHCalibResult
void GcodeSuite::M2086() {}
//GetPHExtLEDVoltage
void GcodeSuite::M2087() {}
//GuppiParamsStore
void GcodeSuite::M2090() {}
//GuppiParamsRestoreToDefault
void GcodeSuite::M2091() {}
//GuppiParamsReset
void GcodeSuite::M2092() {}
//GuppiParamsRestore
void GcodeSuite::M2093() {}
//SetPHTempCorrModel
void GcodeSuite::M2095() {}
//GetPHTempCorrModel
void GcodeSuite::M2096() {}
//SetPHTempRegIParams
void GcodeSuite::M2097() {}
//GetPHTempRegIParams
void GcodeSuite::M2098() {}
//SetPHTempRegOvershoot
void GcodeSuite::M2099() {}
//GetPHTempRegOvershoot
void GcodeSuite::M2100() {}
//SetCoaxialCouple
void GcodeSuite::M2110() {}
//ResetCoaxialCouple
void GcodeSuite::M2111() {}
//DebugGetEncoders
void GcodeSuite::M2200()
{
    const auto res = ph_controller.debug_get_encoders();
    SERIAL_ECHOLNPGM(",SLIDER_0_ENCODER:",
                     res.packet.payload[0],
                     ",EXTRUDER_0_ENCODER:",
                     res.packet.payload[1],
                     ",SLIDER_1_ENCODER:",
                     res.packet.payload[2],
                     ",EXTRUDER_1_ENCODER:",
                     res.packet.payload[3],
                     ",SLIDER_2_ENCODER:",
                     res.packet.payload[4],
                     ",EXTRUDER_2_ENCODER:",
                     res.packet.payload[5]);

    // FIXME: Put this in a better place and modularize
    TERN_(AUTO_REPORT_CHANTARELLE, printhead_reporter.set_interval(parser.byteval('S')));
}

size_t printhead::printhead_rx_err_counter = 0;
size_t printhead::printhead_tx_err_counter = 0;

// get printhead communication error counters
void GcodeSuite::M2201()
{
    SERIAL_ECHOLNPGM("PRINTHEAD_TX_ERRORS:",
                     printhead::printhead_tx_err_counter,
                     ",PRINTHEAD_RX_ERRORS:",
                     printhead::printhead_rx_err_counter);
}

#endif //  CHANTARELLE_SUPPORT
