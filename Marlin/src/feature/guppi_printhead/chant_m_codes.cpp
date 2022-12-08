// copyright Cellink 2022 GPLv3

#include "../../gcode/gcode.h"
#include "../../gcode/parser.h"
#include "../../module/planner.h"

#include "request.h"

#if ENABLED(CHANTARELLE_SUPPORT)

printhead::Controller ph_controller(CHANT_SERIAL);

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

void ph_debug_print(printhead::Response response)
{
    if (DEBUGGING(INFO))
        printhead::print_response(response);
}

void ph_debug_print(printhead::Result result)
{
    if (DEBUGGING(INFO))
        SERIAL_ECHOLN(printhead::string_from_result_code(result));
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
    auto res = ph_controller.home_extruder(index, printhead::ExtruderDirection::Extrude);
    ph_debug_print(res);
}

/**
 * @brief Home slider valve
 * 
 */
void GcodeSuite::G512()
{
    BIND_INDEX_OR_RETURN(index);
    auto res = ph_controller.home_slider_valve(index, printhead::SliderDirection::Pull);
    ph_debug_print(res);
}

/**
 * @brief Slider valve move
 * 
 */
void GcodeSuite::G513()
{
    static constexpr float sv_steps_per_mm = 1000.0f;
    BIND_INDEX_OR_RETURN(index);
    if (!parser.seen('P'))
        return;
    float position = parser.value_float();
    uint16_t steps = static_cast<uint16_t>(position * sv_steps_per_mm);
    auto res = ph_controller.move_slider_valve(index, steps);
    ph_debug_print(res);
}

//
// Common printhead commands
//

// debug arbitrary command, super unsafe
void GcodeSuite::M1069()
{
    static uint8_t cmd_buf[128]{};
    const printhead::Index index = static_cast<printhead::Index>(get_target_extruder_from_command());
    const printhead::Command command = static_cast<printhead::Command>(parser.ushortval('C'));
    const char* payload = parser.string_arg;
    uint8_t cmd_size = 0;
    if (payload != nullptr) {
        if (DEBUGGING(INFO)) SERIAL_ECHOPGM("input: ", payload, " parsed: ");
        if (payload[0] == 'P')
            payload += 1;
        if (payload[0] == '0' && payload[1] == 'x')
            payload += 2;
        while (isHexadecimalDigit(payload[0]) && isHexadecimalDigit(payload[1]) && cmd_size < 128) {
            cmd_buf[cmd_size] = (HEXCHR(payload[0]) << 4) + HEXCHR(payload[1]);
            if (DEBUGGING(INFO)) {SERIAL_PRINT(cmd_buf[cmd_size], PrintBase::Hex); SERIAL_CHAR(' ');}
            ++cmd_size;
            payload += 2;
        }
        if (DEBUGGING(INFO)) SERIAL_EOL();
    }
    printhead::Packet debug_cmd(index, command, cmd_buf, cmd_size);
    const auto response = printhead::send_and_receive(debug_cmd, CHANT_SERIAL);
    ph_debug_print(response);
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
    const float temperature = parser.floatval('C');
    auto res = ph_controller.set_temperature(index, temperature);
    ph_debug_print(res);
}
//GetAllPrintheadsTemps
void GcodeSuite::M772() {}
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
    auto res = ph_controller.set_pid(index, p, i, d);
    ph_debug_print(res);
}
//GetPrintHdHeaterPIDparams
void GcodeSuite::M778() {}
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
    auto res = ph_controller.get_uuid(index);
    ph_debug_print(res);
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
//SetPrintheadFanSpeed
void GcodeSuite::M792() {}
//GetPrintheadFanSpeed
void GcodeSuite::M793() {}
//SetPHAmpCorrParams
void GcodeSuite::M794() {}
//GetPHAmpCorrParams
void GcodeSuite::M795() {}
//GetPHSWVersion
void GcodeSuite::M796() {}
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
void GcodeSuite::M807() {}
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
void GcodeSuite::M818() {}
//SetBedCoolingFans
void GcodeSuite::M819() {}
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
void GcodeSuite::M849() {}
//GetToolMachineOffset
void GcodeSuite::M855() {}
//ProbeSurface
void GcodeSuite::M856() {}
//SetZProbePlanePosition
void GcodeSuite::M857() {}
//GetIsAutoBedLevelingPointsSet
void GcodeSuite::M858() {}
//PrintToolOffset
void GcodeSuite::M860() {}
//SetDropSegments
// void GcodeSuite::M900() {} CONFLICT - linear advance
//DigitalTrimpotControl
void GcodeSuite::M908() {}
//RetrieveHardwareRevision
void GcodeSuite::M910() {}
//GetPHHWConnectionStatus
void GcodeSuite::M1001() {}
//DbgTrig
void GcodeSuite::M1002() {}
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
void GcodeSuite::M1018() {}
//ReadExternalGPIOs
void GcodeSuite::M1020() {}
//GetAllPHTempStatus
void GcodeSuite::M1023() {}
//ActivateToolDBG
void GcodeSuite::M1024() {}
//BedTempDebug
void GcodeSuite::M1025() {}
//SetPeltierValue
void GcodeSuite::M1028() {}
//SetBedTempContrlParams
void GcodeSuite::M1034() {}
//SetBedThermisterParams
void GcodeSuite::M1035() {}
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
//SetPHExtrusionSpeed
void GcodeSuite::M2030()
{
    BIND_INDEX_OR_RETURN(index);
    const feedRate_t feedrate = parser.feedrateval('F');
    if (feedrate == 0.0f)
        return;
    // TODO: maybe need to convert from uL/s to mm/m
    auto res = ph_controller.set_extrusion_speed(index, feedrate);
    ph_debug_print(res);
}
//GetPHExtrusionSpeed
void GcodeSuite::M2031()
{
    BIND_INDEX_OR_RETURN(index);
    auto res = ph_controller.get_extrusion_speed(index);
    ph_debug_print(res);
}
//SetPHIntExtrusionSpeed
void GcodeSuite::M2032() {}
//GetPHIntExtrusionSpeed
void GcodeSuite::M2033() {}
//SetPHExtrusionStepVol
void GcodeSuite::M2034() {}
//GetPHExtrusionStepVol
void GcodeSuite::M2035() {}
//SetPHFullstepExtrusionVol
void GcodeSuite::M2036() {}
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
void GcodeSuite::M2040() {}
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
void GcodeSuite::M2053() {}
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
void GcodeSuite::M2070()
{
    BIND_INDEX_OR_RETURN(index);
    const auto response = ph_controller.get_extruder_rms_current(index);
    if (response.result != printhead::Result::OK || response.packet.payload_size != 2)
        return;
    uint16_t current;
    memcpy(&current, response.packet.payload, 2);
    SERIAL_ECHOLNPGM_P("Printhead ", static_cast<uint8_t>(response.packet.ph_index), " current:", current);
}
//SetPHHoldCurrent
void GcodeSuite::M2071()
{
    BIND_INDEX_OR_RETURN(index);
    const uint16_t current = parser.ushortval('C');
    ph_controller.set_extruder_hold_current(index, current);
}
//GetPHHoldCurrent
void GcodeSuite::M2072()
{
    BIND_INDEX_OR_RETURN(index);
    const auto response = ph_controller.get_extruder_hold_current(index);
    if (response.result != printhead::Result::OK || response.packet.payload_size != 2)
        return;
    uint16_t current;
    memcpy(&current, response.packet.payload, 2);
    SERIAL_ECHOLNPGM_P("Printhead ",
                       static_cast<uint8_t>(response.packet.ph_index),
                       " hold current:",
                       current);
}
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

#endif //  CHANTARELLE_SUPPORT
