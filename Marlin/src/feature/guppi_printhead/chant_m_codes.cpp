// copyright Cellink 2022 GPLv3

#include "../../gcode/gcode.h"
#include "../../gcode/parser.h"

#include "request.h"

printhead::Controller ph_controller(CHANT_SERIAL);

//
// Helper functions
//

inline printhead::Index get_ph_index()
{
    uint8_t tool = GcodeSuite::get_target_extruder_from_command();
    switch (tool) {
    case 1:
        [[fallthrough]];
    case 2:
        [[fallthrough]];
    case 3:
        [[fallthrough]];
    case 0xff:
        return static_cast<printhead::Index>(tool);
    default:
        return printhead::Index::None;
    }
}

//
// Helper Macros
//

#define HANDLE_NONE_INDEX(index) \
    if (index == printhead::Index::None) \
    return

#define APPLY_ALL_INDEX(operation, ...) \
    operation(printhead::Index::One, __VA_ARGS__); \
    operation(printhead::Index::Two, __VA_ARGS__); \
    operation(printhead::Index::Three, __VA_ARGS__)

#define HANDLE_ANY_INDEX(index, operation, ...) \
    HANDLE_NONE_INDEX(index); \
    if (index == printhead::Index::All) { \
        APPLY_ALL_INDEX(operation, __VA_ARGS__); \
        return; \
    } \
    operation(index, __VA_ARGS__)

//
// Common printhead commands
//

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
    const printhead::Index index = get_ph_index();
    const float temperature = parser.floatval('C');
    HANDLE_ANY_INDEX(index, ph_controller.set_temperature, temperature);
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
    const printhead::Index index = get_ph_index();
    const float p = parser.floatval('P');
    const float i = parser.floatval('I');
    const float d = parser.floatval('D');
    HANDLE_ANY_INDEX(index, ph_controller.set_pid, p, i, d);
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
void GcodeSuite::M785() {}
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
void GcodeSuite::M797() {}
//SetPrintheadExternalPWM
void GcodeSuite::M798() {}
//GetPrintheadExternalPWM
void GcodeSuite::M799() {}
//DisableBedTempController
void GcodeSuite::M800() {}
//SetBedTempController
void GcodeSuite::M801() {}
//GetBedTempControllerInfo
void GcodeSuite::M802() {}
//SetPeltierOutputLimit
void GcodeSuite::M803() {}
//SetThermistorUnifiedParams
void GcodeSuite::M804() {}
//SetUVCrosslinkingLEDs
void GcodeSuite::M805() {}
//SetUVSterilization
void GcodeSuite::M806() {}
//SetCleanchamberFan
void GcodeSuite::M807() {}
//SetAirValves
void GcodeSuite::M808() {}
//ControlRGBLED
void GcodeSuite::M810() {}
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
void GcodeSuite::M821() {}
//ControlAirPumpStatus
void GcodeSuite::M822() {}
//SafePark
void GcodeSuite::M823() {}
//GetCurrActiveTool
void GcodeSuite::M824() {}
//GetCurrPhotocuringParams
void GcodeSuite::M825() {}
//SetHomeDirection
void GcodeSuite::M826() {}
//SetDarkMode
void GcodeSuite::M830() {}
//SetMotorCurrent
void GcodeSuite::M842() {}
//ControlPin
void GcodeSuite::M848() {}
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
    const printhead::Index index = get_ph_index();
    const uint16_t cmd_arg = parser.ushortval('C');
    // The following is dangerous and may lead to undefined behavior due to unconstrained enum variants.
    const printhead::Command command = static_cast<printhead::Command>(cmd_arg);
    if (!parser.string_arg)
        return;

    const uint16_t msg_len = strlen(parser.string_arg);
    printhead::Packet(index, command, parser.string_arg, msg_len);
}
//ResetAwaitingResponse
void GcodeSuite::M1008() {}
//SetPrintheadHeaterValue
void GcodeSuite::M1012() {}
//PrintCurrentPosition
void GcodeSuite::M1015() {}
//PrintCurrentMechanicalPos
void GcodeSuite::M1016() {}
//PrintCurrentToolOffset
void GcodeSuite::M1017() {}
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
void GcodeSuite::M1036() {}
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
void GcodeSuite::M1062() {}
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

//SetPHIntExtrusionSpeed
void GcodeSuite::M2030() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2031() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2032() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2033() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2034() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2035() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2036() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2037() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2038() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2039() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2040() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2041() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2042() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2043() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2044() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2045() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2046() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2047() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2048() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2049() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2050() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2051() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2052() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2053() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2054() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2055() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2056() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2057() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2058() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2059() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2060() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2061() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2063() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2064() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2065() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2066() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2067() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2068() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2069() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2070() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2071() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2072() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2073() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2075() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2076() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2080() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2081() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2082() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2083() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2084() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2085() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2086() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2087() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2090() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2091() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2092() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2093() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2095() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2096() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2097() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2098() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2099() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2100() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2110() {}

//SetPHIntExtrusionSpeed
void GcodeSuite::M2111() {}
