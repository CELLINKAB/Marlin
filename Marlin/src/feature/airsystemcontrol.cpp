
#include "../inc/MarlinConfig.h"

#if ENABLED(PNEUMATIC_SYSTEM)

#    include "airsystemcontrol.h"

// Todo: Rewrite regulators as an object of some sort.

// Todo: this will be controlled on either machine or EEPROM settings. Currently, we're just setting a magical number
// The number of pressure regulators available
static uint8_t nofRegulators = 2;

// Todo: These offsets will be stored in EEPROM
// The offset for the regulator feedback in kPa (how much should be added to the measured value for skew)
static int16_t regulatorFeedbackOffset[NOF_MAX_REGULATORS] = {0, 0};
// The general offset from when you set a pressure to how much it actually responds with
static int16_t regulatorSetOffset[NOF_MAX_REGULATORS] = {0, 0};

// The pressure value currently set
static uint16_t currentPressureValue[NOF_MAX_REGULATORS] = {0, 0};

// Todo: This will be defined from EEPROM factory settings later
// The maximum pressure you can set on the regulator (called F.S) in kPa
#    define PRESSURE_FULL_SCALE_REG_SMC 900
#    define PRESSURE_FULL_SCALE_REG_FEST 1000
// The maximum output pressure from the regulator. This pressure is output at max control voltage.
// We assume all regulators have the same full-scale
static uint16_t fullScalePressure = PRESSURE_FULL_SCALE_REG_FEST;

// The threshold values for the tank control (in kPa)
static uint16_t tankRequiredPressure = TANK_REQUIRED_PRESSURE_DEFAULT;
static uint16_t tankFullPressure = TANK_FULL_PRESSURE_DEFAULT;

static bool internalPumpEnabled = false;
static bool externalAirSource = false;

// Static functions

namespace regulator {
// Checks if a regulator is actually used
static bool exists(uint8_t reg);
} // namespace regulator

/*
 * Inits the air-system
 */
void airSystemInit()
{
    for (uint8_t i = 0; i < NOF_MAX_REGULATORS; i++) {
        currentPressureValue[i] = 0;
    }
    internalPumpEnabled = false;
    externalAirSource = false;
    pressurePwm1.setTopValue(PRESSURE_PWM_TOP);
    // Note: No need to set top value for pressurePwm2, as it uses the same timer as pressurePwm1
}

typedef enum
{
    pumpTankFull, // Tank is full or pump not active
    pumpStartup,  // Pump is in startup sequence
    pumpFilling   // Pump is currently filling
} pumpTankState_t;

bool airSourceControl(char airSource)
{
    if (0 == airSource) {
        externalAirSource = true;
    } else if (1 == airSource) {
        externalAirSource = false;
    }
    return true;
}

/*
 * Enable internal pump control
 */
void airControlPumpEnable()
{
    internalPumpEnabled = true;
}

/*
 * Disable internal pump control
 */
void airControlPumpDisable()
{
    airReleasePumpPin.write(AIR_RELEASE_PUMP_OPEN);
    pumpOnPin.write(LOW);
    internalPumpEnabled = false;
}

bool airControlGetInternalPumpStatus()
{
    return internalPumpEnabled;
}

/*
 * This function handles the internal pump state machine and detects when it should be filled or not
 *
 * Sequence:
 * 	When the air pressure is too low, start pump.
 * 		When starting the pump do the following sequence:
 * 			Disconnect the tank
 * 			Start the pump
 * 			Wait for a short time (for the pump to start and gain momentum)
 * 			Reconnect the tank to start filling it
 * 		Wait until pressure is high enough
 * 			Stop pump
 */
void airTankPressureControl()
{
    static pumpTankState_t pumpState = pumpTankFull;
    static uint32_t nextActionTime = 0;
    // Pump not enabled or not time to run
    const uint32_t currentTime = millis();
    if (!internalPumpEnabled || currentTime < nextActionTime) {
        if (!internalPumpEnabled) {
            pumpState = pumpTankFull;
        }
        return;
    }
    nextActionTime = currentTime + PUMP_TASK_TIME;

    const uint16_t tankPressure = getActualTankPressure();
    switch (pumpState) {
    case pumpTankFull: {
        // Check if we need to start filling the tank?
        if (tankPressure < tankRequiredPressure) {
            // Start pump and disconnect the air tank.
            pumpOnPin.write(HIGH);
            airReleasePumpPin.write(AIR_RELEASE_PUMP_OPEN);
            // Set the task time to match when we have finished starting the pump
            nextActionTime = currentTime + PUMP_STARTUP_HOLDOFF;
            pumpState = pumpStartup;
        }
        break;
    }
    case pumpStartup: {
        // The pump has started. Connect the tank
        airReleasePumpPin.write(!AIR_RELEASE_PUMP_OPEN);
        pumpState = pumpFilling;
        break;
    }
    case pumpFilling: {
        // Is tank full yet?
        if (tankPressure > tankFullPressure) {
            // Tank is full. Stop pump and disconnect air from it
            airReleasePumpPin.write(AIR_RELEASE_PUMP_OPEN);
            pumpOnPin.write(LOW);
            pumpState = pumpTankFull;
        }
    }
    }
}

/*
 * Returns the tank pressure.
 */
uint16_t getActualTankPressure()
{
    // TankPressure: PF1/ADC1
    // The sensor has for some reason an offset at 0 pressure
    // Todo: It seems the sensor is not linear. The actual pressure in the system is slightly higher than read.
    uint16_t actualTankPressure = readTankPressure();
    if (actualTankPressure < PRESSURE_SENSOR_OFFSET) {
        return 0;
    }
    actualTankPressure = actualTankPressure - PRESSURE_SENSOR_OFFSET;
    // convert offset tank pressure to Kpa range from 0 to PRESSURE_SENSOR_MAX_PRESSURE
    actualTankPressure = map(actualTankPressure, 0, 1023, 0, PRESSURE_SENSOR_MAX_PRESSURE);
    return actualTankPressure;
}

/*
 * Sets the pressure required for the tank control
 * If 0 is given for both, default will be set
 */
void airTankSetThresholds(uint16_t req, uint16_t full)
{
    if (req == 0 && full == 0) {
        tankRequiredPressure = TANK_REQUIRED_PRESSURE_DEFAULT;
        tankFullPressure = TANK_FULL_PRESSURE_DEFAULT;
    }
    if (req > 0) {
        tankRequiredPressure = req;
    }
    if (full > 0) {
        tankFullPressure = full;
    }
}

void airTankEchoThresholds()
{
    SERIAL_PROTOCOL_KEY_VALUE("TANK_REQ", tankRequiredPressure);
    SERIAL_PROTOCOL_KEY_VALUE("TANK_FULL", tankFullPressure);
}

/*
 * Returns the regulator index (starts from 1) for a given printhead.
 */
uint8_t regulator::lookupRegulatorForPH(uint8_t ph)
{
    static constexpr uint8_t regulator2PHs[3] = REGULATOR_2_PHS;
    if (utilIsArrayElement(ph, regulator2PHs, 3)) {
        return 2;
    } else {
        return 1;
    }
}

/*
 * Sets the output pressure directly to a regulator. Regulators are indexed from 1
 * The pressureValue is in kPa
 */
bool regulator::setOutputPressure(uint16_t pressureValue, uint8_t regulator)
{
    if (!exists(regulator)) {
        return false;
    }
    if (pressureValue > MAXIMUM_EXTRUDE_PRESSURE) {
        // We will not clamp the pressure to the maximum pressure, since this might not be good to set due to a mistype. Therefore, we just fail here.
        return false;
    }
    currentPressureValue[regulator - 1] = pressureValue;
    // Ensure that in the case of a negative offset
    if (regulatorSetOffset[regulator - 1] < 0
        && pressureValue < abs(regulatorSetOffset[regulator - 1])) {
        setPWM(0, regulator);
        return true;
    }
    pressureValue += regulatorSetOffset[regulator - 1]; // All regulators seems to have small constant offset. It's very close for all regulators. This might stem from the pressure sensor used...
    const uint16_t pwm = convertmVToPWM(convertkPaTomV(pressureValue));
    setPWM(pwm, regulator);
    return true;
}

/*
 * Writes a value directly to the pressure set PWM for a certain regulator. Regulator is indexed from 1
 * Don't use this unless you really, REALLY mean it
 */
void regulator::setPWM(uint16_t pwm, uint8_t regulator)
{
    if (!exists(regulator)) {
        SERIAL_PROTOCOL_KEY_VALUE("WrongRegNum", (uint16_t) regulator);
        // Silently fail if regulator is not implemented
        return;
    }

    if (pwm > PRESSURE_PWM_TOP) {
        pwm = PRESSURE_PWM_TOP;
    }
    switch (regulator) {
    case 1:
        // SERIAL_PROTOCOL_KEY_VALUE("regNum",(uint16_t)regulator);
        // SERIAL_PROTOCOL_KEY_VALUE("PWM",pwm);
        pressurePwm1.set_duty(pwm);
        // OCR3B=pwm;
        break;
    case 2:
        // SERIAL_PROTOCOL_KEY_VALUE("regNum",(uint16_t)regulator);
        // SERIAL_PROTOCOL_KEY_VALUE("PWM",pwm);
        pressurePwm2.set_duty(pwm);
        // OCR3C=pwm;
        break;
    default:
        SERIAL_PROTOCOL_KEY_VALUE("WrongRegNum", (uint16_t) regulator);
        // Silently fail if regulator is not implemented
        break;
    }
}

// Correction factor for the, apparently non-linear driver
#    define PWM_TO_VOLT_CORR_DIV 1000
#    define PWM_TO_VOLT_CORR 0
#    define PWM_TO_VOLT_CORR_THRESH 400

/*
 * Converts a voltage (in mV) to pwm value
 */
uint16_t regulator::convertmVToPWM(uint16_t mV)
{
    if (mV > PRESSURE_MAX_CTRL_VOLTAGE) {
        mV = PRESSURE_MAX_CTRL_VOLTAGE;
    }
    if (mV < PRESSURE_MIN_CTRL_VOLTAGE) {
        return 0;
    }
    uint32_t pwm = (uint32_t) (mV) *PRESSURE_PWM_TOP / PRESSURE_MAX_CTRL_VOLTAGE;
    const uint32_t pwmCorrVal = pwm * PWM_TO_VOLT_CORR / PWM_TO_VOLT_CORR_DIV;
    if (mV > PWM_TO_VOLT_CORR_THRESH) {
        pwm = pwm + pwmCorrVal;
    } else {
        if (pwm < pwmCorrVal) {
            pwm = 0;
        } else {
            pwm = pwm - pwmCorrVal;
        }
    }
    if (pwm > UINT16_MAX) {
        pwm = UINT16_MAX;
    }
    return (uint16_t) pwm;
}

/*
 * Converts a pressure to what PWM needs to be set for it
 */
uint16_t regulator::convertkPaTomV(uint16_t kpa)
{
    if (kpa > (2 * fullScalePressure)) {
        // Surely, something went wrong here. Return 0
        return 0;
    }
    if (kpa > fullScalePressure) {
        return PRESSURE_MAX_CTRL_VOLTAGE;
    }
    // Todo: Add calibration and support to use a linearity correction, if deemed necessary.
    uint32_t tmpmv = (uint32_t) ((PRESSURE_MAX_CTRL_VOLTAGE * (uint32_t) kpa) / (fullScalePressure));
    return (uint16_t) tmpmv;
}

/*
 * Return the currently set output pressure
 */
uint16_t regulator::getCurrentSetPressure(uint8_t regulator)
{
    if (exists(regulator)) {
        return currentPressureValue[regulator - 1];
    }
    return 0; // Return 0 if regulator doesn't exist
}

/*
 * Returns the output pressure (in kPa) based on the feedback from the regulator
 */
uint16_t regulator::getPressureFeedback(uint8_t regulator)
{
    if (!exists(regulator)) {
        return 0;
    }
    // Regulator1 feedback
    uint16_t feedbackVoltage = 0;
    if (regulator == 1) {
        // PressureRegFB1: PF3/ADC3
        feedbackVoltage = read_adc_millivolts(PRESSURE_REG);
    } else if (regulator == 2) {
        // PressureRegFB2: PF4/ADC4
#    ifdef BIOX6
        feedbackVoltage = read_adc_millivolts(PRESSURE_REG2);
#    else
        feedbackVoltage = 0;
#    endif
    }
    if (feedbackVoltage < PRESSURE_FEEDBACK_MIN_VOLT) {
        return 0;
    }

    uint32_t regPress = (((uint32_t) feedbackVoltage - PRESSURE_FEEDBACK_MIN_VOLT) * fullScalePressure)
                        / (PRESSURE_FEEDBACK_MAX_VOLT - PRESSURE_FEEDBACK_MIN_VOLT);
    int32_t offsetRegPress = (int32_t) regPress + regulatorFeedbackOffset[regulator - 1];
    offsetRegPress = LIM(offsetRegPress, 0, UINT16_MAX);
    return (uint16_t) offsetRegPress;
}

/*
 *	Set/Get the regulator feedback offset (in kPa)
 *	This value will be added to the measured value
 */
void regulator::setRegFBOffset(int16_t kPa, uint8_t regulator)
{
    if (exists(regulator)) {
        regulatorFeedbackOffset[regulator - 1] = kPa;
    }
}
int16_t regulator::getRegFBOffset(uint8_t regulator)
{
    if (exists(regulator)) {
        return regulatorFeedbackOffset[regulator - 1];
    }
    return 0;
}

/*
 *	Set/Get the regulator set offset (in kPa)
 *	This value will be added to the wanted value before setting the output to the regulator
 */
void regulator::setRegControlOffset(int16_t kPa, uint8_t regulator)
{
    if (exists(regulator)) {
        regulatorSetOffset[regulator - 1] = kPa;
    }
}
int16_t regulator::getRegControlOffset(uint8_t regulator)
{
    if (exists(regulator)) {
        return regulatorSetOffset[regulator - 1];
    }
    return 0;
}

// Echoes info about air system
void regulator::echoPressures()
{
    SERIAL_NEWLINE();
    SERIAL_PROTOCOL_KEY_VALUE("TP", getActualTankPressure());
    SERIAL_PROTOCOL_KEY_VALUE("RP", getPressureFeedback(1));
    SERIAL_PROTOCOL_KEY_VALUE("SRP", getCurrentSetPressure(1));
    SERIAL_PROTOCOL_KEY_VALUE("RP2", getPressureFeedback(2));
    SERIAL_PROTOCOL_KEY_VALUE_LN("SRP2", getCurrentSetPressure(2));
}

/*
 * Sets the number of regulators for this system
 */
void regulator::setNofRegs(uint8_t nof)
{
    if (nof > 0 && nof < NOF_MAX_REGULATORS) {
        nofRegulators = nof;
    }
}

/*
 * Sets the current fullscale pressure
 */
void regulator::setFullscalePressure(uint16_t kpa)
{
    fullScalePressure = kpa;
}

//------------------Calibration handling------------------//

// Defines the current version for calibrations for this module. Must be updated whenever axisStoreParams_t is changed
#    define AIR_SYSTEM_CALIB_VERSION ((uint16_t) (1))
static bool airSystemIsCalibrated = false;

/*
 * Defines which variables are to be saved
 */
typedef struct
{
    uint16_t version;
    uint8_t nofRegulators;
    int16_t regulatorFeedbackOffset[NOF_MAX_REGULATORS];
    int16_t regulatorSetOffset[NOF_MAX_REGULATORS];
    uint16_t fullScalePressure;
    uint16_t tankRequiredPressure;
    uint16_t tankFullPressure;
} pressureStoreParams_t;

/*
 * Loads the stored configuration for movement parameters
 */
void airSystemLoadConfig(bool resetParams /*= false*/)
{
    pressureStoreParams_t tmp;
    // Reset all parameters to default
    if (resetParams) {
        for (uint8_t i = 0; i < NOF_MAX_REGULATORS; i++) {
            regulatorFeedbackOffset[i] = 0;
            regulatorSetOffset[i] = 0;
        }
        fullScalePressure = PRESSURE_FULL_SCALE_REG_FEST;
        nofRegulators = NOF_MAX_REGULATORS;
        airTankSetThresholds(0, 0);
        airSystemIsCalibrated = false;
    } else {
        // Read all data
        const configErr_t err = configReadDataSet(CONFIG_GET_ADDR(CONFIG_SYS_PRESSURE),
                                                  AIR_SYSTEM_CALIB_VERSION,
                                                  (uint8_t*) (&tmp),
                                                  sizeof(pressureStoreParams_t));
        if (err != CONFIG_ERR_OK) {
            // Something went wrong. Echo error and reset variables to default
            SERIAL_PROTOCOLLNPGM("Air params load error");
            configEchoError(err);
            airSystemLoadConfig(true);
            return;
        } else // Load successful
        {
            for (uint8_t i = 0; i < NOF_MAX_REGULATORS; i++) {
                regulatorFeedbackOffset[i] = tmp.regulatorFeedbackOffset[i];
                regulatorSetOffset[i] = tmp.regulatorSetOffset[i];
            }
            fullScalePressure = tmp.fullScalePressure;
            nofRegulators = tmp.nofRegulators;
            tankRequiredPressure = tmp.tankRequiredPressure;
            tankFullPressure = tmp.tankFullPressure;
            airSystemIsCalibrated = true;
        }
    }
}

/*
 * Stores all current parameters to non-volatile memory
 */
void airSystemStoreConfig(bool reset /*= false*/)
{
    pressureStoreParams_t tmp;
    // Fill struct
    if (!reset) {
        tmp.version = AIR_SYSTEM_CALIB_VERSION;
        airSystemIsCalibrated = true;
    } else {
        tmp.version = 0;
        airSystemIsCalibrated = false;
    }
    for (uint8_t i = 0; i < NOF_MAX_REGULATORS; i++) {
        tmp.regulatorFeedbackOffset[i] = regulatorFeedbackOffset[i];
        tmp.regulatorSetOffset[i] = regulatorSetOffset[i];
    }
    tmp.fullScalePressure = fullScalePressure;
    tmp.nofRegulators = nofRegulators;
    tmp.tankRequiredPressure = tankRequiredPressure;
    tmp.tankFullPressure = tankFullPressure;
    // Write to memory
    configWriteDataSet(CONFIG_GET_ADDR(CONFIG_SYS_PRESSURE),
                       (uint8_t*) (&tmp),
                       sizeof(pressureStoreParams_t));
}

/*
 * Returns the calibration status of the air system
 */
bool airSystemGetCalibrationStatus()
{
    return airSystemIsCalibrated;
}

/*
 * Returns true if the regulator given exists. False otherwise
 */
static bool regulator::exists(uint8_t reg)
{
    if (reg >= 1 && reg <= nofRegulators) {
        return true;
    } else {
        return false;
    }
}

#endif // PNEUMATIC_SYSTEM