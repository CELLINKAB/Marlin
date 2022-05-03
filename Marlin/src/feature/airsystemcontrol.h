
#ifndef AIRSYSTEMCONTROL_H_
#define AIRSYSTEMCONTROL_H_

#include "../inc/MarlinConfig.h"

#if ENABLED(PNEUMATIC_SYSTEM)

void airSystemInit();

bool airSourceControl(char airSource);
void airTankPressureControl();

void airControlPumpDisable();
void airControlPumpEnable();
bool airControlGetInternalPumpStatus();

uint16_t getActualTankPressure();
void airTankSetThresholds(uint16_t req, uint16_t full);
void airTankEchoThresholds();

void airSystemLoadConfig(bool resetParams = false);
void airSystemStoreConfig(bool reset = false);
bool airSystemGetCalibrationStatus();

namespace regulator {
uint8_t lookupRegulatorForPH(uint8_t ph);

uint16_t getPressureFeedback(uint8_t regulator = 1);

bool setOutputPressure(uint16_t pressureValue, uint8_t regulator);
uint16_t getCurrentSetPressure(uint8_t regulator);
void setPWM(uint16_t pwm, uint8_t regulator);
uint16_t convertmVToPWM(uint16_t mV);
uint16_t convertkPaTomV(uint16_t kpa);

void setRegFBOffset(int16_t kPa, uint8_t regulator);
int16_t getRegFBOffset(uint8_t regulator);
void setRegControlOffset(int16_t kPa, uint8_t regulator);
int16_t getRegControlOffset(uint8_t regulator);

void echoPressures();

void setNofRegs(uint8_t nof);
void setFullscalePressure(uint16_t kpa);
} // namespace regulator

#endif // PNEUMATIC_SYSTEM

#endif /* AIRSYSTEMCONTROL_H_ */
