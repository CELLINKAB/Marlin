/*!
 * @file    TMP117.cpp
 * @author  Nils Minor
 * 
 * @license  GNU GENERAL PUBLIC LICENSE (see license.txt)
 * 
 * v1.0.0   - Initial library version
 * 
 * 
 */

#include "TMP117.h"
#include <Wire.h>

/**
 * @brief Construct a new template<typename Bus>
TMP117<Bus>::TMP117 object
 * 
 * @param addr typesafe address
 * @param alt_bus alternative wire implementation - must live longer than resulting object! Ideally static lifetime
 */
template<typename Bus>
TMP117<Bus>::TMP117(TMPAddr addr, Bus& alt_bus)
    : bus(alt_bus)
    , address(static_cast<uint8_t>(addr))
    , alert_pin(-1)
    , alert_type(TMP117_ALERT::NOALERT)
    , newDataCallback(nullptr)
{
  bus.begin();
}

/**
 * @brief Construct a new template<typename Bus>
TMP117<Bus>::TMP117 object
 * 
 * @param addr 
 * @param alt_bus alternative wire implementation - must live longer than resulting object! Ideally static lifetime
 */
template<typename Bus>
TMP117<Bus>::TMP117(uint8_t addr, Bus& alt_bus)
    : bus(alt_bus)
    , address(addr)
    , alert_pin(-1)
    , alert_type(TMP117_ALERT::NOALERT)
    , newDataCallback(nullptr)
{
  bus.begin();
}

/*!
    @brief   Initialize in default mode 
    @param   _newDataCallback   callback function will be called when new data is available
*/
template<typename Bus> void
TMP117<Bus>::init(void (*_newDataCallback)(void))
{
    setConvMode(TMP117_CMODE::CONTINUOUS);
    setConvTime(TMP117_CONVT::C125mS);
    setAveraging(TMP117_AVE::AVE8);
    setAlertMode(TMP117_PMODE::DATA);
    setOffsetTemperature(0);

    newDataCallback = _newDataCallback;
}

/*!
    @brief    Read configuration register and handle events.
              Should be called in loop in order to call callback functions 
*/
template<typename Bus> 
void TMP117<Bus>::update(void)
{
    readConfig();
}

/*!
    @brief   Performs a soft reset. All default values will be loaded to the configuration register
*/
template<typename Bus>
void TMP117<Bus>::softReset(void)
{
    uint16_t reg_value = 0;
    reg_value |= 1UL << 1;
    writeConfig(reg_value);
}

/*!
    @brief   Set alert pin mode 
    
    @param   mode TMP117_PMODE [Thermal-Alert-Data]
*/
template<typename Bus>
void TMP117<Bus>::setAlertMode(TMP117_PMODE mode)
{
    uint16_t reg_value = readConfig();
    if (mode == TMP117_PMODE::THERMAL) {
        reg_value |= 1UL << 4;    // change to thermal mode
        reg_value &= ~(1UL << 2); // set pin as alert flag
        reg_value &= ~(1UL << 3); // alert pin low active
    } else if (mode == TMP117_PMODE::ALERT) {
        reg_value &= ~(1UL << 4); // change to alert mode
        reg_value &= ~(1UL << 2); // set pin as alert flag
        reg_value &= ~(1UL << 3); // alert pin low active
    } else if (mode == TMP117_PMODE::DATA) {
        reg_value |= 1UL << 2; // set pin as data ready flag
    }
    writeConfig(reg_value);
}

/*!
    @brief   Set alert callback function and ISR pin
    @param   *alert_callback  callback function
    @param   pin callback pin (INT?)
*/
template<typename Bus>
void TMP117<Bus>::setalertCallback(void (*alert_callback)(void), uint8_t pin)
{
    alert_pin = pin;
    pinMode(pin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(pin),
                    alert_callback,
                    FALLING); // Sets up pin 2 to trigger "alert" ISR when pin changes H->L and L->H
}

/*!
    @brief    Set alert temperature
    
    @param    lowtemp   low boundary alert temperature
    @param    hightemp  high boundary alert temperature  
*/
template<typename Bus>
void TMP117<Bus>::setalertTemperature(double lowtemp, double hightemp)
{
    uint16_t high_temp_value = hightemp / TMP117_RESOLUTION;
    uint16_t low_temp_value = lowtemp / TMP117_RESOLUTION;

    i2cWrite2B(TMP117_REG_TEMP_HIGH_LIMIT, high_temp_value);
    i2cWrite2B(TMP117_REG_TEMP_LOW_LIMIT, low_temp_value);
}

/*!
    @brief    Set conversion mode
    
    @param    cmode   ::TMP117_CMODE [CONTINUOUS-SHUTDOWN-ONESHOT]
*/
template<typename Bus>
void TMP117<Bus>::setConvMode(TMP117_CMODE cmode)
{
    uint16_t reg_value = readConfig();
    reg_value &= ~((1UL << 11) | (1UL << 10));                      // clear bits
    reg_value = reg_value | (static_cast<int>(cmode) & 0x03) << 10; // set bits
    writeConfig(reg_value);
}

/*!
    @brief    Set conversion time
    
    @param    convtime  ::TMP117_CONVT [C15mS5-C125mS-C250mS-C500mS-C1S-C4S-C8S-C16S]
*/
template<typename Bus>
void TMP117<Bus>::setConvTime(TMP117_CONVT convtime)
{
    uint16_t reg_value = readConfig();
    reg_value &= ~((1UL << 9) | (1UL << 8) | (1UL << 7));             // clear bits
    reg_value = reg_value | (static_cast<int>(convtime) & 0x07) << 7; // set bits
    writeConfig(reg_value);
}
/*!
    @brief    Set averaging mode
    
    @param    ave  ::TMP117_AVE [NOAVE-AVE8-AVE32-AVE64]
*/
template<typename Bus>
void TMP117<Bus>::setAveraging(TMP117_AVE ave)
{
    uint16_t reg_value = readConfig();
    reg_value &= ~((1UL << 6) | (1UL << 5));                     // clear bits
    reg_value = reg_value | (static_cast<int>(ave) & 0x03) << 5; // set bits
    writeConfig(reg_value);
}

/*!
    @brief    Set offset temperature
    
    @param    double  target offset temperature  in the range of ±256°C  
*/
template<typename Bus>
void TMP117<Bus>::setOffsetTemperature(double offset)
{
    int16_t offset_temp_value = offset / TMP117_RESOLUTION;
    i2cWrite2B(TMP117_REG_TEMPERATURE_OFFSET, offset_temp_value);
}

/*!
    @brief    Set target temperature for calibration purpose
    
    @param    double  target temperature to calibrate to in the range of ±256°C  
*/
template<typename Bus>
void TMP117<Bus>::setTargetTemperature(double target)
{
    double actual_temp = getTemperature();
    double delta_temp = target - actual_temp;
    setOffsetTemperature(delta_temp);
}

/*!
    @brief    Read configuration register and handle events.

    @return   uint16_t  read value of the configuration regsiter          
*/
template<typename Bus>
uint16_t TMP117<Bus>::readConfig(void)
{
    uint16_t reg_value = i2cRead2B(TMP117_REG_CONFIGURATION);
    bool data_ready = reg_value >> 13 & 1UL;

    // following bits are a comment in order to not create compiler warnings
    // but might be used in the future for some purpose
    // bool eeprom_busy = reg_value >> 12 & 1UL;
    // bool high_alert = reg_value >> 15 & 1UL;
    // bool low_alert = reg_value >> 14 & 1UL;

    if (data_ready && newDataCallback != NULL)
        newDataCallback();

    if (reg_value >> 15 & 1UL) {
        alert_type = TMP117_ALERT::HIGHALERT;
    } else if (reg_value >> 14 & 1UL) {
        alert_type = TMP117_ALERT::LOWALERT;
    } else {
        alert_type = TMP117_ALERT::NOALERT;
    }

    //printConfig ( reg_value );

    return reg_value;
}

/*!
    @brief    Returns the recalculated temperature
    
    @return   double  temperature in °C
*/
template<typename Bus>
double TMP117<Bus>::getTemperature(void)
{
    int16_t temp = i2cRead2B(TMP117_REG_TEMPERATURE);
    return (temp * TMP117_RESOLUTION);
}
/*!
    @brief    Get Device Revision 
    
    @return   uint16_t device revision
*/
template<typename Bus>
uint16_t TMP117<Bus>::getDeviceRev(void)
{
    // read bits [15:12]
    uint16_t raw = i2cRead2B(TMP117_REG_DEVICE_ID);

    return ((raw >> 12) & 0x3);
}

/*!
    @brief    Get Device ID (always 0x117)
    
    @return   uint16_t  device ID
*/
template<typename Bus>
uint16_t TMP117<Bus>::getDeviceID(void)
{
    // read bits [11:0]
    uint16_t raw = i2cRead2B(TMP117_REG_DEVICE_ID);
    return (raw & 0x0fff);
}

/*!
    @brief    Returns the information which alert type happend
    
    @return   TMP117_ALERT [NoAlert-HighTempAlert-LowTempAlert]
*/
template<typename Bus>
TMP117_ALERT TMP117<Bus>::getAlertType(void)
{
    return alert_type;
}

/*!
    @brief    Returns the content of the offset register in °C
    
    @return   double  offset temperature in °C
*/
template<typename Bus>
double TMP117<Bus>::getOffsetTemperature(void)
{
    int16_t temp = i2cRead2B(TMP117_REG_TEMPERATURE_OFFSET);
    return (temp * TMP117_RESOLUTION);
}

/*!
    @brief    Write data to EEPROM register
    
    @param    data        data to write to the EEPROM
    
    @param    eeprom_nr   represents the EEPROM number [1 - 3] 
*/
template<typename Bus>
void TMP117<Bus>::writeEEPROM(uint16_t data, uint8_t eeprom_nr)
{
    if (!EEPROMisBusy()) {
        unlockEEPROM();
        switch (eeprom_nr) {
        case 1:
            i2cWrite2B(TMP117_REG_EEPROM1, data);
            break;
        case 2:
            i2cWrite2B(TMP117_REG_EEPROM2, data);
            break;
        case 3:
            i2cWrite2B(TMP117_REG_EEPROM3, data);
            break;
        default:
            Serial.println("EEPROM value must be between 1 and 3");
        }
        lockEEPROM();
    } else {
        Serial.println("EEPROM is busy");
    }
}

/*!
    @brief    Read data from EEPROM register
    
    @param    eeprom_nr  represents the EEPROM number [1 - 3] 
    
    @return   uint16_t   read EEPROM data
*/
template<typename Bus>
uint16_t TMP117<Bus>::readEEPROM(uint8_t eeprom_nr)
{
    // read the 48 bit number from the EEPROM
    if (!EEPROMisBusy()) {
        uint16_t eeprom_data = 0;
        switch (eeprom_nr) {
        case 1:
            eeprom_data = i2cRead2B(TMP117_REG_EEPROM1);
            break;
        case 2:
            eeprom_data = i2cRead2B(TMP117_REG_EEPROM2);
            break;
        case 3:
            eeprom_data = i2cRead2B(TMP117_REG_EEPROM3);
            break;
        default:
            Serial.println("EEPROM value must be between 1 and 3");
        }
        return eeprom_data;
    } else {
        Serial.println("EEPROM is busy");
        return 0;
    }
}

/**************************************************************************/
/* ********************* Library internal functions  ******************** */
/**************************************************************************/


/*!
    @brief    Write two bytes (16 bits) to TMP117 I2C sensor
    
    @param    reg  target register
    @param    data data to write
*/
template<typename Bus>
void TMP117<Bus>::i2cWrite2B(uint8_t reg, uint16_t data)
{
    bus.beginTransmission(address);
    bus.write(reg);
    bus.write((data >> 8));
    bus.write((data & 0xff));
    bus.endTransmission();
    delay(10);
}

/*!
    @brief    Read two bytes (16 bits) from TMP117 I2C sensor
    
    @param    reg  target register to read from
    
    @return   uint16_t  read data
*/
template<typename Bus>
uint16_t TMP117<Bus>::i2cRead2B(uint8_t reg)
{
    uint8_t data[2] = {0};
    int16_t datac = 0;

    bus.beginTransmission(address);
    bus.write(reg);
    bus.endTransmission();
    bus.requestFrom((uint8_t) address, (uint8_t) 2);
    if (bus.available() <= 2) {
        data[0] = bus.read();
        data[1] = bus.read();
        datac = ((data[0] << 8) | data[1]);
    }
    return datac;
}

/*!
    @brief    Write configuration to config register.  Also store it
              to the EEPROM so the settings will be used on reboot.
    
    @param    config_data  configuration
*/
template<typename Bus>
void TMP117<Bus>::writeConfig(uint16_t config_data)
{
    if (!EEPROMisBusy()) {
        unlockEEPROM();
        i2cWrite2B(TMP117_REG_CONFIGURATION, config_data);
        lockEEPROM();
    } else {
        Serial.println("EEPROM is busy");
    }
}

/*!
    @brief    Prints configuration in user readable format
    
    @param    reg_value  configuration value
*/
template<typename Bus>
void TMP117<Bus>::printConfig()
{
    uint16_t reg_value = i2cRead2B(TMP117_REG_CONFIGURATION);
    Serial.println(reg_value, BIN);

    Serial.print("HIGH alert:  ");
    Serial.println((reg_value >> 15) & 0b1, BIN);
    Serial.print("LOW alert:   ");
    Serial.println((reg_value >> 14) & 0b1, BIN);
    Serial.print("Data ready:  ");
    Serial.println((reg_value >> 13) & 0b1, BIN);
    Serial.print("EEPROM busy: ");
    Serial.println((reg_value >> 12) & 0b1, BIN);
    Serial.print("MOD[1:0]:    ");
    Serial.println((reg_value >> 10) & 0b11, BIN);
    Serial.print("CONV[2:0]:   ");
    Serial.println((reg_value >> 7) & 0b111, BIN);
    Serial.print("AVG[1:0]:    ");
    Serial.println((reg_value >> 5) & 0b11, BIN);
    Serial.print("T/nA:        ");
    Serial.println((reg_value >> 4) & 0b1, BIN);
    Serial.print("POL:         ");
    Serial.println((reg_value >> 3) & 0b1, BIN);
    Serial.print("DR/Alert:    ");
    Serial.println((reg_value >> 2) & 0b1, BIN);
    Serial.print("Soft_Reset:  ");
    Serial.println((reg_value >> 1) & 0b1, BIN);
}
/*!
    @brief    Lock EEPROM, write protection
*/
template<typename Bus>
void TMP117<Bus>::lockEEPROM(void)
{
    // clear bit 15
    uint16_t code = 0;
    code &= ~(1UL << 15);
    i2cWrite2B(TMP117_REG_EEPROM_UNLOCK, code);
    delay(100);
}

/*!
    @brief    Unlock EEPROM, remove write protection
*/
template<typename Bus>
void TMP117<Bus>::unlockEEPROM(void)
{
    // set bit 15
    uint16_t code = 0;
    code |= 1UL << 15;
    i2cWrite2B(TMP117_REG_EEPROM_UNLOCK, code);
    delay(100);
}

/*!
    @brief    States if the EEPROM is busy
    
    @return   Ture if the EEPROM is busy, fals else
*/
template<typename Bus>
bool TMP117<Bus>::EEPROMisBusy(void)
{
    // Bit 14 indicates the busy state of the eeprom
    uint16_t code = i2cRead2B(TMP117_REG_EEPROM_UNLOCK);
    return (bool) ((code >> 14) & 0x01);
}

template class TMP117<TwoWire>;
