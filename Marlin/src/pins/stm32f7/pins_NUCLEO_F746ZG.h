/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#if NOT_TARGET(STM32F767xx)
#error "Oops! Select an STM32F746 environment"
#endif

#define BOARD_INFO_NAME "NUCLEO-F746ZG"
#define DEFAULT_MACHINE_NAME "Prototype Board"

#if NO_EEPROM_SELECTED
#define FLASH_EEPROM_EMULATION // Use Flash-based EEPROM emulation
#endif

#if ENABLED(FLASH_EEPROM_EMULATION)
// Decrease delays and flash wear by spreading writes across the
// 128 kB sector allocated for EEPROM emulation.
// Not yet supported on F7 hardware
//#define FLASH_EEPROM_LEVELING
#endif

/**
 * Timer assignments
 *
 * TIM1 -
 * TIM2 - Hardware PWM (Fan/Heater Pins)
 * TIM3 - Hardware PWM (Servo Pins)
 * TIM4 - STEP_TIMER (Marlin)
 * TIM5 -
 * TIM6 - TIMER_TONE (variant.h)
 * TIM7 - TIMER_SERVO (variant.h)
 * TIM9 - TIMER_SERIAL (platformio.ini)
 * TIM10 - For some reason trips Watchdog when used for SW Serial
 * TIM11 -
 * TIM12 - lid gripper (temporary)
 * TIM13 -
 * TIM14 - TEMP_TIMER (Marlin)
 *
 */
#define STEP_TIMER 4
#define TEMP_TIMER 14

/**
 * These pin assignments are arbitrary and intending for testing purposes.
 * Assignments may not be ideal, and not every assignment has been tested.
 * Proceed at your own risk.
 *                                                            _CN7_
 *                                              (X_STEP) PC6 | · · | PB8 (I2C_SCL)
 *                                              (X_DIR) PB15 | · · | PB9 (I2C_SDA)
 *                                               (X_EN) PB13 | · · | AVDD
 *                 _CN8_                                PB12 | · · | GND
 *             NC | · · | PC8                (HEATER_0) PA15 | · · | PA5  (SCLK)
 *          IOREF | · · | PC9                   (BEEPER) PC7 | · · | PA6  (MISO)
 *          RESET | · · | PC10                           PB5 | · · | PA7  (MOSI)
 *          +3.3V | · · | PC11              (HEATER_BED) PB3 | · · | PD14 (pvalve_1)
 *            +5V | · · | PC12 TX                        PA4 | · · | PD15 (pvalve_2)
 *            GND | · · | PD2  RX                        PB4 | · · | PF12 (pvalve_3)
 *            GND | · · | PG2                                 ￣￣￣
 *            VIN | · · | PG3
 *                 ￣￣￣                                      _CN10
 *                                                      AVDD | · · | PF13 (BTN_EN1)
 *                 _CN9_                                AGND | · · | PE9  (CHANT_RTS)
 *   (TEMP_0) PA3 | · · | PD7                            GND | · · | PE11 (BTN_ENC)
 * (TEMP_BED) PC0 | · · | PD6 (XYZ_RX)                   PB1 | · · | PF14 (PROBE_LED_ON)
 *    (PROBE) PC3 | · · | PD5 (XYZ_TX)                   PC2 | · · | PE13 (PROBE_MFI)
 * (PRESSURE) PF3 | · · | PD4                    (Y2_EN) PF4 | · · | PF15 (PROBE_ERR)
 *(PRESSURE2) PF5 | · · | PD3                  (Y2_STEP) PB6 | · · | PG14  LG_TX/E_TX
 *           PF10 | · · | GND                   (Y2_DIR) PB2 | · · | PG9   LG_RX/E_TX
 *             NC | · · | PE2                            GND | · · | PE8   PROBE_TX
 *            PA7 | · · | PE4 (E_EN)           (RDP_EN) PD13 | · · | PE7   PROBE_RX
 *    (E_DIR) PF2 | · · | PE5 (E_STEP)       (RDP_STOP) PD12 | · · | GND
 *   (Y_STEP) PF1 | · · | PE6 (Y_EN)           (Z_STEP) PD11 | · · | PE10 (Z_EN)
 *    (Y_DIR) PF0 | · · | PE3 (Y_DIAG)           (Z_DIR) PE2 | · · | PE12 (Z_DIAG)
 *            GND | · · | PF8 (calibration)              GND | · · | PE14
 * (optical1) PD0 | · · | PF7 (X _STOP)                  PA0 | · · | PE15 (SERVO0)
 * (optical2) PD1 | · · | PF9 (E1_STOP)                  PB0 | · · | PB10 (FAN)
 *  (Y2_STOP) PG0 | · · | PG1 (E0_STOP)        (Z_PROBE) PE0 | · · | PB11 (FAN1)
 *                 ￣￣￣                                     ￣￣￣￣
 */

#if DISABLED(SENSORLESS_HOMING)
    #define X_MIN_PIN PF7
    #define X_MAX_PIN PD0
    #define Y_MIN_PIN PF9
    #define Y_MAX_PIN PD1
    #define Z_MIN_PIN PG1
    #define Z_MAX_PIN PG0
#else
    #define X_STOP_PIN PF7
    #define Y_STOP_PIN PE3
    #define Y2_STOP_PIN PG0
    #define Z_STOP_PIN PE12
    #define E0_STOP_PIN PG1
    #define E1_STOP_PIN PF9
    //#define E2_STOP_PIN PF7
#endif

//
// Steppers
//
#define X_STEP_PIN PC6
#define X_DIR_PIN PB15
#define X_ENABLE_PIN PB13

#define Y_STEP_PIN PF1
#define Y_DIR_PIN PF0
#define Y_ENABLE_PIN PE6

#define Y2_STEP_PIN PB6
#define Y2_DIR_PIN PB2
#define Y2_ENABLE_PIN PF4

#define Z_STEP_PIN PD11
#define Z_DIR_PIN PE2
#define Z_ENABLE_PIN PE10

#define E0_STEP_PIN PE5
#define E0_DIR_PIN PF2
#define E0_ENABLE_PIN PE4

#if HAS_TMC_UART

    #define X_HARDWARE_SERIAL  MSerial2
    #define Y_HARDWARE_SERIAL  MSerial2
    #define Y2_HARDWARE_SERIAL MSerial2
    #define Z_HARDWARE_SERIAL  MSerial2

    #define X_SLAVE_ADDRESS  0
    #define Y_SLAVE_ADDRESS  2
    #define Y2_SLAVE_ADDRESS 1
    #define Z_SLAVE_ADDRESS  3

    #define E0_HARDWARE_SERIAL MSerial6
    #define E0_SLAVE_ADDRESS 0

#else // HAS_TMC_UART
  #define X_CS_PIN PB12
  #define Y_CS_PIN PE3
  #define Y2_CS_PIN PC2
  #define Z_CS_PIN PE12
#endif // HAS_TMC_UART

// Servos
#define SERVO0_PIN PE15

// Pressure sensor
#define PRESSURE_SENSOR_PIN PF3
#define PRESSURE_SENSOR_2_PIN PF5

// Pressure valves
#define PRESSURE_VALVE_1_PIN PD14
#define PRESSURE_VALVE_2_PIN PD15
#define PRESSURE_VALVE_3_PIN


#if ENABLED(LID_GRIPPER_STATION)
    // #define LG_STEP_PIN         PD12
    // #define LG_DIR_PIN          PF2
    #define LG_INDEX_PIN        PF2
    #define LG_EN_PIN           PE4

    #define LG_HARDWARE_SERIAL MSerial2
    #ifndef USING_HW_SERIAL2
        #define USING_HW_SERIAL2 1
    #endif
    #define LG_SLAVE_ADDRESS    0
    #define LG_STOP_PIN         PE5

    //#define USING_HW_SERIAL4 1
#endif

#if ENABLED(OPTICAL_SURFACE_PROBE)
    #define OPT_SURF_IN_PIN PC3 // white
    #define OPT_SURF_ERR_PIN PF15 // brown
    #define OPT_SURF_MFI_PIN PE13// violet
    #define OPT_SURF_LED_ON_PIN PF14 // black

    #define OPT_SURF_HW_SERIAL MSerial7
    #ifndef USING_HW_SERIAL7
      #define USING_HW_SERIAL7 1
    #endif
#endif

#if ENABLED(OPTICAL_AUTOCAL)
  #define OPTICAL_SENSOR_1_PIN PD0
  #define OPTICAL_SENSOR_2_PIN PD1
#endif

#if ENABLED(STM_INEMO_IMU_SUPPORT)
  #define STM_INEMO_SDA_PIN PB9
  #define STM_INEMO_SCL_PIN PB8
  #define STM_INEMO_SAD_0_BIT 0
#endif

#if ENABLED(STEPPER_RETRACTING_PROBE)
  #define PROBE_EN_PIN PD13
  #define PROBE_STOP_PIN PD12
  #define PROBE_SERIAL_ADDRESS 2
#endif


// cartridge station communication
#define CHANTRELLE_SUPPORT
#if ENABLED(CHANTRELLE_SUPPORT)
  #define CHANT_SERIAL MSerial7
  #define USING_HW_SERIAL7 1
  #define CHANT_RTS_PIN PE9
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN PA3
//#define TEMP_BED_PIN PC0

#define HEATER_0_PIN PA15

#define CALIBRATION_PIN PF8
#define CALIBRATION_PIN_PULLUP

#define Z_MIN_PROBE_PIN PE0

#define LED_PIN LED_BLUE


