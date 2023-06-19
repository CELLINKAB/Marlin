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

#define BOARD_INFO_NAME "CELLINK_MYCORRHIZA_V1.1"
#define DEFAULT_MACHINE_NAME "Exocyte"

#if NO_EEPROM_SELECTED
#define FLASH_EEPROM_EMULATION // Use Flash-based EEPROM emulation
//#define DISABLE_FLASH_WRITE_PROTECTION // in case factory setting has it enabled
#endif

#define STEP_TIMER 5
#define TEMP_TIMER 14

//
// UART
//

#define MYCO_TMC_SOFT_SERIAL
#ifdef MYCO_TMC_SOFT_SERIAL
  #define X_SERIAL_TX_PIN PC10
  #define X_SERIAL_RX_PIN PC11
  #define Y_SERIAL_TX_PIN PC10
  #define Y_SERIAL_RX_PIN PC11
  #define Y2_SERIAL_TX_PIN PC10
  #define Y2_SERIAL_RX_PIN PC11
  #define Z_SERIAL_TX_PIN PC10
  #define Z_SERIAL_RX_PIN PC11

  #define TMC_BAUD_RATE 19200
  
  #undef MYCO_TMC_SOFT_SERIAL
#else
  #define X_HARDWARE_SERIAL  MSerial3
  #define Y_HARDWARE_SERIAL  MSerial3
  #define Y2_HARDWARE_SERIAL MSerial3
  #define Z_HARDWARE_SERIAL  MSerial3
#endif
//
// Steppers
//

#define X_STEP_PIN PD8
#define X_DIR_PIN PD10
#define X_ENABLE_PIN PB15
#define X_STOP_PIN PD9

#define X_SLAVE_ADDRESS  0

#define Y_STEP_PIN PD13
#define Y_DIR_PIN PD12
#define Y_ENABLE_PIN PD11
#define Y_STOP_PIN PD14
#define Y_SLAVE_ADDRESS  2

#define Y2_STEP_PIN PD15
#define Y2_DIR_PIN PG3
#define Y2_ENABLE_PIN PG2
#define Y2_STOP_PIN PG4

#define Y2_SLAVE_ADDRESS 1

#define Z_STEP_PIN PG8
#define Z_DIR_PIN PG6
#define Z_ENABLE_PIN PG5
#define Z_STOP_PIN PG7

#define Z_SLAVE_ADDRESS  3

#define PROBE_EN_PIN PE0
#define PROBE_INDEX_PIN PE1
#define PROBE_STOP_PIN PD2
#define SP_SERIAL_TX_PIN PB13
#define SP_SERIAL_RX_PIN PB12
//#define SP_HARDWARE_SERIAL TMC_UART2
#define PROBE_SERIAL_ADDRESS 2

#define PC_STEP_PIN PB11
#define PC_DIR_PIN PB10
#define PC_ENABLE_PIN PE15
#define PC_DIAG_PIN PE14
#define PC_ENDSTOP_PIN PD3 // physical switch, backup for DIAG
#define PC_SERIAL_TX_PIN PB13
#define PC_SERIAL_RX_PIN PB12
// #define PC_HARDWARE_SERIAL TMC_UART2
#define PC_SLAVE_ADDRESS 0

// #define SIMPLE_TMC_HW_SERIAL TMC_UART2
// #define USING_HW_SERIAL5 1

//
// bed leveling and calibration
//

#define OPTICAL_SENSOR_1_PIN PC5
#define OPTICAL_SENSOR_2_PIN PB0

#define Z_MIN_PROBE_PIN PB9 // SP_DET

//
// temperature sensing and control
//

#define PRINTBED_TEMP_SCL_PIN PB6
#define PRINTBED_TEMP_SDA_PIN PB7

#define HEATER_BED_PIN PE9 // TEM B
#define HEATER_BED_2_PIN PE11 // TEM A

#define HEATER_BED_CS_PIN PF10

#define HEATER_BED_INVERTING true

#define MYCO_HEATER
#define BED_TEMP_IS_BIDIRECTIONAL true

#define FAN_PIN PE13 // PB
#define FAN1_PIN PC7 // CC
#define FAN_MIN_PWM 255


#define E0_FAN_TACHO_PIN PE12 // PB
#define E1_FAN_TACHO_PIN PA8 // CC 1
#define E2_FAN_TACHO_PIN PC9 // CC 2
#define E3_FAN_TACHO_PIN PC8 // CC 3

//
// pneumatics
//

#define PRESSURE_REGULATOR_PIN PA5

#define PRESSURE_REGULATOR_SENSE_PIN PB1 // PREG_DAC
#define GRIPPER_VACUUM_PIN PC3 // PSENSE_1
#define PRESSURE_TANK_PIN PC2 // PSENSE_2

#define PRESSURE_PUMP_EN_PIN PE8

#define PRESSURE_VALVE_LID_PIN PF11 // V1
#define PRESSURE_VALVE_LID2_PIN PF12 // V2
#define PRESSURE_VALVE_PUMP_OUT_PIN PF13 // V3
#define PRESSURE_VALVE_C1_PIN PG0  // V4_1
#define PRESSURE_VALVE_C2_PIN PG1 // V4_2
#define PRESSURE_VALVE_C3_PIN PE7 // V4_3
#define PRESSURE_VALVE_CLOSE_LEVEL LOW

//
// leds
//

#define NEOPIXEL_PIN PF0 // UI LED bar
#define NEOPIXEL2_PIN PF1 // Chamber LED bar

//
// Photocuring LEDs
//

#define PC_365_PIN PG15
#define PC_400_PIN PG14
#define PC_480_PIN PG13
#define PC_520_PIN PG12

#define PC_PWM_PIN PB8

#define PC_FEEDBACK PG14

#define EXOCYTE_UV_CROSSLINKING

//
// cartridge station
//

#define CHANTARELLE_SUPPORT

#define CHANT_SERIAL MSerial2
#define USING_HW_SERIAL2 1
#define CHANT_RTS_PIN PA1

#define CHANT_IRQ1_PIN -1 //PC13
#define CHANT_IRQ2_PIN -1 //PC14

#define E0_ENABLE_INIT() NOOP
#define E0_DIR_INIT() NOOP
#define E0_STEP_INIT() pinMode(CHANT_IRQ1_PIN, OUTPUT)

//
// UVC sterilization
//

#define UVC_PWM_PIN PC6  // LED_DRV_PWM
#define UVC_PWM_INVERTING true

#define UVC_TFAULT_PIN PG11
#define UVC_TFAULT_ACTIVE_STATE LOW

#define UVC_INTERLOCK_PIN PF8

#define UVC_SWITCH_1_PIN PD4
#define UVC_SWITCH_2_PIN PD5
#define UVC_SWITCH_3_PIN PD6
#define UVC_SWITCH_4_PIN PD7
#define UVC_SWITCH_5_PIN PG9

#define UVC_SH_CS_1_PIN PA4
#define UVC_SH_CS_2_PIN PA0
#define UVC_SH_CS_3_PIN PA6
#define UVC_SH_CS_4_PIN PA7
#define UVC_SH_CS_5_PIN PC4

#define UVC_RELAY_PIN PC12

#define UVC_STERILIZATION

//
// misc
//

#define CS_24V_PIN PF9
#define CS_BED_24V_CS_PIN PF10

#define PSU_FAULT_PIN PG10

#define DOOR_PIN UVC_INTERLOCK_PIN
#define DOOR_SENSOR_INVERTING true
#define FREEZE_PIN PA10

#define LED_RED PE3
#define LED_GREEN PE2

#define MYCO_ID_PIN PC0 
#define BAMBOO_ID_PIN PC1

#define CALIBRATION_PIN PD0
#define CALIBRATION_GROUND_PIN PD1
#define CALIBRATION_PIN_INVERTING true // Set to true to invert the custom pin
#define CALIBRATION_PIN_PULLUP