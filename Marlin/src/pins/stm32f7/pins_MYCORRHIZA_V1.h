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

#define BOARD_INFO_NAME "CELLINK_MYCORRHIZA_V1"
#define DEFAULT_MACHINE_NAME "Exocyte"

#if NO_EEPROM_SELECTED
#define FLASH_EEPROM_EMULATION // Use Flash-based EEPROM emulation
#endif

#define STEP_TIMER 4
#define TEMP_TIMER 14

#define MSERIAL2_TX_PIN PA2
#define MSERIAL2_RX_PIN PA3

//
// Steppers
//
#define X_STEP_PIN PD12
#define X_DIR_PIN PD10
#define X_ENABLE_PIN PD11
#define X_STOP_PIN PD9
#define X_HARDWARE_SERIAL  MSerial3
#define X_SLAVE_ADDRESS  0

#define Y_STEP_PIN PD13
#define Y_DIR_PIN PB15
#define Y_ENABLE_PIN PD8
#define Y_STOP_PIN PB14
#define Y_HARDWARE_SERIAL  MSerial3
#define Y_SLAVE_ADDRESS  2

#define Y2_STEP_PIN PD14
#define Y2_DIR_PIN PG3
#define Y2_ENABLE_PIN PG2
#define Y2_STOP_PIN PG4
#define Y2_HARDWARE_SERIAL MSerial3
#define Y2_SLAVE_ADDRESS 1

#define Z_STEP_PIN PD15
#define Z_DIR_PIN PG6
#define Z_ENABLE_PIN PG5
#define Z_STOP_PIN PG7
#define Z_HARDWARE_SERIAL  MSerial3
#define Z_SLAVE_ADDRESS  3

#define PROBE_EN_PIN PC11
#define PROBE_INDEX_PIN PC12
#define PROBE_STOP_PIN PD2
#define PROBE_HARDWARE_SERIAL MSerial5
#define PROBE_SERIAL_ADDRESS 1

#define PC_STEP_PIN PB9
#define PC_DIR_PIN PB10
#define PC_ENABLE_PIN PE15
#define PC_STOP_PIN PE14
#define PC_HARDWARE_SERIAL MSerial5
#define PC_SLAVE_ADDRESS 2

#define SIMPLE_TMC_HW_SERIAL MSerial5
#define USING_HW_SERIAL5 1

//
// bed leveling and calibration
//

#define OPTICAL_SENSOR_1_PIN PC4
#define OPTICAL_SENSOR_2_PIN PC5

#define Z_MIN_PROBE_PIN PC6

//
// temperature sensing and control
//

#define PRINTBED_TEMP_SCL_PIN PB6
#define PRINTBED_TEMP_SDA_PIN PB7

#define HEATER_BED_PIN PE9
#define HEATER_BED_2_PIN PE11

//
// pneumatics
//

#define PRESSURE_REGULATOR_PIN PA7

#define PRESSURE_REGULATOR_SENSE_PIN PB2
#define GRIPPER_VACUUM_PIN PF12
#define PRESSURE_TANK_PIN PF11

#define PRESSURE_PUMP_EN_PIN PE8

#define PRESSURE_VALVE_LID_PIN PF13
#define PRESSURE_VALVE_PUMP_IN_PIN PF14
#define PRESSURE_VALVE_PUMP_OUT_PIN PF15
#define PRESSURE_VALVE_C1_PIN PG0
#define PRESSURE_VALVE_C2_PIN PG1
#define PRESSURE_VALVE_C3_PIN PE7
#define PRESSURE_VALVE_CLOSE_LEVEL LOW

//
// leds
//

#define NEOPIXEL_PIN PF0 // UI LED bar
#define NEOPIXEL2_PIN PF1 // Chamber LED bar

//
// cartridge station
//

#define CHANTARELLE_SUPPORT
#define CHANT_SERIAL MSerial2
#define USING_HW_SERIAL2 1
#define CHANT_RTS_PIN PA1
#define CHANT_IRQ_PIN PA4

//
// Photocuring LEDs
//

#define PC_365_PIN PG10
#define PC_400_PIN PG11
#define PC_480_PIN PG12
#define PC_520_PIN PG13

#define PC_FEEDBACK PG14

//
// misc
//

#define FREEZE_PIN PA8