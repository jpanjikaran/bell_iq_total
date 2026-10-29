/*
 * build_config.h
 *
 *  Created on: Mar. 22, 2022
 *      Author: jasonp
 */

#ifndef BUILD_CONFIG_H_
#define BUILD_CONFIG_H_

#include "sl_bt_api.h"
#include "lora_xdot_radio.h"

//-----------------------------------------------------------------------------------
// Build Config Block Format - As saved at EEPROM_CONFIG_OFFSET
//
// At EEPROM_CONFIG_OFFSET within EEPROM is a block of 32 bytes reserved
// for fully describing the Sensor Firmware as it was built and flashed.  This includes:
//
//     Hardware
//         Radio Technology
//             Radio
//             Zone
//             Class
//         Processor Technology
//         Specific Sensor Components
//         Hardware Version
//
//     Firmware
//         Based on Specific Sensor Components
//         Firmware Compilation Version
//
// Purpose of Configuration Structure
//
//     It is not absolutely possible to physically examine the Sensor to
//     determine exactly what it is.  Firmware Version alone, is just one
//     example that is impossible know.  Of course, it is possible (in theory)
//     to put all details on a sticker on the Sensor.
//
//     Unfortunately, a Sticker can't be read by Firmware.  By writing the
//     configuration into EEPROM, it can be queried by the firmware for testing
//     purposes, runtime configuration at initialization, and even dynamic
//     sending of relevant configuration items via radio to the cloud.
//
//     This configuration is written once when the product is built and the
//     firmware is flashed by the Radio Bridge Programming App.  Of course, it
//     is possible (in theory) to re-flash a different Firmware Version, BUT,
//     this is certainly not the plan.  Firmware (and this configuration) is
//     flashed exactly once when the Sensor is built.  Reprogramming firmware
//     (and EEPROM) is expensive and labor intensive endeavor.
//
//     While it is possible (in theory) for the running Sensor Firmware to
//     modify all or parts of this configuration, that is NOT the plan.  This
//     configuration should always be considered STATIC configuration at
//     build time.
//
//----------------
//
//  ##############################################################################################
//  #                                                                                            #
//  #                                     IMPORTANT NOTE                                         #
//  #                                                                                            #
//  ##############################################################################################
//  #                                                                                            #
//  # All changes to this file requires corresponding changes to the ProgrammingApp and TestApp  #
//  #                                                                                            #
//  ##############################################################################################
//
//----------------------------------------
// Version
//
// Because it is difficult to predict the future, the following byte is used for built in
// extensibility allowing for future Configuration Versions.
#define   CONFIG_VERSION                  api_eeprom_read_byte(CONFIG_VERSION_OFFSET)
#define   CONFIG_VERSION_OFFSET           (EEPROM_CONFIG_OFFSET + 0x00)  // 1 byte length
  #define CONFIG_VERSION_0                0xFF  // Initial Format Version of Configuration (used for first 300 sensors)
  #define CONFIG_VERSION_1                0x01  // First Format Version of Configuration (after first 300 sensors)
  #define CONFIG_VERSION_2                0x02  // Second Format Version of Configuration (Multi Sensors/Device, Multi DOWNLINKS, etc)
  #define CONFIG_VERSION_3                0x03  // Placeholder for future
  #define CONFIG_VERSION_UNDEFINED        0xFE  // 0xFE because 0xFF in EEPROM is uninitialized

//----------------------------------------
// Release
//
// Necessary to prevent and protect against unintentional release of firmware not built and
// recorded by the official build process.  The ProgrammingApp and TestApp will visually indicate
// anything other than a PRODUCTION release which can only be generated from firmware built
// from files checked into the library.
#define   CONFIG_RELEASE                  api_eeprom_read_byte(CONFIG_RELEASE_OFFSET)
#define   CONFIG_RELEASE_OFFSET           (EEPROM_CONFIG_OFFSET + 0x01)  // 1 byte length
  #define CONFIG_RELEASE_PRODUCTION       0x01  // Official PRODUCTION release by manufacturing (from fully automated build process)
  #define CONFIG_RELEASE_STAGING          0x02  // Intermediate STAGING test release that may evolve to PRODUCTION status (from fully automated build process)
  #define CONFIG_RELEASE_DEVELOPMENT      0x03  // Rapid DEVELOPMENT trial of unknown and untracked origins (from manual/custom build process)
  #define CONFIG_RELEASE_UNDEFINED        0xFE  // 0xFE because 0xFF in EEPROM is uninitialized

//----------------------------------------
// Radio Technologies
#define   CONFIG_RADIO                    api_eeprom_read_byte(CONFIG_RADIO_OFFSET)
#define   CONFIG_RADIO_OFFSET             (EEPROM_CONFIG_OFFSET + 0x02)  // 1 byte length
  #define CONFIG_RADIO_SIGFOX             0x01 // deprecated
  #define CONFIG_RADIO_SUBGIG             0x02
  #define CONFIG_RADIO_LORA               0x03
  #define CONFIG_RADIO_NBIOT              0x04
  #define CONFIG_RADIO_UNDEFINED          0xFE  // 0xFE because 0xFF in EEPROM is uninitialized

//----------------------------------------
// Radio Zones
#define   CONFIG_ZONE                     api_eeprom_read_byte(CONFIG_ZONE_OFFSET)
#define   CONFIG_ZONE_OFFSET              (EEPROM_CONFIG_OFFSET + 0x03)  // 1 byte length
  #define CONFIG_ZONE_LORA_US             0x01 // Note that it is OK to share these values with Sigfox and Subgig.  We only have lower nibble available to send in manf_test in test_messages
  #define CONFIG_ZONE_LORA_EU             0x02
  #define CONFIG_ZONE_LORA_AU             0x03
  #define CONFIG_ZONE_LORA_IN             0x04

  #define CONFIG_ZONE_UNDEFINED           0xFE  // 0xFE because 0xFF in EEPROM is uninitialized

//----------------------------------------
// Radio Classes
#define   CONFIG_CLASS                    api_eeprom_read_byte(CONFIG_CLASS_OFFSET)
#define   CONFIG_CLASS_OFFSET             (EEPROM_CONFIG_OFFSET + 0x04)  // 1 byte length
  #define CONFIG_CLASS_SIGFOX_1           0x01
  #define CONFIG_CLASS_SIGFOX_2           0x02
  #define CONFIG_CLASS_SIGFOX_3           0x03

  #define CONFIG_CLASS_UNDEFINED          0xFE  // 0xFE because 0xFF in EEPROM is uninitialized

//----------------------------------------
// Processor Targets
#define   CONFIG_PROCESSOR                api_eeprom_read_byte(CONFIG_PROCESSOR_OFFSET)
#define   CONFIG_PROCESSOR_OFFSET         (EEPROM_CONFIG_OFFSET + 0x05)  // 1 byte length
  #define CONFIG_PROCESSOR_16F18325       0x01
  #define CONFIG_PROCESSOR_16LF18325      0x02
  #define CONFIG_PROCESSOR_16LF18326      0x03
  #define CONFIG_PROCESSOR_16LF15356      0x04
  #define CONFIG_PROCESSOR_18LF26K40      0x05  //  64KB Flash, 28 pin
  #define CONFIG_PROCESSOR_18LF27K40      0x06  // 128KB Flash, 28 pin
  #define CONFIG_PROCESSOR_18LF46K40      0x07  //  64KB Flash, 40 pin
  #define CONFIG_PROCESSOR_18LF47K40      0x08  // 128KB Flash, 40 pin

  #define CONFIG_PROCESSOR_UNDEFINED      0xFE  // 0xFE because 0xFF in EEPROM is uninitialized

//----------------------------------------
// Sensor Types
#define   CONFIG_SENSOR                   api_eeprom_read_byte(CONFIG_SENSOR_OFFSET)
#define   CONFIG_SENSOR_OFFSET            (EEPROM_CONFIG_OFFSET + 0x06)  // 1 byte length
      // Proximity
  #define CONFIG_SENSOR_DWS               0x01  // Door Window
  #define CONFIG_SENSOR_DWS_HS            0x02  // Door Window - High Security
  #define CONFIG_SENSOR_CON               0x03  // Contact
      // Temperaure
  #define CONFIG_SENSOR_TEMP_NOP          0x04  // Temperature - No Probe
  #define CONFIG_SENSOR_TEMP_EXT          0x05  // Temperature - External Probe
      // Push Button
  #define CONFIG_SENSOR_BUTTON_1          0x06  // Push Button - Single
  #define CONFIG_SENSOR_BUTTON_2          0x07  // Push Button - Dual
      // Movement
  #define CONFIG_SENSOR_ABM               0x08  // Acceleration based motion
  #define CONFIG_SENSOR_TILT              0x09  // Tilt
      // Liquid
  #define CONFIG_SENSOR_WAT               0x0A  // Water
  #define CONFIG_SENSOR_TLF               0x0B  // Tank Level Float
      // Acoustic
  #define CONFIG_SENSOR_GB                0x0C  // Glass Break
      // Optical
  #define CONFIG_SENSOR_AL                0x0D  // Ambient Light
      // Air
  #define CONFIG_SENSOR_ATH               0x0E  // Air Temperature and Humidity


      // New Gen 2 Sensors
  #define CONFIG_SENSOR_TILT_HP           0x0F  // Tilt High Precision
  #define CONFIG_SENSOR_ULTRASONIC        0x10  // Ultra Sonic Level
  #define CONFIG_SENSOR_420MA             0x11  // 4-20 mA Loop
  #define CONFIG_SENSOR_ATH_EXT           0x12  // Air Temperature and Humidity - External Probe
  #define CONFIG_SENSOR_TC                0x13  // Thermocouple
  #define CONFIG_SENSOR_VOLTMETER         0x14  // Voltmeter
  #define CONFIG_SENSOR_CUST              0x15  // Custom Sensor specifically for Customers needing unique functionality
  #define CONFIG_SENSOR_GPS               0x16  // GPS tracker
  #define CONFIG_SENSOR_HW5800            0x17  // honeywell sensors
  #define CONFIG_SENSOR_MAG               0x18  // Magnetometer
  #define CONFIG_SENSOR_TEMP_INT          0x19  // Temperature - ath INTernal to enclosure, but only TEMP is sent
  #define CONFIG_SENSOR_COMPASS           0x1A  // Compass
  #define CONFIG_SENSOR_VWS               0x1B  // Vaisala Weather Station
  #define CONFIG_SENSOR_VSHB_CHAN1       0x1C  // Vibration sensor high bandwicth Chan1
  #define CONFIG_SENSOR_VSHB_CHAN2       0x1D  // Vibration sensor high bandwicth Chan2
  #define CONFIG_SENSOR_VSHB_CHAN3       0x1E  // Vibration sensor high bandwicth Chan3
  #define CONFIG_SENSOR_VSHB_CHAN4       0x1F  // Vibration sensor high bandwicth Chan4
  #define CONFIG_SENSOR_VSHB_FFT         0x20  // Vibration sensor high bandwicth condensed fft

// The following are a collection of temporary use Generic Sensors.
// They are to be used for the rapid introduction of new sensor types until the AppTools
// can be updated with the new official names.  AND an official code (0x80 -> 0x23 for example) selected.
// We must never ship these to the field because otherwise it will be impossible to reassign the offical codes.
#define CONFIG_SENSOR_GENERIC0           0x80  // Generic Sensor 0
#define CONFIG_SENSOR_GENERIC1           0x81  // Generic Sensor 1
#define CONFIG_SENSOR_GENERIC2           0x82  // Generic Sensor 2
#define CONFIG_SENSOR_GENERIC3           0x83  // Generic Sensor 3
#define CONFIG_SENSOR_GENERIC4           0x84  // Generic Sensor 4
#define CONFIG_SENSOR_GENERIC5           0x85  // Generic Sensor 5
#define CONFIG_SENSOR_GENERIC6           0x86  // Generic Sensor 6
#define CONFIG_SENSOR_GENERIC7           0x87  // Generic Sensor 7


      // New Button type, that will probably never become a product...
  #define CONFIG_SENSOR_BUTTON_3          0xF0  // Push Button - Triple  (Probably never a real product, but we need a Sensor Config for now)


      // ERROR - When config_sensor stored into the EEPROM is not identical to the
      // built firmware flashed to the sensor.  This value is returned during manufacturing
      // test time instead of the value stored into EEPROM configuration.  This is likely
      // the result of a serious build problem where the generated file name (*.hex) is
      // not consistent with what has been built.
  #define CONFIG_SENSOR_ERROR_CONFIG_TIME_NOT_SAME_AS_BUILD_TIME 0xFD
      //
  #define CONFIG_SENSOR_UNDEFINED         0xFE  // 0xFE because 0xFF in EEPROM is uninitialized

//----------------------------------------
// Hardware Version (2 BYTES !!! - Most Significant Byte is at (offset), Least Significant Byte is at (offset + 1))
#define   CONFIG_HARDWARE_VERSION         api_eeprom_read_u16_big_endian(CONFIG_HARDWARE_VERSION_OFFSET)
#define   CONFIG_HARDWARE_VERSION_OFFSET  (EEPROM_CONFIG_OFFSET + 0x07)  // 2 BYTE LENGTH !!!

  // The follow evaluates to a constant uint16_t at compile time as long as its 2 parameters are constants.
  // If the parameters are variables, then it generates code that is evaluated at run-time
  #define CONFIG_HARDWARE_VERSION_GEN(major,minor) \
     ((((major) & 0x000F) << 4) | (((minor) & 0x000F) << 0))

//----------------------------------------
// Firmware Version (2 BYTES !!! - Most Significant Byte is at (offset), Least Significant Byte is at (offset + 1))
#define   CONFIG_FIRMWARE_VERSION         api_eeprom_read_u16_big_endian(CONFIG_FIRMWARE_VERSION_OFFSET)
#define   CONFIG_FIRMWARE_VERSION_OFFSET  (EEPROM_CONFIG_OFFSET + 0x09)  // 2 BYTE LENGTH !!!

  // The follow evaluates to a constant uint16_t at compile time as long as its 3 parameters are constants.
  // If the parameters are variables, then it generates code that is evaluated at run-time
  #define CONFIG_FIRMWARE_VERSION_GEN(major,minor,build) \
     ((0x8000) | (((major) & 0x001F) << 10) | (((minor) & 0x001F) << 5) | (((build) & 0x1F) << 0))

//----------------------------------------
// Wall Power Options
#define   CONFIG_POWER                    api_eeprom_read_byte(CONFIG_POWER_OFFSET)
#define   CONFIG_POWER_OFFSET             (EEPROM_CONFIG_OFFSET + 0x0B)  // 1 byte length
  #define   CONFIG_POWER_EXT              0x80  // Power Source is External to the Sensor, MSB == 1 is mask for External

  #define   CONFIG_POWER_INT_BAT          0x01  // Power Source is Internal to the Sensor
  #define   CONFIG_POWER_EXT_12V          0x81  // Power Source is External to the Sensor, 12 Volts DC (via cable)
  #define   CONFIG_POWER_EXT_24V          0x82  // Power Source is External to the Sensor, 24 Volts DC (via cable)

//----------------------------------------
// Multi Sensor List  (4 CONFIG_SENSOR bytes follow.  The number of valid sensors can be found in CONFIG_MULTI_SENSOR_COUNT
#define   CONFIG_MULTI_SENSOR_0           api_eeprom_read_byte(CONFIG_MULTI_SENSOR_0_OFFSET)
#define   CONFIG_MULTI_SENSOR_0_OFFSET    (EEPROM_CONFIG_OFFSET + 0x0C)  // 1 byte length

#define   CONFIG_MULTI_SENSOR_1           api_eeprom_read_byte(CONFIG_MULTI_SENSOR_1_OFFSET)
#define   CONFIG_MULTI_SENSOR_1_OFFSET    (EEPROM_CONFIG_OFFSET + 0x0D)  // 1 byte length

#define   CONFIG_MULTI_SENSOR_2           api_eeprom_read_byte(CONFIG_MULTI_SENSOR_2_OFFSET)
#define   CONFIG_MULTI_SENSOR_2_OFFSET    (EEPROM_CONFIG_OFFSET + 0x0E)  // 1 byte length

#define   CONFIG_MULTI_SENSOR_3           api_eeprom_read_byte(CONFIG_MULTI_SENSOR_3_OFFSET)
#define   CONFIG_MULTI_SENSOR_3_OFFSET    (EEPROM_CONFIG_OFFSET + 0x0F)  // 1 byte length

//----------------------------------------
// Multi Sensor Count  (between 1 and 4 Sensors are supported per device when the CONFIG_VERSION == 2)
#define   CONFIG_MULTI_SENSOR_COUNT       api_eeprom_read_byte(CONFIG_MULTI_SENSOR_COUNT_OFFSET)
#define   CONFIG_MULTI_SENSOR_COUNT_OFFSET   (EEPROM_CONFIG_OFFSET + 0x10)  // 1 byte length

//----------------------------------------
// CONFIG_DEVICE (often same as CONFIG_SENSOR, but there up to N CONFIG_SENSORs per CONFIG_DEVICE)
#define   CONFIG_DEVICE_ID                api_eeprom_read_byte(CONFIG_DEVICE_ID_OFFSET)
#define   CONFIG_DEVICE_ID_OFFSET         (EEPROM_CONFIG_OFFSET + 0x11)  // 1 byte length

//----------------------------------------
// FAMILY
#define   CONFIG_FAMILY                   api_eeprom_read_byte(CONFIG_FAMILY_OFFSET)
#define   CONFIG_FAMILY_OFFSET            (EEPROM_CONFIG_OFFSET + 0x12)  // 1 byte length

  #define CONFIG_FAMILY_RBSx01            1  // gen1 RBS301 family  (gen1 original)
  #define CONFIG_FAMILY_RBSx02            2  // gen1 RBS302 family  (gen1 glass-break???)
  #define CONFIG_FAMILY_RBSx03            3  // gen2 RBS303 family  (gen2 version of gen1 RBS301)
  #define CONFIG_FAMILY_RBSx04            4  // gen2 RBS304 family  (gen2 Push Buttons)
  #define CONFIG_FAMILY_RBSx05            5  // gen2 RBS305 family  (gen2 ATH)
  #define CONFIG_FAMILY_RBSx06            6  // gen2 RBS306 family  (gen2 Industrial)
  #define CONFIG_FAMILY_RBSx07            7  // gen2 RBS306 family  (gen2 Industrial)

//----------------------------------------
// CONFIG_RADIO_VERSION_OFFSET
#define   CONFIG_RADIO_VERSION                   api_eeprom_read_byte(CONFIG_RADIO_VERSION_OFFSET)
#define   CONFIG_RADIO_VERSION_OFFSET            (EEPROM_CONFIG_OFFSET + 0x13)  // 1 byte length

  #define CONFIG_RADIO_VERSION_GENERIC           1
  #define CONFIG_RADIO_VERSION_MURATA_1_0_02     2
  #define CONFIG_RADIO_VERSION_MURATA_1_1_03     3
  #define CONFIG_RADIO_VERSION_XDOT              4
  #define CONFIG_RADIO_VERSION_UNDEFINED         0xFE // 0xFE because 0xFF in EEPROM is uninitialized

//----------------------------------------
// CONFIG_RADIO_FEATURE_OFFSET
#define   CONFIG_RADIO_FEATURE                  api_eeprom_read_byte(CONFIG_RADIO_FEATURE_OFFSET)
#define   CONFIG_RADIO_FEATURE_OFFSET            (EEPROM_CONFIG_OFFSET + 0x14)  // 1 byte length

  #define CONFIG_RADIO_FEATURE_GENERIC           1
  #define CONFIG_RADIO_FEATURE_1SJ               2
  #define CONFIG_RADIO_FEATURE_UNDEFINED         0xFE // 0xFE because 0xFF in EEPROM is uninitialized

//#define   CONFIG_RADIO_FEATURE                  api_eeprom_read_byte(CONFIG_RADIO_FEATURE_OFFSET)
#define   CONFIG_LORA_ENABLE_OFFSET            (EEPROM_CONFIG_OFFSET + 0x15)  // 1 byte length
#define   CONFIG_LORA_HB_OFFSET            (EEPROM_CONFIG_OFFSET + 0x16)  // 1 byte length

//  #define CONFIG_RADIO_FEATURE_GENERIC           1
//  #define CONFIG_RADIO_FEATURE_1SJ               2
//  #define CONFIG_RADIO_FEATURE_UNDEFINED         0xFE // 0xFE because 0xFF in EEPROM is uninitialized

void print_device_config(void);
//void api_eeprom_read(uint16_t address);


#endif /* BUILD_CONFIG_H_ */
