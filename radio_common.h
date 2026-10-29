/*
 * radio_common.h
 *
 *  Created on: Mar. 17, 2022
 *      Author: jasonp
 */

#ifndef RADIO_COMMON_H_
#define RADIO_COMMON_H_

/*
 Common functions for radio comm

  (c) 2017 Radio Bridge Inc. and its subsidiaries.
*/

#include "radio_tx_buf.h"
#include "sl_bt_api.h"

#define JOIN_FORCE_WAKEUP    true
#define JOIN_NO_FORCE_WAKEUP false
// message types
#define RADIO_RESET_EVENT                           0x00
#define RADIO_SUPERVISORY                           0x01
#define RADIO_TAMPER                                0x02
// note that reed switch and hall effect sensor are the same event 0x03
#define RADIO_EVENT_TYPE_REED_SWITCH                0x03
#define RADIO_EVENT_TYPE_HES                        0x03
#define RADIO_EVENT_TYPE_GLASS_BREAK                0x05
#define RADIO_EVENT_TYPE_PUSH_BUTTON                0x06
#define RADIO_EVENT_TYPE_CONTACT_SENSOR             0x07
#define RADIO_EVENT_TYPE_WATER_SENSOR               0x08
#define RADIO_EVENT_TYPE_TEMP_SENSOR                0x09
#define RADIO_EVENT_TYPE_TILT_SENSOR                0x0a
#define RADIO_EVENT_TYPE_AMBIENT_LIGHT_SENSOR       0x0b
#define RADIO_EVENT_TYPE_TANK_LEVEL_SENSOR          0x0c
#define RADIO_EVENT_TYPE_AIR_TEMP_HUMIDITY_SENSOR   0x0d
#define RADIO_EVENT_TYPE_ACCEL_MOTION_SENSOR        0x0e
#define RADIO_EVENT_TYPE_TILT_HP_SENSOR             0x0f
#define RADIO_EVENT_TYPE_ULTRASONIC_SENSOR          0x10
#define RADIO_EVENT_TYPE_420MA_SENSOR               0x11
#define RADIO_EVENT_TYPE_TC_SENSOR                  0x13
#define RADIO_EVENT_TYPE_VOLTMETER                  0x14
#define RADIO_EVENT_TYPE_CUST                       0x15   // Custom Sensor Event Type
#define RADIO_EVENT_TYPE_GPS                        0x16
#define RADIO_EVENT_TYPE_HW5800                     0x17
#define RADIO_EVENT_TYPE_MAG_SENSOR                 0x18
#define RADIO_EVENT_TYPE_TEMP_INT_SENSOR            0x19
#define RADIO_EVENT_TYPE_COMPASS_SENSOR             0x1a
#define RADIO_EVENT_TYPE_VWS                        0x1b
#define RADIO_EVENT_TYPE_VSHB_CH1                   0x1c
#define RADIO_EVENT_TYPE_VSHB_CH2                   0x1d
#define RADIO_EVENT_TYPE_VSHB_CH3                   0x1e
#define RADIO_EVENT_TYPE_VSHB_CH4                   0x1f
#define RADIO_EVENT_TYPE_VSHB_FFT                   0x20

#define RADIO_EVENT_TYPE_RESERVED_LOWER_START       0x40
// ###########################################################################
// ################  RESERVED BETWEEN THIS RANGE FOR NOW - intended for creating two byte RADIO_EVENTS and other special TBD things
// ###########################################################################
#define RADIO_EVENT_TYPE_RESERVED_UPPER_END         0xBF

// the custom event 0xf9 is followed by the customer ID defined below
#define RADIO_EVENT_TYPE_CUSTOM                     0xf9
#define RADIO_EVENT_TYPE_DEVICE_INFO                0xfa  // to xmit certain device info from saved in EEPROM
#define RADIO_EVENT_TYPE_LINK_QUALITY               0xfb
//#define RADIO_EVENT_TYPE_RATE LIMIT               0xfc  // we should retire 0xfc from use until we are sure it isn't used again
#define RADIO_EVENT_TYPE_DEBUG_MESSAGE              0xfd
#define RADIO_EVENT_TYPE_TEST_RESULT                0xfe
#define RADIO_EVENT_TYPE_DOWNLINK_ACK               0xff

//------------------------------------------------------------------------
//------------------------------------------------------------------------
//------------------------------------------------------------------------
// TRC - This is a convient location for table of strings for each EVENT
// If EUSART_TRC_ENABLE is not defined, then it is completely compiled out of the build
//
#ifdef EUSART_TRC_ENABLE
   #ifdef INCLUDE_ONLY_IN_TRC_C
      static const char * trc_lookup_radio_event_string(uint8_t radio_event)
      {
         switch (radio_event)
         {
            case RADIO_RESET_EVENT:                            return "RESET";  // keep all short names to aid formatting
            case RADIO_SUPERVISORY:                            return "SUPER";
            case RADIO_TAMPER:                                 return "TAMPER";
            case RADIO_EVENT_TYPE_HES:                         return "HALL";
            case RADIO_EVENT_TYPE_GLASS_BREAK:                 return "GLASS";
            case RADIO_EVENT_TYPE_PUSH_BUTTON:                 return "BUTTON";
            case RADIO_EVENT_TYPE_CONTACT_SENSOR:              return "CONTACT";
            case RADIO_EVENT_TYPE_WATER_SENSOR:                return "WATER";
            case RADIO_EVENT_TYPE_TEMP_SENSOR:                 return "TEMP";
            case RADIO_EVENT_TYPE_TILT_SENSOR:                 return "TILT";
            case RADIO_EVENT_TYPE_AMBIENT_LIGHT_SENSOR:        return "LIGHT";
            case RADIO_EVENT_TYPE_TANK_LEVEL_SENSOR:           return "TLF";
            case RADIO_EVENT_TYPE_AIR_TEMP_HUMIDITY_SENSOR:    return "ATH";
            case RADIO_EVENT_TYPE_ACCEL_MOTION_SENSOR:         return "ABM";
            case RADIO_EVENT_TYPE_TILT_HP_SENSOR:              return "TILT-HP";
            case RADIO_EVENT_TYPE_ULTRASONIC_SENSOR:           return "ULTRA";
            case RADIO_EVENT_TYPE_420MA_SENSOR:                return "420MA";
            case RADIO_EVENT_TYPE_TC_SENSOR:                   return "TC";
            case RADIO_EVENT_TYPE_VOLTMETER:                   return "VOLT";
            case RADIO_EVENT_TYPE_CUST:                        return "CUST";
            case RADIO_EVENT_TYPE_GPS:                         return "GPS";
            case RADIO_EVENT_TYPE_HW5800:                      return "HW-5800";
            case RADIO_EVENT_TYPE_MAG_SENSOR:                  return "MAG";
            case RADIO_EVENT_TYPE_VWS:                         return "VWS";
            case RADIO_EVENT_TYPE_VSHB_CH1:                      return "VSHB-CH1";
            case RADIO_EVENT_TYPE_VSHB_CH2:                      return "VSHB-CH2";
            case RADIO_EVENT_TYPE_VSHB_CH3:                      return "VSHB-CH3";
            case RADIO_EVENT_TYPE_VSHB_CH4:                      return "VSHB-CH4";
            case RADIO_EVENT_TYPE_CUSTOM:                      return "CUSTOM";
            case RADIO_EVENT_TYPE_DEVICE_INFO:                 return "DEV-INFO";
            case RADIO_EVENT_TYPE_LINK_QUALITY:                return "LINK-QUAL";
            case RADIO_EVENT_TYPE_DEBUG_MESSAGE:               return "DEBUG";
            case RADIO_EVENT_TYPE_TEST_RESULT:                 return "TEST";
            case RADIO_EVENT_TYPE_DOWNLINK_ACK:                return "DL_ACK";
            default:                                           return "UNKNOWN";
         }
      }
   #endif // INCLUDE_ONLY_IN_TRC_C
#endif // EUSART_TRC_ENABLE
//
//------------------------------------------------------------------------
//------------------------------------------------------------------------
//------------------------------------------------------------------------


// PROTOCOL VERSION
#define PROTOCOL_VERSION_RESET_EVENT                0x10
#define PROTOCOL_VERSION_SUPERVISORY                0x10
#define PROTOCOL_VERSION_TAMPER                     0x10
#define PROTOCOL_VERSION_REED_SWITCH                0x10
#define PROTOCOL_VERSION_HES                        0x10
#define PROTOCOL_VERSION_ACCEL_MOTION_SENSOR        0x10
#define PROTOCOL_VERSION_GLASS_BREAK                0x10
#define PROTOCOL_VERSION_PUSH_BUTTON                0x10
#define PROTOCOL_VERSION_CONTACT_SENSOR             0x10
#define PROTOCOL_VERSION_WATER_SENSOR               0x10
#define PROTOCOL_VERSION_TEMP_SENSOR                0x10
#define PROTOCOL_VERSION_TILT_SENSOR                0x10
#define PROTOCOL_VERSION_TILT_HP_SENSOR             0x10
#define PROTOCOL_VERSION_MAG_SENSOR                 0x10
#define PROTOCOL_VERSION_COMPASS_SENSOR             0x10
#define PROTOCOL_VERSION_ULTRASONIC_SENSOR          0x10
#define PROTOCOL_VERSION_GPS                        0x10
#define PROTOCOL_VERSION_VSHB                       0x10
#define PROTOCOL_VERSION_VWS                        0x10
#define PROTOCOL_VERSION_HW5800                     0x10
#define PROTOCOL_VERSION_420MA_SENSOR               0x10
#define PROTOCOL_VERSION_TC_SENSOR                  0x10
#define PROTOCOL_VERSION_VOLTMETER                  0x10
#define PROTOCOL_VERSION_CUST                       0x10   // Custom Sensor Protocol Version

#define PROTOCOL_VERSION_AMBIENT_LIGHT_SENSOR       0x10
#define PROTOCOL_VERSION_TANK_LEVEL_SENSOR          0x10
#define PROTOCOL_VERSION_AIR_TEMP_HUMIDITY_SENSOR   0x10
#define PROTOCOL_VERSION_TEMP_INT_SENSOR            0x10

#define PROTOCOL_VERSION_DEVICE_INFO                0x10
#define PROTOCOL_VERSION_DEBUG_MESSAGE              0x10
#define PROTOCOL_VERSION_TEST_RESULT                0x10
#define PROTOCOL_VERSION_DOWNLINK_ACK               0x10
#define PROTOCOL_VERSION_LINK_QUALITY               0x10

#ifdef ENABLE_AES
#define PROTOCOL_VERSION_AES_FORMAT                 0x80   // this can be OR'd with the above PROTOCOL_VERSION_* to turn on AES
#endif // ENABLE_AES

// buffer size for radio messages
#define RADIO_READ_BUFFER_SIZE 80
#if RADIO_READ_BUFFER_SIZE < (80)
   #error The Read Buffer MUST be big enough to handle the largest message from Radio (xDot is currently sending 'Set Network Key: .....' messages longer than 68 bytes)
#endif

#define RADIO_TX_MESSAGE_MAX_LENGTH 64
#define RADIO_TX_BUFFER_SIZE 4

#ifdef LORA
#define DOWNLINK_MESSAGE_LENGTH 13
#endif

// This compiler check is redundent and is also checked elsewhere, but is necessary to absolutely
// ensure that the size of Tx_string and next_event_buf are large enough.  If too small, than a
// very hard to debug run time error is possible where the buffer is over-written.
#if TX_BUF_SIZE_MAX < (TX_STRING_LENGTH_MAX + 1)
   #error TX_BUF_SIZE_MAX must be >= DOWNLINK_MESSAGE_LENGTH
#endif

extern char Tx_string            [TX_STRING_LENGTH_MAX + 1];  // +1 is for '\0' character
extern char next_event_buf_string[TX_STRING_LENGTH_MAX + 1];  // +1 is for '\0' character
extern bool next_event_downlink_request;
extern uint16_t uart_tmr;



void radio_transmit_gen_hex(uint8_t index, uint8_t data);
//void radio_transmit_gen_dec(uint8_t index, uint16_t data, uint8_t num_digits);

//void radio_transmit_hex_data_3(uint8_t protocol_version, uint8_t radio_event, bool downlink_request, bool join_wakeup);
//void radio_transmit_hex_data_4(uint8_t protocol_version, uint8_t radio_event, uint8_t data_2, bool downlink_request, bool join_wakeup);
//void radio_transmit_hex_data_5(uint8_t protocol_version, uint8_t radio_event, uint8_t data_2, uint8_t data_3, bool downlink_request, bool join_wakeup);
void radio_transmit_hex_data_6(uint8_t protocol_version, uint8_t radio_event, uint8_t data_2, uint8_t data_3, uint8_t data_4, bool downlink_request, bool join_wakeup);
//void radio_transmit_hex_data_7(uint8_t protocol_version, uint8_t radio_event, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, bool downlink_request, bool join_wakeup);
//void radio_transmit_hex_data_8(uint8_t protocol_version, uint8_t radio_event, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, bool downlink_request, bool join_wakeup);
//void radio_transmit_hex_data_9(uint8_t protocol_version, uint8_t radio_event, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7, bool downlink_request, bool join_wakeup);
//void radio_transmit_hex_data_10(uint8_t protocol_version, uint8_t radio_event, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7, uint8_t data_8, bool downlink_request, bool join_wakeup);
//void radio_transmit_hex_data_11(uint8_t protocol_version, uint8_t radio_event, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7, uint8_t data_8, uint8_t data_9, bool downlink_request, bool join_wakeup);
void radio_transmit_hex_data_12(uint8_t protocol_version, uint8_t radio_event, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7, uint8_t data_8, uint8_t data_9, uint8_t data_10, bool downlink_request, bool join_wakeup);

//void radio_transmit_special__5(uint8_t protocol_version, uint8_t radio_event, uint16_t data_2, uint16_t data_3, bool downlink_request, bool join_wakeup);
//void radio_transmit_special__6(uint8_t protocol_version, uint8_t radio_event, uint8_t data_2, uint8_t data_3, uint8_t data_4, bool downlink_request, bool join_wakeup);
//void radio_transmit_special__10(uint8_t protocol_version, uint8_t radio_event, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7, uint8_t data_8, bool downlink_request, bool join_wakeup);
//void radio_transmit_special__12(uint8_t protocol_version, uint8_t radio_event, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7, uint8_t data_8, uint8_t data_9, uint8_t data_10, bool downlink_request, bool join_wakeup);

void radio_transmit(uint8_t radio_event, bool downlink_request, bool join_wakeup);
void radio_send_string(const char * uart_string, uint8_t len);
//void radio_send_string(uint8_t * uart_string);
void check_radio_uart_read(void);
void set_radio_state_delay(uint8_t state_delay);
void radio_uart_flush(void);
void reset_link_quality_periodic_timer(void);
void lowPowerDelay(uint32_t millisec);
void initLetimer(uint8_t en);
void LETIMER0_IRQHandler(void);
void start_radio_response_timeout(uint16_t timeout_val);
void stop_radio_response_timeout(void);
void reset_radio_governor(uint32_t timeout_val);
bool is_timer_running(void);

#endif /* RADIO_COMMON_H_ */
