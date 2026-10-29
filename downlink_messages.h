/*
 * downlink_messages.h
 *
 *  Created on: Mar. 17, 2022
 *      Author: jasonp
 */

#ifndef DOWNLINK_MESSAGES_H_
#define DOWNLINK_MESSAGES_H_

/*
    Message receive processing for downlink messages

  (c) 2017 Radio Bridge Inc. and its subsidiaries.
*/
#include "sl_bt_api.h"
// message length for downlink ack message
#define DOWNLINK_ACK_LENGTH 4

// the downlink is always 8 bytes
#define DOWNLINK_MESSAGE_SIZE 8

// define ack payloads

// we are not using the no message downlink
//#define DOWNLINK_ACK_NO_MESSAGE 0x00
#define DOWNLINK_ACK_MESSAGE_INVALID 0x01
#define DOWNLINK_ACK_MESSAGE_VALID 0x02

// define downlink commands
#define DOWNLINK_NO_ACTION 0x00

// downlink command bytes, one for each sensor type
// and usually match the uplink command byte
#define DOWNLINK_DISABLE_OPEN_CLOSE_EVENT_MESSAGES 0x03
#define DOWNLINK_GLASS_BREAK 0x05
#define DOWNLINK_PUSH_BUTTON 0x06
#define DOWNLINK_CONTACT 0x07
#define DOWNLINK_WATER 0x08

#define DOWNLINK_TEMP 0x09
#define DOWNLINK_TILT 0x0a
#define DOWNLINK_AMBIENT_LIGHT 0x0b
#define DOWNLINK_TANK_LEVEL 0x0c
#define DOWNLINK_AIR_TEMP_HUMIDITY 0x0d
#define DOWNLINK_ACCEL_MOTION 0x0e
#define DOWNLINK_TILT_HP 0x0f
#define DOWNLINK_ULTRASONIC 0x10
#define DOWNLINK_420MA 0x11
#define DOWNLINK_TC 0x13
#define DOWNLINK_VOLTMETER 0x14
#define DOWNLINK_CUST 0x15
#define DOWNLINK_GPS 0x16
#define DOWNLINK_HW5800 0x17
#define DOWNLINK_MAG 0x18      // magnetometer
#define DOWNLINK_TEMP_INT 0x19
#define DOWNLINK_COMPASS 0x1A
#define DOWNLINK_VWS 0x1B
#define DOWNLINK_VSHB_CH1 0x1C
#define DOWNLINK_VSHB_CH2 0x1D
#define DOWNLINK_VSHB_CH3 0x1E
#define DOWNLINK_VSHB_CH4 0x1F

#define DOWNLINK_CONFIG_GENERAL 0x01     // DOWNLINK General Configuration
#define DOWNLINK_CONFIG_ADVANCED 0xfc    // DOWNLINK Advanced Configuration

void transmit_downlink_ack(uint8_t downlink_message_valid);
void write_downlink_to_eeprom(uint8_t *downlink_payload);
void process_downlink_message(uint8_t * downlink_config, uint8_t downlink_num_bytes, bool downlink_from_radio);
void load_all_downlink_config_from_eeprom(void);

uint16_t convert_period_to_minutes(uint8_t value);
uint16_t convert_period_to_seconds(uint8_t value);
uint16_t convert_hold_to_ticks(uint8_t value);
uint32_t convert_period_to_ticks(uint8_t value);
bool is_downlink_eeprom_valid(void);
bool are_upper_bytes_zero(uint8_t *byte_array, uint8_t byte_length);
bool are_upper_bits_zero(uint8_t byte_containing_bits, uint8_t bit_length);



#endif /* DOWNLINK_MESSAGES_H_ */
