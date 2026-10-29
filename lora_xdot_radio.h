/*
 * lora_xdot_radio.h
 *
 *  Created on: Mar. 16, 2022
 *      Author: jasonp
 */

#ifndef LORA_XDOT_RADIO_H_
#define LORA_XDOT_RADIO_H_

#include "sl_bt_api.h"
#include "build_config.h"
#include "em_gpio.h"
#include "utility.h"

// Device ID response: 00-01-02-03-04-05-06-07
#define LORA_XDOT_ID_MESSAGE_LENGTH (16 + 7)
#define LORA_XDOT_RECEIVED_STRING_MAX_LEN (LORA_XDOT_ID_MESSAGE_LENGTH + 1)   // max string len returned from xDot Radio is currently Lora DeviceID with bytes seperated by '-'  ex: "00-80-00-00-04-01-20-63"


//B01-004 PCBA
#define LORA_RST_PORT gpioPortC
#define LORA_RST_PIN  4
#ifdef NEW_HW
#define LORA_WAKE_PORT  gpioPortC
#else
#define LORA_WAKE_PORT  gpioPortB
#endif
#define LORA_WAKE_PIN 1


// DATA_RATE (DR) response:
//DR0 - SF10BW125
//DR1 - SF9BW125
//DR2 - SF8BW125
//DR3 - SF7BW125
//DR4 - SF8BW500

//set the base address where LoRa values can be stored
#define LORA_PSKEY_ADDRESS  0x4030

// DEVICE_ID (Sigfox)
// DEV-EUI   (LoRa)
#define EEPROM_DEVICE_ID_OFFSET                   LORA_PSKEY_ADDRESS
#define EEPROM_DEVICE_ID_LENGTH                   0x08    // (Sigfox 4 bytes, Lora - 8 bytes)

// The following are two - 2 byte checksums using Fletcher16 algorithm
// KEY Checksum
#define EEPROM_KEY_CHECKSUM_OFFSET                LORA_PSKEY_ADDRESS+1
#define EEPROM_KEY_CHECKSUM_LENGTH                0x02    // (Lora - 2 bytes (MSB first, LSB second)

// LORA_APP_EUI Checksum
#define EEPROM_LORA_APP_EUI_CHECKSUM_OFFSET       LORA_PSKEY_ADDRESS+2
#define EEPROM_LORA_APP_EUI_CHECKSUM_LENGTH       0x02    // (Lora - 2 bytes (MSB first, LSB second), Sigfox Undefined)

// LORA UNIQUE
#define EEPROM_LORA_APP_KEY_OFFSET                LORA_PSKEY_ADDRESS+3  // Overlaps with Sigfox (EEPROM_KEY_OFFSET) ################ Careful ############
#define EEPROM_LORA_APP_KEY_LENGTH                0x10    // (Lora - 16 bytes)

#define EEPROM_LORA_APP_KEY_MFG_TEST_OFFSET       LORA_PSKEY_ADDRESS+4
#define EEPROM_LORA_APP_KEY_MFG_TEST_LENGTH       0x10    // (Lora - 16 bytes)

#define EEPROM_LORA_APP_EUI_OFFSET                LORA_PSKEY_ADDRESS+5
#define EEPROM_LORA_APP_EUI_LENGTH                0x08    // (Lora - 8 bytes)

#define EEPROM_LORA_APP_EUI_MFG_TEST_OFFSET       LORA_PSKEY_ADDRESS+6
#define EEPROM_LORA_APP_EUI_MFG_TEST_LENGTH       0x08    // (Lora - 8 bytes)

// The following chunk of EEPROM is reserved for the ProgrammingApp to write certain details into EEPROM
// only known at programming.  This includes Date and Time of when it was programmed, and a
// manufacturing code.  At the time the format of this block is reserved for the ProgrammingApp
// as this firmware will never look at and read these values.  If the future, it is possible that
// the sensor may be able to return parts of the data in this block in future TBD messages, but that
// is not currently planned.
#define EEPROM_MANUFACTURING_LOT_CODE_OFFSET      LORA_PSKEY_ADDRESS+7
#define EEPROM_MANUFACTURING_LOT_CODE_LENGTH      0x10    // size: 0x10 (16 bytes)

// Build_Config (see build_config.h)
// NOTE that the first byte of the EEPROM_CONFIG contains an EEPROM_VERSION useful to determining the ...
// ... entire format of EEPROM ######################## VERY USEFUL TO REMEMBER #########################
#define EEPROM_CONFIG_OFFSET                      LORA_PSKEY_ADDRESS+8
#define EEPROM_CONFIG_LENGTH                      0x20    // size: 0x20 (32 bytes)

#define EEPROM_DEPRECATED_LORA_SUB_BAND_CURRENT_OFFSET LORA_PSKEY_ADDRESS+9  // size: 1 byte

// DOWNLINK_CONFIG data

// The following indicated the total number of DOWNLINKs that are saved
// in EEPROM.  All 6 of them follow:
#define EEPROM_DOWNLINK_MAX_NUMBER_SAVED_IN_EEPROM   6

// The following 1 define are used when (CONFIG_VERSION == CONFIG_VERSION_1)
#define EEPROM_DOWNLINK_CONFIG_SENSOR_OFFSET      LORA_PSKEY_ADDRESS+10 // size: 0x08 (8 bytes) // note this is same offset as next #define

// The following 4 defines are used when (CONFIG_VERSION == CONFIG_VERSION_2)
//#define EEPROM_DOWNLINK_CONFIG_MULTI_SENSOR_0_OFFSET  LORA_PSKEY_ADDRESS+10 // size: 0x08 (8 bytes)
//#define EEPROM_DOWNLINK_CONFIG_MULTI_SENSOR_1_OFFSET  LORA_PSKEY_ADDRESS+11 // size: 0x08 (8 bytes)
//#define EEPROM_DOWNLINK_CONFIG_MULTI_SENSOR_2_OFFSET  LORA_PSKEY_ADDRESS+12 // size: 0x08 (8 bytes)
//#define EEPROM_DOWNLINK_CONFIG_MULTI_SENSOR_3_OFFSET  LORA_PSKEY_ADDRESS+13 // size: 0x08 (8 bytes)

//#define EEPROM_DOWNLINK_CONFIG_GENERAL_OFFSET     LORA_PSKEY_ADDRESS+14 // size: 0x08 (8 bytes)
//#define EEPROM_DOWNLINK_CONFIG_ADVANCED_OFFSET    LORA_PSKEY_ADDRESS+15 // size: 0x08 (8 bytes)

// Feature List Length
//#define EEPROM_FEATURE_LIST_LENGTH_OFFSET         LORA_PSKEY_ADDRESS+16 // size: 0x01 (1 byte)
//
//#define EEPROM_LORA_NET_SESSION_KEY_MFG_TEST_OFFSET  LORA_PSKEY_ADDRESS+17
//#define EEPROM_LORA_NET_SESSION_KEY_MFG_TEST_LENGTH  0x0010
//
//#define EEPROM_LORA_DATA_SESSION_KEY_MFG_TEST_OFFSET  LORA_PSKEY_ADDRESS+18
//#define EEPROM_LORA_DATA_SESSION_KEY_MFG_TEST_LENGTH  0x0010
//
//#define EEPROM_LORA_NET_ADDR_MFG_TEST_OFFSET  LORA_PSKEY_ADDRESS+19
//#define EEPROM_LORA_NET_ADDR_MFG_TEST_LENGTH  0x0004
//
//#define EEPROM_LORA_TXFREQ_MFG_TEST_OFFSET  LORA_PSKEY_ADDRESS+20   // MSB first, LSB last
//#define EEPROM_LORA_TXFREQ_MFG_TEST_LENGTH  0x0004

#define EEPROM_DEVICE_ID_LEN_IN_BYTES     8
#define EEPROM_KEY_LEN_IN_BYTES           16

// 256ms timer, 0.5 secs between transmissions to allow for radio to complete
#define RADIO_GOVERNOR_TIMEOUT_LORA 0x02

// Response timeout for radio module: 8 sec
// use power of 2 so we can use & in timer instead of >
#define RADIO_RESPONSE_TIMEOUT_STANDARD 0x08

// Response timeout after send: 16 sec
// use power of 2 so we can use & in timer instead of >
#define RADIO_RESPONSE_ACK_TIMEOUT 0x40

// Response timeout for joining network: 16 sec
// use power of 2 so we can use & in timer instead of >
#define RADIO_RESPONSE_TIMEOUT_JOIN 0x10

// Response timeout for joining network: 16 sec
// use power of 2 so we can use & in timer instead of >
#define RADIO_RESPONSE_TIMEOUT_JOIN__XDOT_LORA_1_0  0x10

// Response timeout for joining network: 64 sec
// use power of 2 so we can use & in timer instead of >
#define RADIO_RESPONSE_TIMEOUT_JOIN__MURATA_LORA_1_1_03  0x40

// use power of 2 so we can use & in timer instead of >
// Response timeout for downlink: 4 sec
#define RADIO_DOWNLINK_TIMEOUT 0x04


// 256ms timer, 0.256 secs to delay before JOIN or RE-JOIN
#define RADIO_GOVERNOR_DELAY_BEFORE_JOIN 0x01

// 1 minute delay after successful join
//#define RADIO_GOVERNOR_DELAY_AFTER_JOIN 0x10
#define RADIO_GOVERNOR_DELAY_AFTER_JOIN 0x01   // SMS - decreased to 250 ms because can't see reason to wait.

#define TEMP_UART_BUF_LEN         127
#define TEMP_WORK_BUF_LEN         127
#define TEMP_WORK_BUF_32           32



// state machine states for the lora radio
typedef enum lora_state {
    LORA_STATE_RESET,
    LORA_STATE_RELEASE_RESET,

    LORA_STATE_GET_AT,
    LORA_STATE_SET_FSB,
    LORA_STATE_DIS_NONCE,
    LORA_STATE_GET_FIRMWARE_VERSION,
    LORA_STATE_GET_DEFAULT_FREQUENCY_BAND,
    LORA_STATE_SET_DEFAULT_FREQUENCY_BAND,
    LORA_STATE_STORE_DEFAULT_FREQUENCY_BAND,
    LORA_STATE_SET_ATZ,
    LORA_STATE_SET_ATZ_WAIT,
  LORA_STATE_SET_WAKE_MODE,
  LORA_STATE_SET_WAKE_MODE_DELAY,
  LORA_STATE_SET_WAKE_DELAY_INTERVAL,
    LORA_STATE_SET_WAKE_PIN,
    LORA_STATE_SET_DEVICE_ID,
    LORA_STATE_SET_MODE,
    LORA_STATE_SET_RFPOWER,
    LORA_STATE_SET_PUBLIC_NETWORK,
    LORA_STATE_SET_KEY,
    LORA_STATE_SET_APP_EUI,
    LORA_STATE_SET_REP,
    LORA_STATE_DISABLE_ECHO,

    LORA_STATE_WAKE_UP,
    LORA_STATE_WAKE_UP_CONT_1,
    LORA_STATE_WAKE_UP_CONT_2,

    LORA_STATE_TRANSMIT_START,
    LORA_STATE_TRANSMIT_SET_DATA_RATE,
    LORA_STATE_TRANSMIT_SET_RFPOWER,
    LORA_STATE_TRANSMIT_PAYLOAD,
    LORA_STATE_TRANSMIT_PAYLOAD_CONT,
    LORA_STATE_TRANSMIT_PAYLOAD_CONT_2,
    LORA_STATE_TRANSMIT_PAYLOAD_CONT_3,
    LORA_STATE_TRANSMIT_PAYLOAD_CONT_4,
    LORA_STATE_TRANSMIT_WAIT_FOR_ACK,
    LORA_STATE_TRANSMIT_WAITING_FOR_ACK,

    LORA_STATE_CHECK_FOR_NEXT_EVENT_SEND,
    LORA_STATE_QUERY_FIRMWARE_VERSION,
    LORA_STATE_RECEIVE_FIRMWARE_VERSION,
    LORA_STATE_QUERY_DATA_RATE,
    LORA_STATE_RECEIVE_DATA_RATE,
    LORA_STATE_QUERY_PAYLOAD,
    LORA_STATE_RECEIVE_PAYLOAD,

    LORA_STATE_WAIT_FOR_STRING_AND_OK,
    LORA_STATE_WAITING_FOR_STRING_AND_OK,
    LORA_STATE_WAIT_FOR_UINT_AND_OK,
    LORA_STATE_WAITING_FOR_UINT_AND_OK,
    LORA_STATE_WAIT_FOR_OK,
    LORA_STATE_WAITING_FOR_OK,


    // JOIN
    LORA_STATE_JOIN_START,
    LORA_STATE_JOIN_ATTEMPT,
    LORA_STATE_JOIN_WAIT,
    LORA_STATE_JOIN_WAITING,

    // GO_TO_SLEEP
    LORA_STATE_GO_TO_SLEEP,
    LORA_STATE_GO_TO_SLEEP_CONT_1,
    LORA_STATE_GO_TO_SLEEP_CONT_2,
    LORA_STATE_SLEEPING,

    // QUERY RSSI and SNR Sequence
    LORA_STATE_QUERY_RSSI,
    LORA_STATE_WAITING_FOR_RSSI,
    LORA_STATE_QUERY_SNR,
    LORA_STATE_WAITING_FOR_SNR,
    LORA_STATE_PING,
    LORA_STATE_WAITING_FOR_PING,

    // QUERY TRANSMIT DELAY  (used for JOIN's and transmit)  // called BACKOFF DELAY for Murata
    LORA_STATE_QUERY_TRANSMIT_DELAY,
    LORA_STATE_RECEIVE_TRANSMIT_DELAY,

    LORA_STATE_INVALID
} LORA_STATE;




char temp_uart_buf[TEMP_UART_BUF_LEN + 1];  // +1 is for '\0' char at end
char temp_work_buf[TEMP_WORK_BUF_LEN + 1];  // +1 is for '\0' char at end
char temp_work_buf_32 [TEMP_WORK_BUF_32  + 1];  // +1 is for '\0' char at end

bool link_quality_message_enable;

void process_lora_message(uint8_t message_length);
bool lora_state_machine(void);
void lora_init(void);
bool validate_app_key(void);
bool validate_app_eui(void);
const char *lora_xdot_radio_get_dfreq_string(int zone);





uint8_t api_eeprom_read_byte(uint16_t offset_in_eeprom);
void api_eeprom_write_byte(uint16_t offset_in_eeprom, uint8_t new_data);
void api_eeprom_read_array(uint8_t * dstP, uint16_t offset_in_eeprom, uint8_t dst_length);
void api_eeprom_write_array(const uint8_t * srcP, uint16_t offset_in_eeprom, uint8_t src_length);

uint16_t api_eeprom_read_u16_big_endian(uint16_t offset_in_eeprom);
uint32_t api_eeprom_read_u32_big_endian(uint16_t offset_in_eeprom);
void api_eeprom_read_array_and_convert_to_string(uint8_t * dst_buf, uint8_t dst_len_in_bytes, uint16_t eeprom_offset, uint8_t eeprom_length_in_bytes);

void stop_radio_response_timeout(void);
void reset_radio_governor(uint32_t timeout_val);
void start_radio_response_timeout(uint16_t timeout_val);
//---------------------------------------------------------------------------------
uint8_t * convert_uint8_to_ascii_string(uint8_t data);    // dst_str MUST be of size >= 4 bytes
void api_eeprom_read_array_and_convert_to_string(uint8_t * dst_buf, uint8_t dst_len_in_bytes, uint16_t eeprom_offset, uint8_t eeprom_length_in_bytes);
int8_t convert_small_floating_point_ascii_array_to_8_bit_twos_complement_int(uint8_t *p);
uint8_t ascii_byte_to_hex(uint8_t *asciival);
uint8_t ascii_nibble_to_hex(uint8_t asciival);
bool is_ascii_coded_hex_number(uint8_t ascii_hex_char);
bool is_ascii_coded_hex_uppercase(uint8_t ascii_hex_char);
bool is_ascii_coded_hex_lowercase(uint8_t ascii_hex_char);
int32_t convert_array_of_ascii_decimal_chars_to_32_bit_integer(uint8_t *array_ascii, int8_t array_size);
void lora_payload(void);
void unreg_lora(void);


#endif /* LORA_XDOT_RADIO_H_ */
