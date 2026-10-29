/*
 * lora_api_internal_data.h
 *
 *  Created on: Mar. 16, 2022
 *      Author: jasonp
 */

#ifndef LORA_API_INTERNAL_DATA_H_
#define LORA_API_INTERNAL_DATA_H_

/*
    Common Header and Data definition for Lora (Murata and xDot)

  (c) 2020 Radio Bridge Inc. and its subsidiaries.
*/

#ifndef   _LORA_API_INTERNAL_DATA_H_
#define   _LORA_API_INTERNAL_DATA_H_

#include "downlink_messages.h"
#include "radio_common.h"
#include "sl_bt_api.h"
#include "app-config.h"

//#if ! defined(LORA_XDOT) && ! defined(LORA_MURATA)
//    #error Must be included from only lora_xdot_radio.c or lora_radio.c
//#endif // ! defined(LORA_XDOT) && ! defined(LORA_MURATA)
//
//#ifndef    _DEFINE_LORA_API_INTERNAL_DATA
//    #error _DEFINE_LORA_API_INTERNAL_DATA must be defined before including this file
//#endif  // _DEFINE_LORA_API_INTERNAL_DATA

// ************************************************************
// ************************************************************
//#define LORA_XDOT
#define TRANSMIT_SLEEP_IN_SECONDS_AFTER_ERROR 1
#define RADIO_TIMER_TICKS_PER_SECOND 10
#define RADIO_STATE_DELAY_IN_TICKS_AFTER_ERROR (TRANSMIT_SLEEP_IN_SECONDS_AFTER_ERROR * RADIO_TIMER_TICKS_PER_SECOND)

#define TOTAL_RETRY_ATTEMPTS__RX_BUF_FULL_OF_MAC_CMDS  19 // default is 19 retries (results in 20 total)
#define TOTAL_RETRY_ATTEMPTS__XMIT_IN_PROGRESS         19 // default is 19 retries (results in 20 total)
#define TOTAL_RETRY_ATTEMPTS__ACK_NOT_RECEIVED          1 // default is  3 retries (results in  4 total)

typedef struct
{
    char *     buf_p;
    uint8_t    max_chars;  // this includes room for the last '\0' character of a valid C-String
    uint8_t    num_chars;
} SAFE_C_STR;



// ************************************************************
// ************************************************************
// Both Murata and xDot
// ************************************************************
// ************************************************************
#ifdef LORA_XDOT // || defined(LORA_MURATA)

  //-----------------------------------------
  // common static data
  //-----------------------------------------

    // true when we receive an OK response
    static bool lora_ok_received           = false;

    // true when a downlink is received
    static bool lora_downlink_received     = false;

    static uint8_t xmit_retry_attempt;                          // no need to initialize

    static uint8_t xmit_retry_attempt__ack_not_received;        // no need to initialize

    // state variable for lora state machine
    static LORA_STATE lora_state = LORA_STATE_RESET;

    // next state is used when we go to a common wait state and need to know
    // where to jump back to
    static LORA_STATE lora_next_state = LORA_STATE_RESET;

    static LORA_STATE lora_next_next_state = LORA_STATE_RESET;

    // contains the last read data rate value
    static uint8_t lora_data_rate_value;

    // true if app key and/or app eui in eeprom are valid and can be used
    static bool app_key_valid = false;
    static bool app_eui_valid = false;

    static char lora_app_key_string[(EEPROM_LORA_APP_KEY_LENGTH * 2) + 1];  // the +1 is for C-strings null '\0' char at end
    static char lora_app_eui_string[(EEPROM_LORA_APP_EUI_LENGTH * 2) + 1];  // the +1 is for C-strings null '\0' char at end

    // true when we receive a response to join request
    static bool lora_join_received = false;
    static bool lora_waiting_for_join = false;

    // true when we receive a downlink response
    static bool lora_waiting_for_downlink = false;

    static uint8_t downlink_message_bytes[DOWNLINK_MESSAGE_SIZE+1];

    // The following structure is used for accumulating 1 or more sub-strings from the radio.
    // It provides a safe mechanism where its primary purpose to to ensure that memory
    // following the buffer is never over-written independent of what the radio might send
    // us.  Specifically, this is critically important for protecting against the Radio
    // from sending us arbitrarily long and unexpected strings.
    static SAFE_C_STR safe_c_str;

    static int8_t lora_last_downlink_rssi = 0;  // -128 <= lora_last_downlink_rssi <= 127
    static int8_t lora_last_downlink_snr = 0;   // -128 <= lora_last_downlink_snr  <= 127
    static int8_t lora_last_ping_rssi = 0;  // -128 <= lora_last_downlink_rssi <= 127
    static int8_t lora_last_ping_snr = 0;   // -128 <= lora_last_downlink_snr  <= 127

  //-----------------------------------------
  // common extern data
  //-----------------------------------------

    // true when the governor has expired and we can send to lora
    volatile bool radio_ok_to_send;

    // true when the lora uart read has timed out
volatile bool radio_response_timeout;

    // true when the lora downlink has timed out
    volatile bool radio_downlink_timeout;

    // set to true if there has ever been a timeout, will send with supervisory
    volatile bool radio_response_has_timed_out;

    // true when we are timing a delay between states
    volatile bool radio_state_delay_running;

    // lora_adr is disabled when 'false'
    bool lora_adr_disable;
    bool lora_adr_disable_changed;

    uint8_t lora_port_number;
    uint8_t lora_port_number_new;

    // true if the lora module is in sleep mode
    volatile bool radio_is_sleeping;

    // buffer to hold incoming message from radio
    volatile uint8_t radio_uart_read_buffer[RADIO_READ_BUFFER_SIZE];
    volatile uint8_t radio_uart_read_index;

    // contains the current sub band when attempting to JOIN.  To keep track of what we are JOINing.
    uint8_t lora_api_internal_join__sub_band_attempt;

    // successfully JOINed sub_band
    uint8_t lora_api_internal_join__sub_band_current;

    // contains the next sub band to try when cycling through all sub bands when finding one to JOIN.
    uint8_t lora_api_internal_join__next_sub_band_attempt;

    bool link_quality_periodic_timer_expired;

    // true for confirmed messages, that is with ACKs,  false for unconfirmed messages
    bool lora_confirmed_messages;

    // number of uplink retries for lora
    int8_t lora_uplink_retries;

    // contains an algorithm generated chmask string when trying to JOIN varous sub bands
    #define CHMASK_STRING_LENGTH 18
    uint8_t chmask_str[CHMASK_STRING_LENGTH + 1]; // the +1 is for the '\0' character

  //-----------------------------------------
  // common global data
  //-----------------------------------------


#endif // defined(LORA_XDOT) || defined(LORA_MURATA)

// ************************************************************
// ************************************************************
// xDot Only
// ************************************************************
// ************************************************************
#if defined(LORA_XDOT)

  //-----------------------------------------
  // xDot static data
  //-----------------------------------------

    static uint8_t xmit_retry_attempt__tx_buf_full_of_mac_cmds; // no need to initialize
    static uint8_t xmit_retry_attempt__xmit_in_progress;        // no need to initialize

    static bool lora_error_received__tx_buf_full_of_mac_cmds = false;
    static bool lora_error_received__xmit_in_progress        = false;
    static bool lora_error_received__ack_not_received        = false;
    static bool lora_error_received                          = false;

    // true when the join process is initialted
    static bool lora_join_started = false;

    static bool lora_waiting_for_ok = false;
    static bool lora_waiting_for_error = false;
    static bool lora_waiting_for_string_and_ok = false;
    static bool lora_string_received = false;
    static bool lora_waiting_for_uint_and_ok = false;
    static bool lora_uint_received = false;

    // We save the uplink_counter as the C-string we receive from the AT+ULC? query.  We save as a
    // string of 11 bytes (10 + '\0') instead of converting it to uint32_t because the firmware size
    // increase is much more significant to accomplish this conversion is substantially more than
    // the 7 extra bytes of RAM, which we are not in short supply of.
    //#define NUM_DIGITS_IN_UINT32   10
    //static uint8_t uplink_counter_string[NUM_DIGITS_IN_UINT32 + 1];  // +1 is for '\0' trailing null character to ensure we always have a valid c-string
    //static uint8_t downlink_counter_string[NUM_DIGITS_IN_UINT32 + 1];  // +1 is for '\0' trailing null character to ensure we always have a valid c-string
    static uint32_t lora_radio_received_uint = 0;

    static bool lora_firmware_version_capture = false;

    // lora_waiting_for_transmit_delay_to_elapse is set to true whenever the xDot radio (via AT+TXN?)
    // indicates that we must wait before trying a JOIN or a TRANSMIT.
    static bool lora_waiting_for_transmit_delay_to_elapse = false;

    static bool lora_waiting_for_rssi = false;
    static bool lora_rssi_received = false;
    static bool lora_waiting_for_snr = false;
    static bool lora_snr_received = false;
    static bool lora_waiting_for_ping = false;
    static bool lora_ping_received = false;

  //-----------------------------------------
  // xDot extern data
  //-----------------------------------------


  //-----------------------------------------
  // common global data
  //-----------------------------------------
    #ifdef _DEFINE_LORA_API_INTERNAL_DATA

    #endif // _DEFINE_LORA_API_INTERNAL_DATA

#endif // defined(LORA_XDOT)

// ************************************************************
// ************************************************************
// Murata Only
// ************************************************************
// ************************************************************
#if defined(LORA_MURATA)

  //-----------------------------------------
  // Murata static data
  //-----------------------------------------

    // true when we receive an DATA_RATE (DR) response
    static bool lora_data_rate_received = false;
    static bool lora_waiting_for_data_rate = false;

    static bool lora_join_received__retry = false;

    // true if an +ERR=-7 ("LoRa MAC is busy in transmission.") is received from Murata
    static bool lora_busy_received = false;

    // used by Radio commands that might result in Lora MAC BUSY (+ERR=-7").  This is the retry count
    #define LORA_RETRY_COUNT_FOR_LORA_MAC_BUSY  50
    static uint8_t lora_retry_count = LORA_RETRY_COUNT_FOR_LORA_MAC_BUSY;

    // contains the radio command string that might be retried a few times because of busy
    static const char * lora_cmd_str_p = NULL;

    // true when we receive a radio reboot
    static bool lora_radio_reboot_received = false;
    static bool lora_waiting_for_radio_reboot = false;

    // true when we receive Murata version
    static bool lora_version_received = false;
    static bool lora_waiting_for_version = false;

    // true when we receive an ID response
    static bool lora_id_received = false;
    static bool lora_waiting_for_id = false;

    // true when we receive a backoff delay response
    static bool lora_backoff_delay_received = false;
    static bool lora_waiting_for_backoff_delay = false;
    static int32_t backoff_delay_in_sec;  // must be 32 bit signed integer

    // true when we receive an ack
    static bool lora_ack_received = false;
    static bool lora_waiting_for_ack = false;
    static bool lora_noack_received = false;
    static bool lora_retry_received = false;

    // contains the lora zone
    static uint8_t config_zone;

    // there are two cases when we want to send a confirmed message:
    //   1)  always when configured with confirmed messages, -OR-
    //   2)  when something is overriding un-confirmed messages and FORCING confirmed messages
    static bool lora_dynamic_confirmed_message = false;

    // The following is part of the state machine that gets the last RSSI using "AT_RFQ?"
    static bool lora_waiting_for_rfq = false;
    static bool lora_rfq_received = false;


  //-----------------------------------------
  // Murata extern data
  //-----------------------------------------

    // true if we are receiving a downlink
    extern volatile bool lora_downlink_bytes_next;


  //-----------------------------------------
  // common global data
  //-----------------------------------------
    #ifdef _DEFINE_LORA_API_INTERNAL_DATA

    #endif // _DEFINE_LORA_API_INTERNAL_DATA

#endif // defined(LORA_MURATA)



#endif // _LORA_API_INTERNAL_DATA_H_


#endif /* LORA_API_INTERNAL_DATA_H_ */
