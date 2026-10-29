/*
 * radio_tx_buf.h
 *
 *  Created on: Mar. 17, 2022
 *      Author: jasonp
 */

#ifndef RADIO_TX_BUF_H_
#define RADIO_TX_BUF_H_

/*
 * Radio tx buf (transmit buffer) management functions for radio
 *
 * (c) 2017, 2018 Radio Bridge Inc. and its subsidiaries.
 */
#include "sl_bt_api.h"

#define TX_STRING_LENGTH_MAX     28

#if defined(VWS_ENABLE) || defined(VIBE_ENABLE)
    #define TX_BUF_SENSOR_EVENTS_MAX  16  // MUST be a power of 2 (ie, 2, 4, 8, 16 ,etc)
#else
    #define TX_BUF_SENSOR_EVENTS_MAX  4  // MUST be a power of 2 (ie, 2, 4, 8, 16 ,etc)
#endif

#define TX_BUF_SIZE_MAX          32  // SHOULD be power of 2 (ie, 2, 4, 8, 16 ,etc) and MUST be >= TX_STRING_LENGTH_MAX

#if TX_BUF_SIZE_MAX < (TX_STRING_LENGTH_MAX + 1)
   #error TX_BUF_SIZE_MAX must be >= DOWNLINK_MESSAGE_LENGTH
#endif
void radio_tx_buf_init(void);
bool radio_tx_buf_new_event_enqueued(void);
void radio_tx_buf_new_join_wakeup_event_clear(void);
bool radio_tx_buf_new_join_wakeup_event_enqueued(void);
void radio_tx_buf_enqueue(uint8_t radio_event, bool downlink_request, bool join_wakeup);
bool radio_tx_buf_dequeue(char * buf_string_ptr /* OUTPUT PARAMETER */ );
bool radio_tx_buf_has_event_to_send(void);
bool radio_tx_buf_can_device_info_be_sent(void);
bool radio_tx_buf_has_downlink_ack_to_send(void);
bool radio_tx_buf_did_manufacturing_test_just_end(void);
uint8_t radio_tx_buf_get_packet_count(void);
void radio_tx_buf_received_send_confirmation(void);
void radio_tx_buf_received_ack_confirmation(bool ack_received);
void radio_tx_buf_set_send_handler(void (* send_handler)(uint8_t data));
void radio_tx_buf_set_ack_handler (void (* ack_handler) (uint8_t data));
void radio_tx_buf_default_send_handler(uint8_t data);
void radio_tx_buf_default_ack_handler (uint8_t data);
void clear_tx_buf(void);

extern bool radio_tx_buf_force_confirmed_event__set;
extern bool radio_tx_buf_force_confirmed_event__get;
extern bool radio_tx_buf_force_confirmed_event__enable;

#define RADIO_TX_BUF_FORCE_CONFIRMED_EVENT__SET(x)     { radio_tx_buf_force_confirmed_event__set = (x); }
#define RADIO_TX_BUF_FORCE_CONFIRMED_EVENT__GET()       radio_tx_buf_force_confirmed_event__get
#define RADIO_TX_BUF_FORCE_CONFIRMED_EVENT__ENABLE(x) { radio_tx_buf_force_confirmed_event__enable = (x); }


#endif /* RADIO_TX_BUF_H_ */
