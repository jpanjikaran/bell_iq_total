/*
 * radio_tx_buf.c
 *
 *  Created on: Mar. 28, 2022
 *      Author: jasonp
 */

#include "radio_tx_buf.h"
#include "radio_common.h"
#include "lora_xdot_radio.h"
#include "utility.h"

#ifdef ENABLE_AES
#include "utils.h"
#include "aes.h"
#endif // ENABLE_AES

void (*tx_buf_send_handler)(uint8_t data) = radio_tx_buf_default_send_handler;
void (*tx_buf_ack_handler) (uint8_t data) = radio_tx_buf_default_ack_handler;
static uint8_t tx_buf_radio_data_in_progress;

// The following defines are constant and should be only used internal to this file.

static char tx_buf_downlink_ack_old_event                 [TX_BUF_SIZE_MAX];
static char tx_buf_downlink_ack_new_event                 [TX_BUF_SIZE_MAX];
static char tx_buf_reset_event                            [TX_BUF_SIZE_MAX];
static char tx_buf_link_quality_event                     [TX_BUF_SIZE_MAX];
static char tx_buf_supervisory_event                      [TX_BUF_SIZE_MAX];
static char tx_buf_tamper_open_event                      [TX_BUF_SIZE_MAX];
static char tx_buf_tamper_close_event                     [TX_BUF_SIZE_MAX];
static char tx_buf_device_info_event                      [TX_BUF_SIZE_MAX];  // for sending Device Info like current DOWNLINKs, Config Info(Radio Zone, Power-Source, Class, Device-Type), and MFG_LOT
static char tx_buf_sensor_events[TX_BUF_SENSOR_EVENTS_MAX][TX_BUF_SIZE_MAX];
static uint8_t tx_buf_radio_data[TX_BUF_SENSOR_EVENTS_MAX];
static bool tx_buf_force_confirmed_event_radio_data[TX_BUF_SENSOR_EVENTS_MAX];

static bool tx_buf_downlink_ack_old_valid        = false;
static bool tx_buf_downlink_ack_new_valid        = false;
static bool tx_buf_reset_event_valid             = false;
static bool tx_buf_link_quality_event_valid      = false;
static bool tx_buf_supervisory_event_valid       = false;
static bool tx_buf_tamper_open_event_valid       = false;
static bool tx_buf_tamper_close_event_valid      = false;
static bool tx_buf_device_info_event_valid       = false;
static bool tx_buf_new_event_enqueued            = false;
static bool tx_buf_new_join_wakup_event_enqueued = false;

static bool tx_buf_force_confirmed_event_downlink_ack_old_event  = false;
static bool tx_buf_force_confirmed_event_downlink_ack_new_event  = false;
static bool tx_buf_force_confirmed_event_reset_event             = false;
static bool tx_buf_force_confirmed_event_link_quality_event      = false;
static bool tx_buf_force_confirmed_event_supervisory_event       = false;
static bool tx_buf_force_confirmed_event_tamper_open_event       = false;
static bool tx_buf_force_confirmed_event_tamper_close_event      = false;
static bool tx_buf_force_confirmed_event_device_info_event       = false;

static uint8_t tx_buf_sensor_events_head    = 0;
static uint8_t tx_buf_sensor_events_tail    = 0;

typedef enum radio_tx_buf_type
{
    RADIO_TX_BUF_TYPE__NONE,
    RADIO_TX_BUF_TYPE__DOWNLINK_ACK,  // there are two types of these now (old: FF01) and (new: FF 04 x0 x1 x2 x3 x4 x5 x6 x7 )
    RADIO_TX_BUF_TYPE__RESET_EVENT,
    RADIO_TX_BUF_TYPE__LINK_QUALITY_EVENT,
    RADIO_TX_BUF_TYPE__SUPERVISORY_EVENT,
    RADIO_TX_BUF_TYPE__TAMPER_OPEN_EVENT,
    RADIO_TX_BUF_TYPE__TAMPER_CLOSE_EVENT,
    RADIO_TX_BUF_TYPE__DEVICE_INFO_EVENT,
    RADIO_TX_BUF_TYPE__SENSOR_EVENT,
} RADIO_TX_BUF_TYPE_IN_PROGRESS;

static RADIO_TX_BUF_TYPE_IN_PROGRESS radio_tx_buf_type_in_progress = RADIO_TX_BUF_TYPE__NONE;



// When set to true, the immediate next event is sent as a confirmed message instead
// of an unconfirmed message.   After the message is enqueued here, this flag is changed
// back to false.  This only affects the single following message.
//
// This only works if un-confirmed messages is configured.  However, this should
// never be set to true for Zone_EU until we understand if this is a good idea (and if
// it works).  This may change in the a future release V2.1 after more experimentation
// and testing.
bool radio_tx_buf_force_confirmed_event__set = false;
bool radio_tx_buf_force_confirmed_event__get = false;

// the following is set true when permanently disable the capability to force_confirmed_event
bool radio_tx_buf_force_confirmed_event__enable = false;

static uint8_t packet_count_server = 0;
static bool wait_for_ack_confirmation = false;

#ifdef ENABLE_AES

// when true, aes is enabled
bool aes_enable = false;
uint8_t aes_key_index = 0xFF;
uint32_t aes_count = 0;
uint8_t downlink_request_key_index = 0xFF;
uint32_t downlink_request_count = 0;
uint8_t xcrypt_buffer[AES_BLOCKLEN];

#endif // ENABLE_AES

//----------------------------------------------------------------------
void radio_tx_buf_init(void)
{
    tx_buf_downlink_ack_old_valid   = false;
    tx_buf_downlink_ack_new_valid   = false;
    tx_buf_reset_event_valid        = false;
    if (link_quality_message_enable)
    {
        tx_buf_link_quality_event_valid = false;
    }
    tx_buf_supervisory_event_valid  = false;
    tx_buf_tamper_open_event_valid  = false;
    tx_buf_tamper_close_event_valid = false;
    tx_buf_device_info_event_valid  = false;
    tx_buf_new_event_enqueued       = false;

    tx_buf_force_confirmed_event_downlink_ack_old_event = false;
    tx_buf_force_confirmed_event_downlink_ack_new_event = false;
    tx_buf_force_confirmed_event_reset_event            = false;

    if (link_quality_message_enable)
    {
        tx_buf_force_confirmed_event_link_quality_event  = false;
    }

    tx_buf_force_confirmed_event_supervisory_event   = false;
    tx_buf_force_confirmed_event_tamper_open_event   = false;
    tx_buf_force_confirmed_event_tamper_close_event  = false;
    tx_buf_force_confirmed_event_device_info_event   = false;

    tx_buf_sensor_events_head    = 0;
    tx_buf_sensor_events_tail    = 0;

    radio_tx_buf_type_in_progress = RADIO_TX_BUF_TYPE__NONE;

    radio_tx_buf_force_confirmed_event__set = false;
    radio_tx_buf_force_confirmed_event__get = false;
}
//----------------------------------------------------------------------
bool radio_tx_buf_new_event_enqueued(void)
{
  bool rc = tx_buf_new_event_enqueued;
  tx_buf_new_event_enqueued = false;
  return rc;
}

//----------------------------------------------------------------------
bool radio_tx_buf_new_join_wakeup_event_enqueued(void)
{
  return tx_buf_new_join_wakup_event_enqueued;
}

//----------------------------------------------------------------------
void radio_tx_buf_new_join_wakeup_event_clear(void)
{
    tx_buf_new_join_wakup_event_enqueued = false;

}

//----------------------------------------------------------------------
void radio_tx_buf_enqueue(uint8_t radio_event, bool downlink_request, bool join_wakeup)
{
    char * send_string;

    // grab current copy of force_confirmed_event and then reset it to false
    bool force_confirmed_event = (radio_tx_buf_force_confirmed_event__set && radio_tx_buf_force_confirmed_event__enable);
    radio_tx_buf_force_confirmed_event__set = false;

//    TRC_EVENT_ENQUEUE(radio_event, force_confirmed_event, Tx_string);


    // set flag indicating that a new event has been enqueued
    tx_buf_new_event_enqueued = true;

    if(join_wakeup)
        tx_buf_new_join_wakup_event_enqueued = true;

//    switch(radio_event)
//    {
//        case RADIO_EVENT_TYPE_DOWNLINK_ACK:
//            // if this is an old (original) ACK of FF01 or FF02, then we have to enqueue things in old queue
//            if ((Tx_string[4] == '0') && ((Tx_string[5] == '1') || (Tx_string[5] == '2')))
//            {
//                // get approprate buffer address of where to save the Tx_string
//                send_string = tx_buf_downlink_ack_old_event;
//                tx_buf_downlink_ack_old_valid                       = true;
//                tx_buf_force_confirmed_event_downlink_ack_old_event = force_confirmed_event;
//            }
//            else // this is an extended DOWNLINK ACK (11 byte message including DOWNLINK and possible error code )
//            {
//                // get approprate buffer address of where to save the Tx_string
//                send_string = tx_buf_downlink_ack_new_event;
//                tx_buf_downlink_ack_new_valid                       = true;
//                tx_buf_force_confirmed_event_downlink_ack_new_event = force_confirmed_event;
//            }
//
//            break;
//        case RADIO_RESET_EVENT:
//            // get approprate buffer address of where to save the Tx_string
//            send_string = tx_buf_reset_event;
//            tx_buf_reset_event_valid                            = true;
//            tx_buf_force_confirmed_event_reset_event            = force_confirmed_event;
//
//            break;
//        case RADIO_EVENT_TYPE_LINK_QUALITY:
//            // get approprate buffer address of where to save the Tx_string
//            send_string = tx_buf_link_quality_event;
//            tx_buf_link_quality_event_valid                     = true;
//            tx_buf_force_confirmed_event_link_quality_event     = true; // force LINK QUALITY to always be confirmed, even for EU868 and all other Zones
//
//            break;
//        case RADIO_SUPERVISORY:
//            // get approprate buffer address of where to save the Tx_string
//            send_string = tx_buf_supervisory_event;
//            tx_buf_supervisory_event_valid                      = true;
//            tx_buf_force_confirmed_event_supervisory_event      = force_confirmed_event;
//
//            break;
//        case RADIO_TAMPER:
//            if (downlink_request)
//            {
//                // get approprate buffer address of where to save the Tx_string
//                send_string = tx_buf_tamper_open_event;
//                tx_buf_tamper_open_event_valid                  = true;
//                tx_buf_force_confirmed_event_tamper_open_event  = force_confirmed_event;
//
//                //because we just enqueued a tamper_open event, then any previous tamper_close must be stale and is discarded here
//                tx_buf_tamper_close_event_valid  = false;
//            }
//            else
//            {
//                // get approprate buffer address of where to save the Tx_string
//                send_string = tx_buf_tamper_close_event;
//                tx_buf_tamper_close_event_valid                 = true;
//                tx_buf_force_confirmed_event_tamper_close_event = force_confirmed_event;
//            }
//
//            break;
//        case RADIO_EVENT_TYPE_DEVICE_INFO:
//            // get approprate buffer address of where to save the Tx_string
//            send_string = tx_buf_device_info_event;
//            tx_buf_device_info_event_valid                      = true;
//            tx_buf_force_confirmed_event_device_info_event      = force_confirmed_event;
//
//            break;
//        default:  // all other radio events
            // get approprate buffer address (from round robin tx_buf_sensor_events queue) of where to save the Tx_string
            send_string = tx_buf_sensor_events[tx_buf_sensor_events_head];

            tx_buf_radio_data[tx_buf_sensor_events_head] = (downlink_request & 0xFE);  // mask off the downlink request boolean 0x01

            // save the force_confirmed_event in the ring buffer
            tx_buf_force_confirmed_event_radio_data[tx_buf_sensor_events_head] = force_confirmed_event;

            // round robin queue that can safely wrap and discard the oldest values
            tx_buf_sensor_events_head = (uint8_t)((tx_buf_sensor_events_head + 1) % TX_BUF_SENSOR_EVENTS_MAX);

            // if the round-robin queue has wrapped...
            if (tx_buf_sensor_events_head == tx_buf_sensor_events_tail)
            {
                // then advance the tail by one to ensure that the queue always contains the
                // most recent events in the queue.  This step inherantly discards the oldest event
                // in the queue.
                tx_buf_sensor_events_tail = ((tx_buf_sensor_events_tail + 1) % TX_BUF_SENSOR_EVENTS_MAX);
            }

//            break;
//    }
#ifdef WIN32
    // windows (compiler fatal warning) wants to now force us to use new strcpy_s() that limits the size of what is written
    strcpy_s(send_string, sizeof(Tx_string), Tx_string);
#else
    strcpy(send_string, Tx_string);
#endif
}

void clear_tx_buf(void)
{
  tx_buf_sensor_events_head = tx_buf_sensor_events_tail;
  tx_buf_downlink_ack_old_valid = false;
  tx_buf_downlink_ack_new_valid = false;
  tx_buf_reset_event_valid = false;
  tx_buf_link_quality_event_valid = false;
  tx_buf_supervisory_event_valid = false;
  tx_buf_tamper_open_event_valid = false;
  tx_buf_tamper_close_event_valid = false;
  tx_buf_device_info_event_valid = false;
  uint8_t i=0;
  for(i=0; i<30; i++)
    {
    next_event_buf_string[i] = 0;
    }
  return;
}
//----------------------------------------------------------------------
bool radio_tx_buf_has_event_to_send(void)
{
//    if ((tx_buf_sensor_events_head != tx_buf_sensor_events_tail) ||
//        tx_buf_downlink_ack_old_valid     ||    // Original ACK
//        tx_buf_downlink_ack_new_valid     ||    // Extended ACK
//        tx_buf_reset_event_valid          ||
//        tx_buf_link_quality_event_valid   ||
//        tx_buf_supervisory_event_valid    ||
//        tx_buf_tamper_open_event_valid    ||
//        tx_buf_tamper_close_event_valid   ||
//        tx_buf_device_info_event_valid)

      if(tx_buf_sensor_events_head != tx_buf_sensor_events_tail)
        {
          return true;
        }
      else
        return false;
}
//----------------------------------------------------------------------
bool radio_tx_buf_can_device_info_be_sent(void)
{
    return ! tx_buf_device_info_event_valid;
}
//----------------------------------------------------------------------
bool radio_tx_buf_has_downlink_ack_to_send(void)
{
    return tx_buf_downlink_ack_old_valid || tx_buf_downlink_ack_new_valid;
}

#ifdef ENABLE_AES
//----------------------------------------------------------------------
static uint8_t radio_tx_buf_prepare_send_string_for_encryption(const char * send_string)
{
    memset(xcrypt_buffer, 0x00, AES_BLOCKLEN);
    uint8_t xcrypt_num_bytes = (uint8_t) strlen(send_string);
    xcrypt_num_bytes = (xcrypt_num_bytes - 2) / 2;
    for (int i = 0; i < xcrypt_num_bytes; i++)
    {
        xcrypt_buffer[i] = ascii_byte_to_hex((uint8_t *) &send_string[(i * 2) + 2]);
    }
    return xcrypt_num_bytes;
}
#endif // ENABLE_AES

#ifdef ENABLE_AES
//----------------------------------------------------------------------
static void radio_tx_buf_merge_xcrypt_buffer_into_buf_string(const char * send_string, uint8_t packet_count, char * buf_string_ptr, uint8_t xcrypt_num_bytes )
{
    uint8_t protocol_version = ascii_byte_to_hex((uint8_t *) &send_string[0]);
    protocol_version |= packet_count | PROTOCOL_VERSION_AES_FORMAT;
    hex_to_ascii_byte(protocol_version,                                     (uint8_t *) &buf_string_ptr[0]);
    hex_to_ascii_byte(((uint8_t) (aes_count >> 16)) | (aes_key_index << 5), (uint8_t *) &buf_string_ptr[2]);
    hex_to_ascii_byte(((uint8_t) (aes_count >>  8)),                        (uint8_t *) &buf_string_ptr[4]);
    hex_to_ascii_byte(((uint8_t) (aes_count)),                              (uint8_t *) &buf_string_ptr[6]);

    for (int i = 0; i < xcrypt_num_bytes; i++)
    {
        hex_to_ascii_byte(xcrypt_buffer[i], (uint8_t *) &buf_string_ptr[(i * 2) + 8]);
    }

    buf_string_ptr[(xcrypt_num_bytes * 2) + 8] = '\0'; // null terminated string
}
#endif // ENABLE_AES

//----------------------------------------------------------------------
// NOTE that buffer addressed by buf_string_ptr is changed by this function
// Also, this function returns downlink_request
bool radio_tx_buf_dequeue(char * buf_string_ptr /* OUTPUT PARAMETER */ )
{
    // at this point, we know that we MUST have a valid buffer to send because
    // radio_tx_buf_is_available() was just previously called and returned true
    char * send_string;
    bool downlink_request;

    // assume there isn't any radio_data, but then fill it in if there is below.
    tx_buf_radio_data_in_progress = 0x00;

//    if (tx_buf_downlink_ack_old_valid)    // Original DOWNLINK ACK
//    {
//        // get pointer to downlink_ack event
//        send_string = tx_buf_downlink_ack_old_event;
//
//        // downlink_ack events never send downlink_request
//        downlink_request = false;
//
//        // mark no longer valid
//        tx_buf_downlink_ack_old_valid = false;
//
//        // grab the value of this force_confirmed_event
//        radio_tx_buf_force_confirmed_event__get = tx_buf_force_confirmed_event_downlink_ack_old_event;
//
////        TRC_EVENT_DEQUEUE("Downlink Ack");
//        char_send('H',0);
//    }
//    else if (tx_buf_downlink_ack_new_valid)   // Extended DOWNLINK ACK
//    {
//        // get pointer to downlink_ack event
//        send_string = tx_buf_downlink_ack_new_event;
//
//        // downlink_ack events never send downlink_request
//        downlink_request = false;
//
//        // mark no longer valid
//        tx_buf_downlink_ack_new_valid = false;
//
//        // grab the value of this force_confirmed_event
//        radio_tx_buf_force_confirmed_event__get = tx_buf_force_confirmed_event_downlink_ack_new_event;
//
////        TRC_EVENT_DEQUEUE("Downlink Ack");
//        char_send('I',0);
//    }
//    else if (tx_buf_reset_event_valid)
//    {
//        // get pointer to reset event
//        send_string = tx_buf_reset_event;
//
//        // reset always sends downlink_request
//        downlink_request = true;
//
//        // grab the value of this force_confirmed_event
//        radio_tx_buf_force_confirmed_event__get = tx_buf_force_confirmed_event_reset_event;
//
//        // set type of tx_buf
//        radio_tx_buf_type_in_progress = RADIO_TX_BUF_TYPE__RESET_EVENT;
//
////        TRC_EVENT_DEQUEUE("Reset");
//        char_send('J',0);
//    }
//    else if (tx_buf_link_quality_event_valid)
//    {
//        // get pointer to link_quality event
//        send_string = tx_buf_link_quality_event;
//
//        // reset always sends downlink_request
//        downlink_request = true;
//
//        // grab the value of this force_confirmed_event
//        radio_tx_buf_force_confirmed_event__get = tx_buf_force_confirmed_event_link_quality_event;
//
//        // set type of tx_buf
//        radio_tx_buf_type_in_progress = RADIO_TX_BUF_TYPE__LINK_QUALITY_EVENT;
//
////        TRC_EVENT_DEQUEUE("Link Quality");
//        char_send('K',0);
//    }
//    else if (tx_buf_supervisory_event_valid)
//    {
//        // get pointer to supervisory event
//        send_string = tx_buf_supervisory_event;
//
//        // supervisory always sends downlink_request
//        downlink_request = true;
//
//        // grab the value of this force_confirmed_event
//        radio_tx_buf_force_confirmed_event__get = tx_buf_force_confirmed_event_supervisory_event;
//
//        // set type of tx_buf
//        radio_tx_buf_type_in_progress = RADIO_TX_BUF_TYPE__SUPERVISORY_EVENT;
//
////        TRC_EVENT_DEQUEUE("Supervisory");
//        char_send('L',0);
//    }
//    else if (tx_buf_tamper_open_event_valid)
//    {
//        // get pointer to tamper open event
//        send_string = tx_buf_tamper_open_event;
//
//        // tamper opened always sends downlink_request
//        downlink_request = true;
//
//        // grab the value of this force_confirmed_event
//        radio_tx_buf_force_confirmed_event__get = tx_buf_force_confirmed_event_tamper_open_event;
//
//        // set type of tx_buf
//        radio_tx_buf_type_in_progress = RADIO_TX_BUF_TYPE__TAMPER_OPEN_EVENT;
//
////        TRC_EVENT_DEQUEUE("Tamper Open");
//        char_send('M',0);
//    }
//    else if (tx_buf_tamper_close_event_valid)
//    {
//        // get pointer to tamper closed event
//        send_string = tx_buf_tamper_close_event;
//
//        // tamper closed is never sends downlink_request
//        downlink_request = false;
//
//        // grab the value of this force_confirmed_event
//        radio_tx_buf_force_confirmed_event__get = tx_buf_force_confirmed_event_tamper_close_event;
//
//        // set type of tx_buf
//        radio_tx_buf_type_in_progress = RADIO_TX_BUF_TYPE__TAMPER_CLOSE_EVENT;
//
////        TRC_EVENT_DEQUEUE("Tamper Closed");
//        char_send('N',0);
//    }
//    else if (tx_buf_sensor_events_head != tx_buf_sensor_events_tail)
//    {
        // get pointer to the next sensor event
        send_string = tx_buf_sensor_events[tx_buf_sensor_events_tail];

        // sensor events never send downlink_request
        downlink_request = false;

        // grab the value of this force_confirmed_event
        radio_tx_buf_force_confirmed_event__get = tx_buf_force_confirmed_event_radio_data[tx_buf_sensor_events_tail];

        // set type of tx_buf
        radio_tx_buf_type_in_progress = RADIO_TX_BUF_TYPE__SENSOR_EVENT;

        // grab the radio_data that was sent in enqueued with the message that is returned
        // when the radio event occurs
        tx_buf_radio_data_in_progress = tx_buf_radio_data[tx_buf_sensor_events_tail];

//        TRC_EVENT_DEQUEUE("Event");
//        char_send('O',0);
//    }
//    else if (tx_buf_device_info_event_valid)
//    {
//        // get pointer to tamper closed event
//        send_string = tx_buf_device_info_event;
//
//        // tamper closed is never sends downlink_request
//        downlink_request = false;
//
//        // grab the value of this force_confirmed_event
//        radio_tx_buf_force_confirmed_event__get = tx_buf_force_confirmed_event_device_info_event;
//
//        // set type of tx_buf
//        radio_tx_buf_type_in_progress = RADIO_TX_BUF_TYPE__DEVICE_INFO_EVENT;
//
////        TRC_EVENT_DEQUEUE("Device Info");
//        char_send('P',0);
//    }
//    else
//    {
//        // Setting the buf_string_ptr to null
//        send_string = (char *) "000000";  // just something stupid because recovery needs to be understood and figured out
//
//        // tamper closed is never sends downlink_request
//        downlink_request = false;
//
//        // don't send this unknown problem as a confirmed
//        radio_tx_buf_force_confirmed_event__get = false;
//
//        // set type of tx_buf
//        radio_tx_buf_type_in_progress = RADIO_TX_BUF_TYPE__NONE;
//
//        // something went wrong
////        TRC_EVENT_DEQUEUE("Fail: nothing to dequeue - ignored");
//        char_send('Q',0);
//    }

    // insert the packet_count_char hex value in the second byte after the protocol version
    uint8_t tx_buf_packet_count_char;
    uint8_t packet_count = radio_tx_buf_get_packet_count();

    if (packet_count < 10)
    {
        tx_buf_packet_count_char = packet_count + '0';
    }
    else
    {
        tx_buf_packet_count_char = packet_count + 'A' - 10;
    }

#ifdef ENABLE_AES

    if (aes_enable)
    {
        // if this message is sent with a downlink_request, then save the key and count to
        // be used to decrypt the DOWNLINK data.
        if (downlink_request)
        {
            downlink_request_key_index = aes_key_index;
            downlink_request_count = aes_count;
        }
        // convert the ASCII send_string into hex bytes and store in the xcrypt_buffer
        uint8_t xcrypt_num_bytes = radio_tx_buf_prepare_send_string_for_encryption(send_string);
        AES_set_iv(aes_count);
        AES_xcrypt_buffer(xcrypt_buffer, AES_BLOCKLEN);
        radio_tx_buf_merge_xcrypt_buffer_into_buf_string(send_string, packet_count, buf_string_ptr, xcrypt_num_bytes);
        aes_count++;
        aes_count &= 0x001FFFFF;  // ensures it doesn't wrap past 2 ^^ 21 (2 million)
    }
    else
    {
        // copy the send_string to the return buffer passed into this function
#ifdef WIN32
        // windows (compiler fatal warning) wants to now force us to use new strcpy_s() that limits the size of what is written
        strcpy_s(buf_string_ptr, sizeof(Tx_string), send_string);
#else
        strcpy(buf_string_ptr, send_string);
#endif
        // over-write the second byte of the return buffer with the packet_count character
        buf_string_ptr[1] = tx_buf_packet_count_char;
    }

#else // ENABLE_AES

    // copy the send_string to the return buffer passed into this function
#ifdef WIN32
    // windows (compiler fatal warning) wants to now force us to use new strcpy_s() that limits the size of what is written
    strcpy_s(buf_string_ptr, sizeof(Tx_string), send_string);
#else
    strcpy(buf_string_ptr, send_string);
#endif

    // over-write the second byte of the return buffer with the packet_count character
//    buf_string_ptr[1] = tx_buf_packet_count_char;

#endif // ENABLE_AES

    // return the downlink_request flag to the caller.
    return downlink_request;
}
//----------------------------------------------------------------------
uint8_t radio_tx_buf_get_packet_count(void)
{
    if ( ! wait_for_ack_confirmation)
    {
        wait_for_ack_confirmation = true;
    }
    return packet_count_server;
}
//----------------------------------------------------------------------
void radio_tx_buf_received_ack_confirmation(bool ack_received)
{
    if (wait_for_ack_confirmation)
    {
        packet_count_server++;
        packet_count_server &= 0x0F;
        wait_for_ack_confirmation = false;
    }

    switch (radio_tx_buf_type_in_progress)
    {
        case RADIO_TX_BUF_TYPE__RESET_EVENT:
            // mark no longer valid
            tx_buf_reset_event_valid = false;
            break;
        case RADIO_TX_BUF_TYPE__LINK_QUALITY_EVENT:
            // mark no longer valid
            tx_buf_link_quality_event_valid = false;
            break;
        case RADIO_TX_BUF_TYPE__SUPERVISORY_EVENT:
            // mark no longer valid
            tx_buf_supervisory_event_valid = false;
            break;
        case RADIO_TX_BUF_TYPE__TAMPER_OPEN_EVENT:
            // mark no longer valid
            tx_buf_tamper_open_event_valid = false;
            break;
        case RADIO_TX_BUF_TYPE__TAMPER_CLOSE_EVENT:
            // mark no longer valid
            tx_buf_tamper_close_event_valid = false;
            break;
        case RADIO_TX_BUF_TYPE__DEVICE_INFO_EVENT:
            // mark no longer valid
            tx_buf_device_info_event_valid = false;
            break;
        case RADIO_TX_BUF_TYPE__SENSOR_EVENT:
            // advance to next event in round robin sensor events queue
            tx_buf_sensor_events_tail = ((tx_buf_sensor_events_tail + 1) % TX_BUF_SENSOR_EVENTS_MAX);
            break;

        case RADIO_TX_BUF_TYPE__NONE:
        case RADIO_TX_BUF_TYPE__DOWNLINK_ACK:
        default: break;
    }
    radio_tx_buf_type_in_progress = RADIO_TX_BUF_TYPE__NONE;

    if (ack_received)
    {
        tx_buf_ack_handler(tx_buf_radio_data_in_progress);
    }
}
//----------------------------------------------------------------------
void radio_tx_buf_received_send_confirmation(void)
{
    tx_buf_send_handler(tx_buf_radio_data_in_progress);
}
//----------------------------------------------------------------------
void radio_tx_buf_set_send_handler(void (* send_handler)(uint8_t data))
{
    tx_buf_send_handler = send_handler;
}
//----------------------------------------------------------------------
void radio_tx_buf_default_send_handler(uint8_t data){
}
//----------------------------------------------------------------------
void radio_tx_buf_set_ack_handler(void (* ack_handler)(uint8_t data))
{
    tx_buf_ack_handler = ack_handler;
}
//----------------------------------------------------------------------
void radio_tx_buf_default_ack_handler(uint8_t data){
}
