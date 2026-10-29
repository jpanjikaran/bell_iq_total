/*
 * lora_xdto_radio.c
 *
 *  Created on: Mar. 17, 2022
 *      Author: jasonp
 */


/*
    Interface to LoRa radio module

  (c) 2017 Radio Bridge Inc. and its subsidiaries.
*/
//#include "main.h"
//
//#ifdef LORA
//#ifdef LORA_XDOT

#include "lora_xdot_radio.h"
//#include "timers.h"
#include "radio_common.h"
#include "downlink_messages.h"
//#include "eeprom.h"
//#include "supervisor.h"
//#include "utils.h"
#include "build_config.h"
#include "lora_api_internal_join.h"
#include "sl_bt_api.h"
#include "utility.h"
#include "em_common.h"
#include "sl_uartdrv_instances.h"
#include "api_eusart_radio.h"
#include "em_letimer.h"
#include "swmodbeacon.h"
#include "radio_tx_buf.h"
#include "collection.h"


//#define SLEEP_FIX
//#define _DEFINE_LORA_API_INTERNAL_DATA
    #include "lora_api_internal_data.h"  // Internal Data and Defines (Common Murata and xDot)
//#undef  _DEFINE_LORA_API_INTERNAL_DATA


static uint32_t join_attempts = 0;
static uint32_t join_sleep_index = 0;
uint8_t letmr_flag = 0;
extern bool my_sys_is_ready_to_sleep;

#define JOIN_ATTEMPTS_PER_SLEEP_LEVEL 16L // number of failed join attempts before advancing to the next sleep interval
#define TICKS_PER_SECOND 4UL // each tick is 0.250 seconds
#define TICKS_PER_MINUTE (60UL * TICKS_PER_SECOND)
#define TICKS_PER_HOUR (60UL * TICKS_PER_MINUTE)

#define TICKS_PER_0_SECONDS (0UL)
#define TICKS_PER_15_MINUTES (15UL * TICKS_PER_MINUTE)
#define TICKS_PER_30_MINUTES (30UL * TICKS_PER_MINUTE)
#define TICKS_PER_1_HOUR (1UL * TICKS_PER_HOUR)
#define TICKS_PER_2_HOURS (2UL * TICKS_PER_HOUR)
#define TICKS_PER_4_HOURS (4UL * TICKS_PER_HOUR)
#define TICKS_PER_8_HOURS (8UL * TICKS_PER_HOUR)
#define TICKS_PER_12_HOURS (12UL * TICKS_PER_HOUR)
extern uint8_t lora_dev_settings[182];
extern deviceDATA MYdevice;
//extern uint8_t first_event_flag;
extern uint8_t first_event_tx;
extern uint8_t first_event_tx_cnt;
extern bool send_lora_join_msg;
extern smartwavebcn     myBeacon;
extern uint8_t reg_flag;
extern battData DeviceBatt;
extern uint32_t lora_hb_cnt;
extern collectionType   coll;
extern MTsystem_t   MTsystem;
uint8_t ping_cnt;
uint32_t waiting_for_tx_delay = 0;
uint8_t tx_delay_expired_flag = 0;
uint8_t tx_delay_buf[10];
uint8_t tx_delay_buf_len = 0;

static uint32_t sleep_time_table[] = {
    TICKS_PER_0_SECONDS,
    TICKS_PER_15_MINUTES,
    TICKS_PER_30_MINUTES,
    TICKS_PER_1_HOUR,
    TICKS_PER_2_HOURS,
    TICKS_PER_4_HOURS,
    TICKS_PER_8_HOURS,
    TICKS_PER_12_HOURS
};

#define SLEEP_TIME_TABLE_COUNT (sizeof(sleep_time_table) / sizeof(sleep_time_table[0]))
char expected_freq_str[]={0,0,0,0,0};

bool radio_transmitting = false;
uint8_t rssi_flag = 0;
uint8_t ack_flag;

// number of expected bytes in a downlink, generally always 8.  Anything other than 8 results in an INVALID ACK  0xFF01
uint8_t lora_downlink_num_bytes = 0;

void api_eeprom_read_array_and_convert_to_string(uint8_t * dst_buf, uint8_t dst_len_in_bytes, uint16_t eeprom_offset, uint8_t eeprom_length_in_bytes)
{
  uint8_t buf[128];
    // first ensure destination buffer is large enough for the result.
    if ( dst_len_in_bytes < ((eeprom_length_in_bytes * 2) + 1))
    {
        // if the destination is not large enough, then return a null ('\0') terminated string
        dst_buf[0] = '\0';
        return;
    }

    // for every byte to be read and converted...
    ps_load(eeprom_offset, buf, eeprom_length_in_bytes);
    for (uint8_t i = 0; i < eeprom_length_in_bytes; i++)
    {
        // read the byte in eeprom, then convert the hex byte to two ascii characters
        hex_to_ascii_byte( buf[i], &dst_buf[i * 2]);
    }
    // terminate the ascii string to ensure it is a proper valid ascii C-String
    dst_buf[eeprom_length_in_bytes * 2] = '\0';
}
//---------------------------------------------------------------------------------
#define MAX_STR_LEN 3
static uint8_t dst_str[MAX_STR_LEN + 1]; // +1 is for '\0' character
uint8_t * convert_uint8_to_ascii_string(uint8_t data)    // dst_str MUST be of size >= 4 bytes
{
    // This function is equivalent to:
    //    #define MAX_STR_LEN 3
    //    static char dst_str[MAX_STR_LEN + 1];
    //    sprintf(dst_str, "%3d", data);
    //    return dst_str;
    // In other words, it returns a non-leading 0 string representation of data that is null terminated
    // Reason for this ugly code: sprintf library increases size of executable by crazy amounts.  While
    // not minimal code, it should work.  Performance isn't a concern.

    uint8_t digit;
    uint8_t divisor = 100;
    for (uint8_t i = 0; i < MAX_STR_LEN; i++)
    {
        digit = data / divisor;
        data = data % divisor;
        divisor /= 10;
        dst_str[i] = digit + '0';
    }
    dst_str[MAX_STR_LEN] = '\0';
    for (uint8_t i = 0; i < MAX_STR_LEN; i++)
    {
        if (dst_str[i] != '0')
        {
            return &dst_str[i];
        }
    }
    return dst_str;
}
//---------------------------------------------------------------------------
// initialize the LoRa module at power up
void lora_init(void)
{
  uint8_t config_zone;
  ps_load(CONFIG_ZONE_OFFSET, &config_zone, 1);
    // if we are in the EU or IN zone then we cannot use confirmed messages
    if ((config_zone == CONFIG_ZONE_LORA_EU) || (config_zone == CONFIG_ZONE_LORA_IN))
    {
        lora_confirmed_messages = false;
        if (link_quality_message_enable)
        {
            RADIO_TX_BUF_FORCE_CONFIRMED_EVENT__ENABLE(false);
        }
    }
    else
    {
        if (link_quality_message_enable)
        {
            RADIO_TX_BUF_FORCE_CONFIRMED_EVENT__ENABLE(true);
        }
    }
}



//---------------------------------------------------------------------------
static void link_quality_transmit_event()
{
    static const uint8_t SUB_BAND_AUTOMATIC = 0;

    reset_link_quality_periodic_timer();
    RADIO_TX_BUF_FORCE_CONFIRMED_EVENT__SET(true);
    radio_transmit_hex_data_6(PROTOCOL_VERSION_LINK_QUALITY,
                                RADIO_EVENT_TYPE_LINK_QUALITY,
                                SUB_BAND_AUTOMATIC, // deprecated
                                lora_last_downlink_rssi,
                                lora_last_downlink_snr,
                                false,
                                JOIN_NO_FORCE_WAKEUP);
}

//---------------------------------------------------------------------------
const char *lora_xdot_radio_get_dfreq_string(int zone)
{
    const char *result = NULL;

    switch(zone)
    {
        case CONFIG_ZONE_LORA_US:
            result = "US915";
            break;

        case CONFIG_ZONE_LORA_EU:
            result = "EU868";
            break;

        case CONFIG_ZONE_LORA_AU:
            result = "AU915";
            break;

        case CONFIG_ZONE_LORA_IN:
            result = "IN865";
            break;
    }

    return result;
}

//---------------------------------------------------------------------------
// main state machine for lora communication
bool lora_state_machine(void)
{
    // don't process state machine when we are waiting
    if(radio_state_delay_running)
        return true;    // returning true means the state_machine is still running and cannot be pre-empted

    if (link_quality_message_enable)
    {
        if (link_quality_periodic_timer_expired)
        {
            reset_link_quality_periodic_timer();
            link_quality_transmit_event();
        }
    }

    // process main state machine when state delay has expired
    switch(lora_state)
    {

        // ------------------------------------------------------------------
        // ------------------------------------------------------------------
        // INIT Sequence
        // ------------------------------------------------------------------
        // ------------------------------------------------------------------

        case LORA_STATE_RESET:
//            TRC_RADIO_SEQUENCE("RESET");
            // flush lingering chars out of buffer
//          char_send('a',0);
            radio_uart_flush();

            radio_is_sleeping = false;

            join_attempts = 0;
            join_sleep_index = 0;

            radio_uart_read_index = 0;
            radio_response_timeout = false;
            radio_downlink_timeout = false;

            radio_transmitting = false;
//
//            if (link_quality_message_enable)
//            {
//                reset_link_quality_periodic_timer();
//            }
//
            lora_join_received = false;
            lora_waiting_for_join = false;
            lora_api_internal_join__set_state(LORA_API_INTERNAL_JOIN__STATE__NOT_JOINED);
            lora_join_started = false;

            lora_error_received__tx_buf_full_of_mac_cmds = false;
            lora_error_received__xmit_in_progress        = false;
            lora_error_received__ack_not_received        = false;
            lora_error_received                          = false;
            lora_ok_received                             = false;
            lora_downlink_received                       = false;

            xmit_retry_attempt = 0;

            lora_waiting_for_ok = false;
            lora_waiting_for_error = false;
            lora_waiting_for_downlink = false;
            lora_waiting_for_string_and_ok = false;
            lora_string_received = false;
            lora_waiting_for_uint_and_ok = false;
            lora_uint_received = false;
            rssi_flag = 0;
            ack_flag = 0;

            lora_firmware_version_capture = false;

            lora_waiting_for_transmit_delay_to_elapse = false;
            MYdevice.lora.network_delay = 0;
            MYdevice.lora.network_delay_tm = 0;

            lora_adr_disable_changed = true;   // force adr to be initialized everytime the state machine is reset

            lora_waiting_for_rssi = false;
            lora_rssi_received = false;
            lora_waiting_for_snr = false;
            lora_snr_received = false;

            // reset the governor, may be set from previous join request and subsequent reset
            reset_radio_governor(RADIO_GOVERNOR_TIMEOUT_LORA);

            // must be set to Low before it is set to High when waking up the Radio
//            API_PIN__RADIO_SLEEPN__SetHigh();
            GPIO_PinModeSet(LORA_WAKE_PORT, LORA_WAKE_PIN, gpioModePushPull     , 1);

            // put lora module in reset
//            API_PIN__RADIO_RSTN__SetLow();
            GPIO_PinModeSet(LORA_RST_PORT, LORA_RST_PIN, gpioModePushPull     , 0);

            // RADIO_RSTN wait 0.5 sec before setting it High
            set_radio_state_delay(5);
            lora_state = LORA_STATE_RELEASE_RESET;
            break;

        case LORA_STATE_RELEASE_RESET:
            // release reset pin
//            API_PIN__RADIO_RSTN__SetHigh();
          GPIO_PinModeSet(LORA_RST_PORT, LORA_RST_PIN, gpioModePushPull     , 1);

            // RADIO_RSTN wait 2.0 sec before sending any commands
            set_radio_state_delay(40);

            // flush any characters from initial "enter" such as error codes
            radio_uart_flush();
            lora_state = LORA_STATE_GET_AT;
            break;

        case LORA_STATE_GET_AT:
            radio_send_string("AT\r", 3);
//          radio_send_string("AT\r");
            lora_state = LORA_STATE_WAIT_FOR_OK;
            if(MYdevice.registration==1||MYdevice.lora.join_test==1)
              lora_next_state = LORA_STATE_SET_FSB;
            else
              lora_next_state = LORA_STATE_DISABLE_ECHO;
//            lora_next_state = LORA_STATE_GET_FIRMWARE_VERSION;
            break;

        case LORA_STATE_DISABLE_ECHO:
          radio_send_string("ATE0\r", 5);
          lora_state = LORA_STATE_WAIT_FOR_OK;
          lora_next_state = LORA_STATE_SET_WAKE_MODE;
          break;

        case LORA_STATE_SET_FSB:
//          radio_send_string("AT+FSB=1\r", 9);
          if(MYdevice.lora.join_test==0)
            {
              radio_send_string("AT+FSB=1\r", 9);
            }
          else
            {
              if((MYdevice.registration!=0x03)||(MYdevice.lora.install_join==1))
                {
                  radio_send_string("AT+FSB=1\r", 9);
                }
              else
                radio_send_string("AT+FSB=3\r", 9);
            }

            lora_state = LORA_STATE_WAIT_FOR_OK;
            lora_next_state = LORA_STATE_DIS_NONCE;
            break;

        case LORA_STATE_DIS_NONCE:
          if((MYdevice.registration!=0x03)||(MYdevice.lora.install_join==1))
            radio_send_string("AT+JNV=0\r", 9);
          else
            radio_send_string("AT+JNV=0\r", 9);


            lora_state = LORA_STATE_WAIT_FOR_OK;
            if(MYdevice.registration != 1)
              lora_next_state = LORA_STATE_GET_FIRMWARE_VERSION;
            else
              lora_next_state = LORA_STATE_SET_DEFAULT_FREQUENCY_BAND;
//              lora_next_state = LORA_STATE_SET_DEVICE_ID;
            break;

        case LORA_STATE_GET_FIRMWARE_VERSION:

            lora_state = LORA_STATE_QUERY_FIRMWARE_VERSION;
//            lora_next_state = LORA_STATE_GET_DEFAULT_FREQUENCY_BAND;
            lora_next_state = LORA_STATE_SET_DEFAULT_FREQUENCY_BAND;
            break;

        case LORA_STATE_GET_DEFAULT_FREQUENCY_BAND:
            radio_send_string("AT+FREQ?\r", 9);

            // initialize a SAFE_C_STR structure to safely accumulate the results of the ATI command
            SAFE_C_STR_INIT( & safe_c_str, temp_work_buf);

            lora_state = LORA_STATE_WAIT_FOR_STRING_AND_OK;
//            lora_state = LORA_STATE_WAIT_FOR_OK;
            lora_next_state = LORA_STATE_SET_DEFAULT_FREQUENCY_BAND;
            break;

        case LORA_STATE_SET_DEFAULT_FREQUENCY_BAND:
        {
          uint8_t config_zone;
          ps_load(CONFIG_ZONE_OFFSET, &config_zone, 1);
//          MYdevice.lora.region = 1;
//          MYdevice.lora.region = config_zone;
//            const char *expected_freq_str = lora_xdot_radio_get_dfreq_string(config_zone);


            switch(MYdevice.lora.region)
            {
                case CONFIG_ZONE_LORA_US:
//                  expected_freq_str[] = "US915";
                  strcpy(expected_freq_str, "US915");
                    break;

                case CONFIG_ZONE_LORA_EU:
//                  expected_freq_str[] = "EU868";
                  strcpy(expected_freq_str, "EU868");
                    break;

                case CONFIG_ZONE_LORA_AU:
//                  expected_freq_str[] = "AU915";
                  strcpy(expected_freq_str, "AU915");
                    break;

                case CONFIG_ZONE_LORA_IN:
//                  expected_freq_str[] = "IN865";
                  strcpy(expected_freq_str, "IN865");
                    break;
//                default:
//                  strcpy(expected_freq_str, "US915");
//                  break;
            }

            // compare against length of expected frequency string since reported the result returns back '\r'
            // set the frequency band only if the band on the modem is different to prevent wear and tear
            // on non-volatile storage of the maxim processor

            // generally this should only be updated once during the lifetime of the device
            if(expected_freq_str == NULL)
            {
                // invalid EEPROM config
//                TRC_LABEL_VALUE_HEX_UINT8("Unsupported Zone in EEPROM", CONFIG_ZONE);
                lora_state = LORA_STATE_RESET;
            }
            else if(strncmp(temp_work_buf, expected_freq_str, strlen(expected_freq_str)) == 0)
            {
               // frequency matches what is stored in PIC EEPROM, don't change
                if(MYdevice.sku==0xFF)
                  lora_state = LORA_STATE_SET_ATZ;
                else
                  lora_state = LORA_STATE_SET_WAKE_MODE;
            }
            else
            {
                radio_send_string("AT+DFREQ=", 9);
                radio_send_string(expected_freq_str, sizeof(expected_freq_str));
                radio_send_string("\r", 1);
                lora_state = LORA_STATE_WAIT_FOR_OK;
                lora_next_state = LORA_STATE_STORE_DEFAULT_FREQUENCY_BAND;
            }

            break;
        }

        case LORA_STATE_STORE_DEFAULT_FREQUENCY_BAND:
                radio_send_string("AT&W\r", 5);

            lora_state = LORA_STATE_WAIT_FOR_OK;
            lora_next_state = LORA_STATE_SET_ATZ;
            break;

        case LORA_STATE_SET_ATZ:
            radio_send_string("ATZ\r", 4);
            lora_state = LORA_STATE_WAIT_FOR_OK;
            lora_next_state = LORA_STATE_SET_ATZ_WAIT;
            break;

        case LORA_STATE_SET_ATZ_WAIT:
            set_radio_state_delay(20); // wait at least 1.5s for ATZ command to take effect
            if(MYdevice.registration != 1)
              lora_state = LORA_STATE_SET_WAKE_MODE;
            else
              lora_state = LORA_STATE_SET_DEVICE_ID;
            break;

        case LORA_STATE_SET_WAKE_MODE:
            // interrupt wake up mode
//            radio_send_string("AT+WM=1\r", 8);
            radio_send_string("AT+WM=2\r", 8);

            lora_state = LORA_STATE_WAIT_FOR_OK;
//            lora_next_state = LORA_STATE_SET_WAKE_PIN;
            lora_next_state = LORA_STATE_SET_WAKE_DELAY_INTERVAL;
            lora_next_next_state = LORA_STATE_GO_TO_SLEEP;
            break;

        case LORA_STATE_SET_WAKE_MODE_DELAY:
            // interrupt wake up mode
            radio_send_string("AT+WM=0\r", 8);

            lora_state = LORA_STATE_WAIT_FOR_OK;
            lora_next_state = LORA_STATE_SET_WAKE_PIN;
//            lora_next_state = LORA_STATE_SET_DEVICE_ID;
            break;

        case LORA_STATE_SET_WAKE_DELAY_INTERVAL:
//          send_string("wake interval");
          if(lora_waiting_for_transmit_delay_to_elapse)
            {
              radio_send_string("AT+WI=", 6);
              if(tx_delay_buf_len == 1)
                {
                  if(tx_delay_buf[0] == 0x31)
                    tx_delay_buf[0] = 0x32;
                }
              radio_send_string(tx_delay_buf, tx_delay_buf_len);
              radio_send_string("\r", 1);
              lora_state = LORA_STATE_WAIT_FOR_OK;
              lora_next_state = LORA_STATE_GO_TO_SLEEP;
//              lora_next_next_state = LORA_STATE_GO_TO_SLEEP;
//              lora_next_state = lora_next_next_state = LORA_STATE_GO_TO_SLEEP_CONT_2
//              send_string("wake interval1");
            }
          else
            {
              radio_send_string("AT+WI=86400\r", 12);
              lora_state = LORA_STATE_WAIT_FOR_OK;
              lora_next_state = lora_next_next_state;

            }
//          if(tx_delay_expired_flag&&(lora_waiting_for_transmit_delay_to_elapse==false))
//            {
//              tx_delay_expired_flag = 0;
//              send_string("start_join");
//              if ( ! lora_api_internal_join__is_network_joined()||MYdevice.lora.rejoin==1)
//                lora_next_state = LORA_STATE_JOIN_START;
//              else
//                {
//                  if(lora_msg_payload.num_of_msg)
//                    lora_next_state = LORA_STATE_TRANSMIT_START;
//                  else
//                    lora_next_state = LORA_STATE_GO_TO_SLEEP;
//                }
//
//            }
//
////          else if (!lora_waiting_for_transmit_delay_to_elapse)
////            {
////              if ( ! lora_api_internal_join__is_network_joined()||MYdevice.lora.rejoin==1)
////                lora_next_state = lora_next_next_state;
////              else
////                lora_next_state = LORA_STATE_SET_WAKE_PIN;
////                send_string("wake interva2");
////            }
//          else if ( ! lora_api_internal_join__is_network_joined()||MYdevice.lora.rejoin==1)
//            {
//              if(!lora_waiting_for_transmit_delay_to_elapse)
//                {
//              lora_state = LORA_STATE_WAIT_FOR_OK;
////              lora_next_state = lora_next_next_state;
//              if(lora_msg_payload.num_of_msg)
//                lora_next_state = lora_next_next_state;
//              else
//                lora_next_state = LORA_STATE_GO_TO_SLEEP;
//              send_string("wake interval3");
//                }
//            }

        break;

        case LORA_STATE_SET_WAKE_PIN:

            // wake up when WAKE pin is toggled
            radio_send_string("AT+WP=6,2,1\r", 12);

            lora_state = LORA_STATE_WAIT_FOR_OK;
            if(MYdevice.registration==1||MYdevice.lora.join_test==1)
              lora_next_state = LORA_STATE_SET_FSB;
            else
              lora_next_state = LORA_STATE_GO_TO_SLEEP;
//            if(MYdevice.production_mode == 1)
//              lora_next_state = LORA_STATE_GO_TO_SLEEP;
//            else
//              lora_next_state = LORA_STATE_SET_DEVICE_ID;
            break;

        case LORA_STATE_SET_DEVICE_ID:
            // send read ID AT command
          api_eeprom_read_array_and_convert_to_string(temp_work_buf, sizeof(temp_work_buf), EEPROM_DEVICE_ID_OFFSET, EEPROM_DEVICE_ID_LEN_IN_BYTES);
            radio_send_string("AT+DI=", 6);
            radio_send_string(temp_work_buf, 16);
            radio_send_string("\r", 1);

            lora_state = LORA_STATE_WAIT_FOR_STRING_AND_OK;
            lora_next_state = LORA_STATE_SET_REP;
            break;

        case LORA_STATE_SET_REP:

            if ( ! lora_confirmed_messages)
            {
                // unconfirmed message retransmission - always use 1
                radio_send_string("AT+REP=1\r", 9);

                lora_state = LORA_STATE_WAIT_FOR_OK;
                lora_next_state = LORA_STATE_SET_MODE;
            }
            else
            {
                lora_state = LORA_STATE_SET_MODE;
            }
            break;

        case LORA_STATE_SET_MODE:

            // over the air activation, no personalization
            radio_send_string("AT+NJM=1\r", 9);

            lora_state = LORA_STATE_WAIT_FOR_OK;
            lora_next_state = LORA_STATE_SET_RFPOWER;
            break;

        case LORA_STATE_SET_RFPOWER:

            // set power levels to 20dBm
          if(MYdevice.lora.region==2)
            {
              radio_send_string("AT+TXP=14\r", 10);
              MYdevice.lora.tx_pwr=14;
            }
          else
            {
              radio_send_string("AT+TXP=20\r", 10);
              MYdevice.lora.tx_pwr=20;
            }

            lora_state = LORA_STATE_WAIT_FOR_OK;
            lora_next_state = LORA_STATE_SET_PUBLIC_NETWORK;
            break;

        case LORA_STATE_SET_PUBLIC_NETWORK:

            // set to public network mode
            radio_send_string("AT+PN=1\r", 8);

            lora_state = LORA_STATE_WAIT_FOR_OK;
            lora_next_state = LORA_STATE_SET_KEY;
            break;

        case LORA_STATE_SET_KEY:
            // verify that the key stored in eeprom is valid
            app_key_valid = validate_app_key();

            if ( ! app_key_valid)
            {
                lora_state = LORA_STATE_GO_TO_SLEEP;
                break;
            }
            if(MYdevice.lora.join_test==0)
              {
            radio_send_string("AT+NK=0,", 8);

            radio_send_string(lora_app_key_string, 32);
              }
            else
              {
                if((MYdevice.registration!=0x03)||(MYdevice.lora.install_join==1))
                  {
                    radio_send_string("AT+NK=0,", 8);

                    radio_send_string(lora_app_key_string, 32);

                  }
                else
                  radio_send_string("AT+NK=0,9da2386bea45b2b1f16973b47e0daa3c", 40);
              }
            radio_send_string("\r", 1);

            lora_state = LORA_STATE_WAIT_FOR_OK;
            lora_next_state = LORA_STATE_SET_APP_EUI;

            break;

        case LORA_STATE_SET_APP_EUI:
            // verify that the key stored in eeprom is valid
            app_eui_valid = validate_app_eui();
            if ( ! app_eui_valid)
            {
                lora_state = LORA_STATE_GO_TO_SLEEP;
                break;
            }
            if(MYdevice.lora.join_test==0)
              {
                radio_send_string("AT+NI=0,", 8);
                radio_send_string(lora_app_eui_string, 16);
              }
            else
              {
                if((MYdevice.registration!=0x03)||(MYdevice.lora.install_join==1))
                  {
                    radio_send_string("AT+NI=0,", 8);
                    radio_send_string(lora_app_eui_string, 16);
                    MYdevice.lora.install_join = 0;
                  }
                else
                  radio_send_string("AT+NI=0,0202020202020202", 24);
              }
            radio_send_string("\r", 1);

            lora_state = LORA_STATE_WAIT_FOR_OK;
//            if(MYdevice.registration==1)
              lora_next_state = LORA_STATE_JOIN_START;
//            else
//              lora_state = LORA_STATE_GO_TO_SLEEP;
            break;

        // ------------------------------------------------------------------
        // ------------------------------------------------------------------
        // JOIN Sequence
        // ------------------------------------------------------------------
        // ------------------------------------------------------------------

        case LORA_STATE_JOIN_START:
//            TRC_RADIO_SEQUENCE("JOIN");

            lora_state = LORA_STATE_QUERY_TRANSMIT_DELAY;
            lora_next_next_state = LORA_STATE_JOIN_ATTEMPT;

            break;

        case LORA_STATE_JOIN_ATTEMPT:
            radio_transmitting = true;
            // This will attempt to join on a random channel within one subband.
            // Subsequent calls will join on other subbands sequentially
            radio_send_string("AT+JOIN\r", 8);
            MYdevice.lora.join_attempts++;
            lora_state = LORA_STATE_JOIN_WAIT;
            MYdevice.lora.join_status = 2;
            break;

        case LORA_STATE_JOIN_WAIT:
            // start the join network timer and then wait
            start_radio_response_timeout(RADIO_RESPONSE_TIMEOUT_JOIN__XDOT_LORA_1_0);

            // will be set to true when the response for joining network is received
            lora_join_received = false;

            // indicates we are waiting for the id
            lora_waiting_for_join = true;

            lora_state = LORA_STATE_JOIN_WAITING;
            break;

        case LORA_STATE_JOIN_WAITING:
            // if the response doesn't come and we timeout waiting,
            // then reset the module
//          char_send('A', 0);
            if(radio_response_timeout)
            {
//                char_send('B', 0);
//                radio_response_has_timed_out = true;
//                lora_waiting_for_join = false;
                lora_api_internal_join__restart();
                lora_state = LORA_STATE_RESET;
//                TRC_RADIO_RESET();
            }
            else if(lora_join_received)
            {
//                char_send('C', 0);
//                lora_waiting_for_join = false;
                stop_radio_response_timeout();

                if (lora_api_internal_join__is_network_joined())
                {
                    join_attempts = 0;
//                    join_sleep_index = 0;
                    MYdevice.lora.join_begin=0;
//                    MYdevice.lora.join_test=0;
                    MYdevice.lora.join_delay = 0;
                    MYdevice.lora.join_delay_flag = 0;
                    MYdevice.lora.rejoin_attempt = 4;
                    MYdevice.lora.rejoin = 0;
                    if(MYdevice.lora.initial_join == 0)
                      {
                        MYdevice.lora.manual_join_delay_tm = 0;
                        MYdevice.lora.initial_join = 1;
                      }
                    lora_api_internal_join__successful();

                    if (link_quality_message_enable)
                    {
                        // since we just JOINed, reset the link quality periodic timer
                        reset_link_quality_periodic_timer();
                    }

                    // set governor to 250 MS before a transmission
                    reset_radio_governor(RADIO_GOVERNOR_DELAY_AFTER_JOIN);

                    // in this path we are definately done with the JOIN, so clear the radio_transmitting flag.
//                    radio_transmitting = false;
//                    send_lora_join_msg=true;
                    if(MYdevice.lora.join_test==1)
                      {

//                        lora_state = LORA_STATE_QUERY_RSSI;

                        //comment out below for test 20251120

                        MYdevice.lora.join_test = 0;
                        MYdevice.lora.ping_result=3;
//                        if(MYdevice.lora.reg_payload_sent == 1)
//                          {
//                            MYdevice.lora.ping_result=3;
//                            ping_cnt = 0;
//                            lora_state = LORA_STATE_PING;
////                            char_send('C',0);
//                          }
//                        else
//                          {
//                            if(MYdevice.lora.payload==0x01)
//                              {
//                                radio_transmit_hex_data_12(0x0A,                              /*communication schema*/
//                                                           MYdevice.sku,                      /*device type*/
//                                                           MYdevice.fw_ver,                              /*firmware version*/
//                                                           ((MYdevice.tx_pwr&0x00FF)>>8)+128, /*transmit power*/
//                                                           myBeacon.RRSSI,                    /*device mode*/
//                                                           MTsystem.temperaturecurrent,       /*temperature*/
//                                                           DeviceBatt.batterylvl,             /*battery level*/
//                                                           coll.EventsCounter,                /*events*/
//                                                           MYdevice.lora.pkt_fail,            /*failed lora transmissions*/
//                                                           MYdevice.lora.join_attempts,       /*join attempts*/
//                                                           0x00,                              /*group ID byte 1*/
//                                                           false,
//                                                           true);
//                              }
//                            else if(MYdevice.lora.payload == 0x02)
//                              {
//                                radio_transmit_hex_data_12(0x07, 0xCD, myBeacon.devMACadd[0], myBeacon.devMACadd[1], myBeacon.devMACadd[2], myBeacon.devMACadd[3], myBeacon.devMACadd[4], myBeacon.devMACadd[5],0x05, MYdevice.sku,MYdevice.fw_ver,false, true);
                        radio_transmit_gen_hex(0, 0x55);
//                        radio_transmit_gen_hex(1, 0xAA);
                        radio_transmit(0x01, false, true);
//                              }
                              lora_state = LORA_STATE_TRANSMIT_START;
                              MYdevice.lora.reg_payload_sent = 1;


                              //end of comment out 20251120
//                              char_send('A',0);
//                          }
                      }
                    else
                      {
//                        if(MYdevice.lora.payload==0x01)
//                          {
//                            radio_transmit_hex_data_12(0x0A,                              /*communication schema*/
//                                                       MYdevice.sku,                      /*device type*/
//                                                       MYdevice.fw_ver,                              /*firmware version*/
//                                                       ((MYdevice.tx_pwr&0x00FF)>>8)+128, /*transmit power*/
//                                                       myBeacon.RRSSI,                    /*device mode*/
//                                                       MTsystem.temperaturecurrent,       /*temperature*/
//                                                       DeviceBatt.batterylvl,             /*battery level*/
//                                                       coll.EventsCounter,                /*events*/
//                                                       MYdevice.lora.pkt_fail,            /*failed lora transmissions*/
//                                                       MYdevice.lora.join_attempts,       /*join attempts*/
//                                                       0x00,                              /*group ID byte 1*/
//                                                       false,
//                                                       true);
//                          }
//                        else if(MYdevice.lora.payload == 0x02)
//                          {
                            radio_transmit_hex_data_12(0x07, 0xCD, myBeacon.devMACadd[0], myBeacon.devMACadd[1], myBeacon.devMACadd[2], myBeacon.devMACadd[3], myBeacon.devMACadd[4], myBeacon.devMACadd[5],0x05, MYdevice.sku,MYdevice.fw_ver,false, true);
//                          }

                        lora_state = LORA_STATE_TRANSMIT_START;
                        MYdevice.lora.reg_payload_sent = 1;
//                        char_send('B',0);
                      }
                }
                else
                {
                    lora_api_internal_join__unsuccessful();
//                    char_send('D', 0);

                    // stay at the current sleep zone and reset the join attempts counter
                    // if an important message is pending.

//                    if(MYdevice.sku != 0xFF)
//                      {
                        join_attempts++;
//                        char_send(join_attempts, 1);
                        MYdevice.lora.join_failed++;




//                        char_send('H',0);

    //                    if(join_attempts < JOIN_ATTEMPTS_PER_SLEEP_LEVEL)
                        if((join_attempts < 2)&&(MYdevice.sku != 0xFF)&&(MYdevice.lora.join_test==0))
                        {
    //                        char_send('H', 0);
                            lora_state = LORA_STATE_JOIN_START;
//                            char_send('F', 0);
                        }
//                        else if((MYdevice.lora.join_test)&&(MYdevice.registration == 0x03)&&(join_attempts < 2))
//                          {
//                            lora_state = LORA_STATE_JOIN_START;
//                            MYdevice.lora.join_test = 0;
//                          }
                        else
                        {
//                            char_send('E', 0);
                            // advance to the next sleep level
                            bool pending_msg = radio_tx_buf_new_join_wakeup_event_enqueued();

                            // this flag is cleared after the join attempts have been attempted
                            // to keep us in the same sleep zone.  this is necessary to avoid
                            // quickly jumping to the longest sleep state
                            radio_tx_buf_new_join_wakeup_event_clear();

    //                        join_attempts = 0;
                            if((MYdevice.lora.rejoin_attempt>3)&&(radio_tx_buf_has_event_to_send()))
                              {
                              clear_tx_buf();
    //                          char_send('B',0);
                              }
                            if(MYdevice.lora.rejoin)
                              MYdevice.lora.rejoin=0;
                            MYdevice.lora.join_test = 0;
                            MYdevice.lora.join_status = 0;
                            if(MYdevice.lora.ping_result==3)
                              MYdevice.lora.ping_result = 1;
                            if(MYdevice.lora.rejoin_attempt==0)
                              MYdevice.lora.manual_join_delay_tm = 1500;

    //                        if(!pending_msg && join_sleep_index < SLEEP_TIME_TABLE_COUNT - 1)
    //                            join_sleep_index++;

                            // Macros are set to perform 16 join attempts without sleeping.
                            // After 16 attempts, the device will sleep and retry another 16 attempts.
                            // See sleep_time_table[] for more details on intervals.
    //                        uint32_t sleep_ticks = sleep_time_table[join_sleep_index];
                            uint32_t sleep_ticks = 0;
                            if(MYdevice.sku!=0xFF)
                              {
                            switch (MYdevice.lora.rejoin_attempt)
                            {
                              case  0:
    //                            MYdevice.lora.join_attempts++;
    //                            if(MYdevice.lora.join_attempts<2)
    //                              {
                                   send_string("jd1");
                                    MYdevice.lora.join_delay_flag=1;
                                    MYdevice.lora.join_delay = 150; //try again in 5min
                                    MYdevice.lora.rejoin_attempt++;
    //                              }
                                break;
                              case  1:
                                send_string("jd2");
                                    MYdevice.lora.join_delay_flag=1;
                                    MYdevice.lora.join_delay = 450; //try again in 15min
                                    MYdevice.lora.rejoin_attempt++;
                                break;
                              case  2:
                                send_string("jd3");
                                    MYdevice.lora.join_delay_flag=1;
                                    MYdevice.lora.join_delay = 900; //try again in 30min
                                    MYdevice.lora.rejoin_attempt++;
                                break;
                              case  3:
                                send_string("jd4");
                                    MYdevice.lora.join_delay_flag=1;
                                    MYdevice.lora.join_delay = 3600; //try again in 120min
                                    MYdevice.lora.rejoin_attempt++;
                                break;
                              default:
                                break;
                            }
                              }
    //                        if(MYdevice.lora.rejoin_attempt==0)
    //                          {
    //                           MYdevice.lora.join_attempts++;
    //                           if(MYdevice.lora.join_attempts<2)
    //                             {
    //                               MYdevice.lora.join_delay_flag=1;
    //                               MYdevice.lora.join_delay = 29;
    //                               MYdevice.lora.rejoin_attempt++;
    //                             }
    //                          }
                            reset_radio_governor(sleep_ticks);
                            lora_state = LORA_STATE_GO_TO_SLEEP;
                        }
//                      }
//                    else
//                      {
//                        radio_tx_buf_new_join_wakeup_event_clear();
//                        reset_radio_governor(1);
//                        lora_state = LORA_STATE_GO_TO_SLEEP;
//                      }

                    char buf[64];
                    sprintf(buf, "JOIN COUNTERS: %lu:%lu", join_sleep_index, join_attempts);
//                    TRC_RADIO_INFO(buf);
                }
            }
            break;

        // ------------------------------------------------------------------
        // ------------------------------------------------------------------
        // TRANSMIT Sequence
        // ------------------------------------------------------------------
        // ------------------------------------------------------------------
        case LORA_STATE_TRANSMIT_START:
//            TRC_RADIO_SEQUENCE("SEND EVENT");


            // if we are not continuing an xmit_retry_attempt, then re-initialize the various retry attempt counts
            if (xmit_retry_attempt == 0)
            {
                // set the retry counts for the various types of errors
                xmit_retry_attempt__tx_buf_full_of_mac_cmds = TOTAL_RETRY_ATTEMPTS__RX_BUF_FULL_OF_MAC_CMDS;
                xmit_retry_attempt__xmit_in_progress        = TOTAL_RETRY_ATTEMPTS__XMIT_IN_PROGRESS;
                xmit_retry_attempt__ack_not_received        = TOTAL_RETRY_ATTEMPTS__ACK_NOT_RECEIVED;
            }

            // if EU or IN Zone then don't set data rate or rf power
            uint8_t config_zone;
            ps_load(CONFIG_ZONE_OFFSET, &config_zone, 1);
//            if ((CONFIG_ZONE == CONFIG_ZONE_LORA_EU) || (CONFIG_ZONE == CONFIG_ZONE_LORA_IN))
            if ((config_zone == CONFIG_ZONE_LORA_EU) || (config_zone == CONFIG_ZONE_LORA_IN))
                lora_state = LORA_STATE_TRANSMIT_PAYLOAD;
            else
            {
                lora_state = LORA_STATE_QUERY_DATA_RATE;
                lora_next_state = LORA_STATE_TRANSMIT_SET_DATA_RATE;
            }
//            if ((config_zone == CONFIG_ZONE_LORA_EU) || (config_zone == CONFIG_ZONE_LORA_IN))
//                lora_state = LORA_STATE_TRANSMIT_PAYLOAD;
//            else
//            {
//                lora_state = LORA_STATE_QUERY_PAYLOAD;
//                lora_next_state = LORA_STATE_QUERY_DATA_RATE;;
//            }

            break;

        case LORA_STATE_TRANSMIT_SET_DATA_RATE:
            // If ADR is Enabled, then before we can start a transmit, we must first
            // ensure that the  DATA_RATE (DR) is '1' or '9'.  So it was just previously queued
            // and if it is '0' then we change it to '1';
            // However, if ADR is Disabled, then we simply always set the Data Rate to '1';
            if ((lora_data_rate_value == '0') || lora_adr_disable)
            {
                // second to lowest data rate since lowest data rate had a max payload of 7 bytes
                if(MYdevice.lora.region==3)
                  radio_send_string("AT+TXDR=2\r", 10);
                else
                  {
//                    if(MYdevice.lora.join_test)
                    if(MYdevice.registration == 0x03)
                      {
                        radio_send_string("AT+TXDR=2\r", 10);
//                        MYdevice.lora.join_test = 0;
                      }
                    else
                      radio_send_string("AT+TXDR=0\r", 10);
                  }
                lora_state = LORA_STATE_WAIT_FOR_OK;
                lora_next_state = LORA_STATE_TRANSMIT_SET_RFPOWER;
            }
            else
            {
                if(MYdevice.lora.region==3)
                  radio_send_string("AT+TXDR=2\r", 10);
                else
                  {
//                    if(MYdevice.lora.join_test)
                    if(MYdevice.registration == 0x03)
                      {
                        radio_send_string("AT+TXDR=2\r", 10);
//                        MYdevice.lora.join_test = 0;
                      }
                    else
                      radio_send_string("AT+TXDR=0\r", 10);
                  }
                lora_state = LORA_STATE_WAIT_FOR_OK;
                lora_next_state = LORA_STATE_TRANSMIT_SET_RFPOWER;
            }
            break;

        case LORA_STATE_TRANSMIT_SET_RFPOWER:
            // set power levels to 20dBm
          if(MYdevice.lora.region==2)
              radio_send_string("AT+TXP=14\r", 10);
          else
            radio_send_string("AT+TXP=20\r", 10);
            lora_state = LORA_STATE_WAIT_FOR_OK;
            lora_next_state = LORA_STATE_TRANSMIT_PAYLOAD;
            break;

        case LORA_STATE_TRANSMIT_PAYLOAD:
            lora_state = LORA_STATE_QUERY_TRANSMIT_DELAY;
            lora_next_next_state = LORA_STATE_TRANSMIT_PAYLOAD_CONT;
            break;

        case LORA_STATE_TRANSMIT_PAYLOAD_CONT:
            radio_transmitting = true;

            // get the next message from the queue
            next_event_downlink_request = radio_tx_buf_dequeue(next_event_buf_string /* OUTPUT PARAMETER */ );

            if ((   link_quality_message_enable && (lora_confirmed_messages || RADIO_TX_BUF_FORCE_CONFIRMED_EVENT__GET())) ||
               (( ! link_quality_message_enable && (lora_confirmed_messages))))
            {
                if (lora_uplink_retries < 0)
                {
                    lora_uplink_retries = -lora_uplink_retries;  // invert the value so we only do this once
                }
                // confirmed message retransmission - also at this point lora_uplink_retries is guaranteed to NEVER be 0
                // ack enabled and will retry lora_uplink_retries times
//                radio_send_string("AT+ACK=", 7);
//                radio_send_string(convert_uint8_to_ascii_string((uint8_t)lora_uplink_retries), sizeof(lora_uplink_retries));
//                radio_send_string("\r", 1);
                // set ack to 0 to disable acks
//                if((first_event_flag==1)&&(first_event_tx==1))
//                  {
                    radio_send_string("AT+ACK=1\r", 9);
                    ack_flag = 1;
//                  }
//                else
//                  {
//                    radio_send_string("AT+ACK=0\r", 9);
//                    ack_flag = 0;
//                  }
            }
            else
            {
                // set ack to 0 to disable acks
//                if((first_event_flag==1)&&(first_event_tx==1)&&(first_event_tx_cnt==0))
//                  {
                    radio_send_string("AT+ACK=1\r", 9);
                    ack_flag = 1;
//                  }
//                else
//                  {
//                    radio_send_string("AT+ACK=0\r", 9);
//                    ack_flag = 0;
//                  }
            }
            lora_state = LORA_STATE_WAIT_FOR_OK;
            lora_next_state = LORA_STATE_TRANSMIT_PAYLOAD_CONT_2;
            break;

        case LORA_STATE_TRANSMIT_PAYLOAD_CONT_2:
            if (lora_port_number != lora_port_number_new)
            {
                lora_port_number = lora_port_number_new;
                // set port
                radio_send_string("AT+AP=", 6);
                radio_send_string(convert_uint8_to_ascii_string(lora_port_number), sizeof(lora_port_number));
                radio_send_string("\r", 1);

                lora_state = LORA_STATE_WAIT_FOR_OK;
                lora_next_state = LORA_STATE_TRANSMIT_PAYLOAD_CONT_3;
            }
            else
            {
                lora_state = LORA_STATE_TRANSMIT_PAYLOAD_CONT_3;
            }
            break;

        case LORA_STATE_TRANSMIT_PAYLOAD_CONT_3:

            // only change adr if it is different
            if (lora_adr_disable_changed)
            {
                lora_adr_disable_changed = false;
                // if ADR is current set to disabled, then send ADR=0, else ADR=1
                if (lora_adr_disable)
                {
                    // adaptive data rate off
                    radio_send_string("AT+ADR=0\r", 9);
                }
                else
                {
                    // adaptive data rate on
                    radio_send_string("AT+ADR=0\r", 9);
                }
                lora_state = LORA_STATE_WAIT_FOR_OK;
                lora_next_state = LORA_STATE_TRANSMIT_PAYLOAD_CONT_4;
            }
            else
            {
                lora_state = LORA_STATE_TRANSMIT_PAYLOAD_CONT_4;
            }
            break;

        case LORA_STATE_TRANSMIT_PAYLOAD_CONT_4:
            // send beginning of tx AT command
            radio_send_string("AT+SENDB=", 9);
            MYdevice.lora.pkt_sent++;
            // send payload as part of AT command
//            radio_send_string(next_event_buf_string, 6);
            uint8_t i = 0;
            uint8_t str_cnt = 0;
//            while(next_event_buf_string[i] != 0)
//              {
//                radio_send_string(*next_event_buf_string[i], 1);
//                i++;
//              }
            for(i=0; i<28; i++)
              {
                if(next_event_buf_string[i]!=0)
                  str_cnt++;
              }
            radio_send_string(next_event_buf_string, str_cnt);
            radio_send_string("\r", 1);

            lora_state = LORA_STATE_TRANSMIT_WAIT_FOR_ACK;

            // once we send a message to lora we need to reset the governor timer (500 MS)
            reset_radio_governor(RADIO_GOVERNOR_TIMEOUT_LORA);
            break;

        case LORA_STATE_TRANSMIT_WAIT_FOR_ACK:
            // ****************************************************************************************
            // Search for the string "DOCUMENT_TRANSMIT_WAIT_FOR_ACK" below for a full description
            // of how the TRANSMIT_WAIT_FOR_ACK state works
            // ****************************************************************************************

            start_radio_response_timeout(RADIO_RESPONSE_ACK_TIMEOUT);

            lora_error_received__tx_buf_full_of_mac_cmds = false;
            lora_error_received__xmit_in_progress        = false;
            lora_error_received__ack_not_received        = false;
            lora_error_received                          = false;  // set to true if/when ERROR is received to indicate a NACK that the Event was NOT sent
            lora_ok_received                             = false;  // set to true if/when OK is received to indicate an ACK that the Event was sent
            lora_downlink_received                       = false;  // set to true to indicate that an optional DOWNLINK was received

            lora_waiting_for_ok = true;
            lora_waiting_for_error = true;
            lora_waiting_for_downlink = true;

            xmit_retry_attempt = 0;

            lora_state = LORA_STATE_TRANSMIT_WAITING_FOR_ACK;
            break;

        case LORA_STATE_TRANSMIT_WAITING_FOR_ACK:

            // if the response doesn't come and we timeout waiting,
            // then reset the module
            if(radio_response_timeout)
            {
//                TRC_RADIO_IMPORTANT("TIMEOUT");
                radio_response_has_timed_out = true;
                lora_api_internal_join__restart();
                xmit_retry_attempt = 0;
                lora_state = LORA_STATE_RESET;
//                TRC_RADIO_RESET();
            }
            else if(lora_error_received)
            {
                stop_radio_response_timeout();
                xmit_retry_attempt = 0;
                if (lora_error_received__tx_buf_full_of_mac_cmds)
                {
                    if (xmit_retry_attempt__tx_buf_full_of_mac_cmds > 0)
                    {
                        xmit_retry_attempt = xmit_retry_attempt__tx_buf_full_of_mac_cmds--;
                    }
                }
                else if (lora_error_received__xmit_in_progress)
                {
                    if ( xmit_retry_attempt__xmit_in_progress > 0)
                    {
                        xmit_retry_attempt = xmit_retry_attempt__xmit_in_progress--;
                    }
                }
                else if (lora_error_received__ack_not_received)
                {
                    if ( xmit_retry_attempt__ack_not_received > 0)
                    {
                        xmit_retry_attempt = xmit_retry_attempt__ack_not_received--;
                    }
                    MYdevice.lora.pkt_fail++;
                }
                else // never before seen ERROR
                {
                    xmit_retry_attempt = 0;
                }

                if (xmit_retry_attempt > 0)
                {
//                    TRC_LABEL_VALUE_HEX_UINT8("XMIT RETRY count", xmit_retry_attempt);
                    lora_state = LORA_STATE_TRANSMIT_START;
                    set_radio_state_delay(RADIO_STATE_DELAY_IN_TICKS_AFTER_ERROR);
                }
                else
                {
//                    TRC_RADIO_IMPORTANT("XMIT ERROR");
//                    lora_state = LORA_STATE_RESET;
                    lora_state = LORA_STATE_JOIN_START;
                    MYdevice.lora.ack_fail = 1;
                    join_attempts=0;
                    lora_api_internal_join__restart();
//                    TRC_RADIO_RESET();
                    if((MYdevice.lora.rejoin_attempt>3)&&(radio_tx_buf_has_event_to_send()))
                      {
                        clear_tx_buf();
                      }
                }
            }
            else if(lora_ok_received)
            {
                stop_radio_response_timeout();
                xmit_retry_attempt = 0;
                radio_transmitting = false;  // makes it possible to check for Debounce sooner

                if(MYdevice.lora.ping_result==3)
                  MYdevice.lora.ping_result = 2;

                // Since we received a successful ACK, then remove the message from the queue
                radio_tx_buf_received_ack_confirmation(true);  // the 'true' parameter indicates this is a real ack

//                if (lora_downlink_received)
//                {
////                    TRC_RADIO_IMPORTANT("DOWNLINK RECEIVED");
//                    // process downlink message
////                    process_downlink_message(downlink_message_bytes, lora_downlink_num_bytes, true);
//
//                    lora_downlink_received = false;
//                    lora_state = LORA_STATE_QUERY_RSSI;
//                    lora_next_next_state = LORA_STATE_CHECK_FOR_NEXT_EVENT_SEND;
//                    break;
//                }
                if(ack_flag)
                  {
                    lora_state = LORA_STATE_QUERY_RSSI;
//                    lora_state = LORA_STATE_QUERY_SNR;
                    lora_next_next_state = LORA_STATE_CHECK_FOR_NEXT_EVENT_SEND;
                    ack_flag = 0;
                  }
                else
                  lora_state = LORA_STATE_CHECK_FOR_NEXT_EVENT_SEND;
//                  lora_next_next_state = LORA_STATE_CHECK_FOR_NEXT_EVENT_SEND;
//                lora_state = LORA_STATE_CHECK_FOR_NEXT_EVENT_SEND;
            }
            break;

        case LORA_STATE_CHECK_FOR_NEXT_EVENT_SEND:
            if (radio_tx_buf_has_downlink_ack_to_send() || radio_tx_buf_has_event_to_send())
            {
                xmit_retry_attempt = 0;
                lora_state = LORA_STATE_TRANSMIT_START;
            }
            else
            {
                lora_state = LORA_STATE_GO_TO_SLEEP;
                reset_radio_governor(RADIO_GOVERNOR_TIMEOUT_LORA);
            }
            break;


        // ------------------------------------------------------------------
        // ------------------------------------------------------------------
        // QUERY_FIRMWARE_VERSION Sequence
        // ------------------------------------------------------------------
        // ------------------------------------------------------------------

        case LORA_STATE_QUERY_FIRMWARE_VERSION:
            // send ATI command
//            radio_send_string("ATI\r", 4);
            radio_send_string("ATE0\r", 5);

            // initialize a SAFE_C_STR structure to safely accumulate the results of the ATI command
//            SAFE_C_STR_INIT( & safe_c_str, temp_work_buf);
            lora_firmware_version_capture = true;

            // need to save the next state to go to after we receive the Transmit Delay
            lora_next_next_state = lora_next_state;

//            lora_state = LORA_STATE_WAIT_FOR_STRING_AND_OK;
//            lora_next_state = LORA_STATE_RECEIVE_FIRMWARE_VERSION;
            lora_state = LORA_STATE_WAIT_FOR_OK;
            if(MYdevice.production_mode == 1)
              lora_next_state = LORA_STATE_SET_WAKE_MODE;
//            lora_next_state = LORA_STATE_GO_TO_SLEEP;
            else
            lora_next_state = LORA_STATE_SET_DEFAULT_FREQUENCY_BAND;
            break;

        case LORA_STATE_RECEIVE_FIRMWARE_VERSION:
            {
                static bool version__written_to_cfg_out = false;
                //if ( ! version__written_to_cfg_out                )
                {
//                    CFG_OUT_STRING("Version", safe_c_str_get( & safe_c_str));
                    version__written_to_cfg_out = true;
                }
            }

            lora_state = lora_next_next_state;
            break;



        // ------------------------------------------------------------------
        // ------------------------------------------------------------------
        // QUERY_DATA_RATE Sequence
        // ------------------------------------------------------------------
        // ------------------------------------------------------------------

        case LORA_STATE_QUERY_DATA_RATE:
            // send read DATA_RATE (DR) command
            radio_send_string("AT+TXDR?\r", 9);

            // initialize a SAFE_C_STR structure to safely accumulate the results of the command
//            SAFE_C_STR_INIT( & safe_c_str, temp_work_buf);

            // need to save the next state to go to after we receive the query
            lora_next_next_state = lora_next_state;

            lora_state = LORA_STATE_WAIT_FOR_STRING_AND_OK;
            lora_next_state = LORA_STATE_RECEIVE_DATA_RATE;
            break;

        case LORA_STATE_RECEIVE_DATA_RATE:
            lora_data_rate_value = temp_work_buf[2];

            // restore the return state of what initiated the Query
            lora_state = lora_next_next_state;
            break;



            // ------------------------------------------------------------------
            // ------------------------------------------------------------------
            // QUERY_PAYLOAD Sequence
            // ------------------------------------------------------------------
            // ------------------------------------------------------------------

            case LORA_STATE_QUERY_PAYLOAD:
                // send read DATA_RATE (DR) command
                radio_send_string("AT+TXS?\r", 8);

                // initialize a SAFE_C_STR structure to safely accumulate the results of the command
    //            SAFE_C_STR_INIT( & safe_c_str, temp_work_buf);

                // need to save the next state to go to after we receive the query
                lora_next_next_state = lora_next_state;

                lora_state = LORA_STATE_WAIT_FOR_UINT_AND_OK;
                lora_next_state = LORA_STATE_RECEIVE_DATA_RATE;
                break;

            case LORA_STATE_RECEIVE_PAYLOAD:
                lora_data_rate_value = temp_work_buf[2];

                // restore the return state of what initiated the Query
                lora_state = lora_next_next_state;
                break;

        // ------------------------------------------------------------------
        // ------------------------------------------------------------------
        // WAIT_FOR_STRING_AND_OK Sequence
        // ------------------------------------------------------------------
        // ------------------------------------------------------------------

        case LORA_STATE_WAIT_FOR_STRING_AND_OK:
            // start the response timer and then wait
            start_radio_response_timeout(RADIO_RESPONSE_TIMEOUT_STANDARD);
            lora_waiting_for_string_and_ok = true;
            lora_string_received = false;
            lora_ok_received = false;
            lora_state = LORA_STATE_WAITING_FOR_STRING_AND_OK;

        case LORA_STATE_WAITING_FOR_STRING_AND_OK:
            // if the response doesn't come and we timeout waiting,
            // then reset the module

            if(radio_response_timeout)
            {
                radio_response_has_timed_out = true;
                stop_radio_response_timeout();
                lora_waiting_for_ok = false;
                lora_state = LORA_STATE_RESET;
//                TRC_RADIO_RESET();
//                if(MYdevice.lora.ping_result==3)
//                  MYdevice.lora.ping_result = 1;
//                if(MYdevice.lora.join_status==3)
//                  MYdevice.lora.join_status = 1;
            }
            else if(lora_string_received && lora_ok_received)
            {
                lora_string_received = false;
                lora_ok_received = false;
                lora_waiting_for_string_and_ok = false;
                stop_radio_response_timeout();
                lora_state = lora_next_state;
            }
            break;

        // ------------------------------------------------------------------
        // ------------------------------------------------------------------
        // WAIT_FOR_UINT_AND_OK Sequence
        // ------------------------------------------------------------------
        // ------------------------------------------------------------------

        case LORA_STATE_WAIT_FOR_UINT_AND_OK:
            // start the response timer and then wait
            start_radio_response_timeout(RADIO_RESPONSE_TIMEOUT_STANDARD);
            lora_waiting_for_uint_and_ok = true;
            lora_uint_received = false;
            lora_ok_received = false;

            lora_state = LORA_STATE_WAITING_FOR_UINT_AND_OK;


        case LORA_STATE_WAITING_FOR_UINT_AND_OK:
            // if the response doesn't come and we timeout waiting,
            // then reset the module
            if(radio_response_timeout)
            {
                radio_response_has_timed_out = true;
                stop_radio_response_timeout();
                lora_waiting_for_ok = false;
                lora_state = LORA_STATE_RESET;
//                TRC_RADIO_RESET();
//                if(MYdevice.lora.ping_result==3)
//                  MYdevice.lora.ping_result = 1;
//                if(MYdevice.lora.join_status==3)
//                  MYdevice.lora.join_status = 1;
            }
            else if(lora_uint_received && lora_ok_received)
            {
                lora_uint_received = false;
                lora_ok_received = false;
                lora_waiting_for_uint_and_ok = false;
                stop_radio_response_timeout();
                lora_state = lora_next_state;
            }
            break;

        // ------------------------------------------------------------------
        // ------------------------------------------------------------------
        // WAIT_FOR_OK Sequence
        // ------------------------------------------------------------------
        // ------------------------------------------------------------------

        case LORA_STATE_WAIT_FOR_OK:
            // start the response timer and then wait
            start_radio_response_timeout(RADIO_RESPONSE_TIMEOUT_STANDARD);
            // will be set to true when the OK is received
            lora_ok_received = false;
            lora_waiting_for_ok = true;
            lora_state = LORA_STATE_WAITING_FOR_OK;
            break;

        case LORA_STATE_WAITING_FOR_OK:
            // if the response doesn't come and we timeout waiting,
            // then reset the module
            if(radio_response_timeout)
            {
//                char_send((uint8_t)lora_waiting_for_rssi, 1);
                radio_response_has_timed_out = true;
                stop_radio_response_timeout();
                lora_waiting_for_ok = false;
                if(rssi_flag)
                  {
                    lora_state = LORA_STATE_GO_TO_SLEEP;
                    rssi_flag = 0;
                  }
                else
                  lora_state = LORA_STATE_RESET;
//                TRC_RADIO_RESET();
//                MYdevice.sku_tmp = MYdevice.sku;
                if(MYdevice.lora.lora_alive_flag && MYdevice.lora.lora_alive_status==3)
                  {
//                    ps_load(DEVICE_SKU_PS,&MYdevice.sku,1);  // check to see what type this device is
                    MYdevice.lora.lora_alive_flag = 0;
                    MYdevice.sku = 0xFE;
                    ps_save(DEVICE_SKU_PS,&MYdevice.sku,1);  // check to see what type this device is
                    MYdevice.lora.lora_alive_status = 1;
                  }
//                if(MYdevice.lora.ping_result==3)
//                  MYdevice.lora.ping_result = 1;
//                if(MYdevice.lora.join_status==3)
//                  MYdevice.lora.join_status = 1;
            }
            else if(lora_ok_received)
            {
                if(MYdevice.lora.lora_alive_flag && MYdevice.lora.lora_alive_status==3)
                  {
//                    ps_load(DEVICE_SKU_PS,&MYdevice.sku,1);  // check to see what type this device is
                    MYdevice.lora.lora_alive_flag = 0;
                    MYdevice.lora.lora_alive_status = 2;
                  }
                lora_ok_received = false;
                lora_waiting_for_ok = false;
                stop_radio_response_timeout();
                // after receiving OK from the last command, move to next state
                lora_state = lora_next_state;
//                if(lora_next_state==LORA_STATE_SLEEPING && is_timer_running()==false)
                if(lora_next_state==LORA_STATE_SLEEPING)
                  {
//                    char_send('A',0);
                    UARTDRV_DeInit(sl_uartdrv_usart_testUART_handle);
//                    GPIO_PinModeSet(gpioPortA, 5, gpioModeDisabled, 1);   //added for sleep current test - JP - 11242023
//                    GPIO_PinModeSet(gpioPortA, 6, gpioModeDisabled, 1);
                    GPIO_PinModeSet(gpioPortA, 5, gpioModeInput, 0);   //added for sleep current test - JP - 11242023
//                    initLetimer(0);
//                    LETIMER_Enable(LETIMER0, false);
                    if(is_timer_running()==false)
                      LETIMER_Reset(LETIMER0);

                  }
                if(rssi_flag)
                  rssi_flag = 0;
            }
//            else
//              char_send('E',0);
            break;

        // ------------------------------------------------------------------
        // ------------------------------------------------------------------
        // GO_TO_SLEEP Sequence
        // ------------------------------------------------------------------
        // ------------------------------------------------------------------

        case LORA_STATE_GO_TO_SLEEP:
//            API_PIN__RADIO_SLEEPN__SetLow();
          GPIO_PinModeSet(LORA_WAKE_PORT, LORA_WAKE_PIN, gpioModePushPull, 0);

//            TRC_RADIO_SEQUENCE("GO TO SLEEP");

             // save the joined session so we can restore when we wake up
             radio_send_string("AT&W\r", 5);
//          radio_send_string("AT+SLEEP=0\r", 11);

             lora_state = LORA_STATE_WAIT_FOR_OK;
             lora_next_state = LORA_STATE_GO_TO_SLEEP_CONT_1;
//             lora_next_state = LORA_STATE_SLEEPING;
             break;

        case LORA_STATE_GO_TO_SLEEP_CONT_1:
             // save the joined session so we can restore when we wake up
             radio_send_string("AT+SS\r", 6);

             lora_state = LORA_STATE_WAIT_FOR_OK;
             lora_next_state = LORA_STATE_GO_TO_SLEEP_CONT_2;
             break;

        case LORA_STATE_GO_TO_SLEEP_CONT_2:
            radio_send_string("AT+SLEEP=0\r", 11);
//                      radio_send_string("AT\r", 3);

            radio_transmitting = false;

            lora_state = LORA_STATE_WAIT_FOR_OK;
            lora_next_state = LORA_STATE_SLEEPING;
            break;

        case LORA_STATE_SLEEPING:
            // lora module in sleep mode, waiting for next event

            // flag to indicate the lora module is sleeping
            radio_is_sleeping = true;
            radio_transmitting = false;
            my_sys_is_ready_to_sleep = true;
//            UARTDRV_DeInit(sl_uartdrv_usart_testUART_handle);

            // lora_waiting_for_transmit_delay_to_elapse is set to true whenever the xDot radio (via AT+TXN?)
            // indicates that we must wait before trying a JOIN or a TRANSMIT.  This delay time is
            // managed by reset_radio_governor() which sets radio_ok_to_send when it is OK to attempt
            // the JOIN or TRANSMIT.
            if (lora_waiting_for_transmit_delay_to_elapse)
            {
                if (radio_ok_to_send)
                {
                    char_send('L',0);
                    lora_waiting_for_transmit_delay_to_elapse = false;
                    tx_delay_expired_flag = 1;
                    MYdevice.lora.network_delay = 0;
                    MYdevice.lora.network_delay_tm = 0;
                }
                else
                {
                    // continue sleeping until radio_ok_to_send is set by the radio_governor
//                    char_send('K',0);
                    break;
                }
            }
//            LETIMER_Reset(LETIMER0);
            // if we are not JOINED to the LoRa network
//            if (!lora_api_internal_join__is_network_joined() && !lora_join_started && radio_tx_buf_new_join_wakeup_event_enqueued())
            if (!lora_api_internal_join__is_network_joined() && !lora_join_started)
            {
                lora_join_started = true;
                if(is_timer_running()==false)
                  {
                  //intilize 100ms state machine timer and UART
                   initLetimer(1);
                   lowPowerDelay(100);
                   letmr_flag = 0;
//                   char_send('F',0);
                  }
//                MYdevice.lora.rejoin=1;
                // Then wait a very little bit (250 MS) before initiating a Join.  This will happen
                // after the short delay in the following in the 'if (...)' that follows.
                reset_radio_governor(RADIO_GOVERNOR_DELAY_BEFORE_JOIN);
            }

            if(is_timer_running()==false)
            {
                if(letmr_flag==0)
                  {
                    LETIMER_Reset(LETIMER0);
                    letmr_flag = 1;
//                    char_send('B',0);
                  }
            }
//char_send('O',0);
            // there is a new event in the buffer to transmit
//             if we do not have a valid app key, we will sit here until another reset
            if ((radio_tx_buf_has_event_to_send() && radio_ok_to_send && app_key_valid && app_eui_valid && MYdevice.lora.lora_en==1  &&  MYdevice.registration==1)||(reg_flag)||(MYdevice.lora.rejoin)|| (MYdevice.lora.ping)||MYdevice.lora.join_test||MYdevice.lora.lora_alive_flag)

//              if (radio_tx_buf_has_event_to_send() && radio_ok_to_send && app_key_valid && app_eui_valid)
            {
                // if the network is already joined, then it isn't just started.
                lora_join_started = false;
//                MYdevice.lora.rejoin = 0;
                sl_uartdrv_init_instances();
                initUSART0();
                USART_IntEnable(USART0, USART_IEN_RXDATAV);
                if(reg_flag||MYdevice.lora.join_test||MYdevice.lora.lora_alive_flag)
                  {
                    lora_state = LORA_STATE_RESET;
                    reg_flag=0;
                    //intilize 100ms state machine timer and UART
                     initLetimer(1);
                     lowPowerDelay(100);
                     letmr_flag = 0;
//                     char_send('2',0);
                  }

                else
                  {
                      lora_state = LORA_STATE_WAKE_UP;
                      if(is_timer_running()==false)
                        {
                        //intilize 100ms state machine timer and UART
                         initLetimer(1);
                         lowPowerDelay(100);
                         letmr_flag = 0;
                        }
                      char_send('3',0);
                  }

//                if(MYdevice.lora.rejoin)
//                  char_send('A',0);
//                if(MYdevice.lora.join_test)
//                  char_send('B',0);
//                if(MYdevice.lora.ping)
//                  char_send('C',0);
//                if(radio_tx_buf_has_event_to_send())
//                  char_send('D ',0);
                // A transition from Low to High on RADIO_SLEEPN wakes up the Radio from Sleeping.ever had to wipe sweaty ass cheeks?

                // This Pin is set Low in the initial state, and is set Low after it is woken up.
                // This ensures that the Radio is always primed to be woken up.
//                API_PIN__RADIO_SLEEPN__SetHigh();
                GPIO_PinModeSet(LORA_WAKE_PORT, LORA_WAKE_PIN, gpioModePushPull, 1);
                send_string("wake p H");

                // give the wakeup a little time
                set_radio_state_delay(20);


//                else


                radio_is_sleeping = false;
                my_sys_is_ready_to_sleep = false;
            }

            break;

        // ------------------------------------------------------------------
        // ------------------------------------------------------------------
        // WAKE_UP Sequence
        // ------------------------------------------------------------------
        // ------------------------------------------------------------------

        case LORA_STATE_WAKE_UP:
//            TRC_RADIO_SEQUENCE("WAKE UP");

            // This prepares waking up the radio module for the next time.
//            API_PIN__RADIO_SLEEPN__SetLow();
            GPIO_PinModeSet(LORA_WAKE_PORT, LORA_WAKE_PIN, gpioModePushPull, 0);
send_string("wake");
            // Reseting the RADIO_SLEEPN might need some time before we can send commands
            set_radio_state_delay(1);

            lora_state = LORA_STATE_WAKE_UP_CONT_1;
            break;

        case LORA_STATE_WAKE_UP_CONT_1:
            radio_send_string("AT+RS\r", 6);

            lora_state = LORA_STATE_WAIT_FOR_OK;
            lora_next_state = LORA_STATE_WAKE_UP_CONT_2;
            break;

        case LORA_STATE_WAKE_UP_CONT_2:
            set_radio_state_delay(1);
            // check to see if we are joined before sending
            if( ! lora_api_internal_join__is_network_joined()||MYdevice.lora.rejoin==1)
            {
                // we have never joined so try to join first
                lora_state = LORA_STATE_JOIN_START;
                send_string("wake join");
            }
            else if(MYdevice.lora.ping)
              {
                MYdevice.lora.ping_result=3;
                MYdevice.lora.ping=0;
                lora_state = LORA_STATE_PING;
                ping_cnt = 0;
              }
            else
            {
                lora_state = LORA_STATE_TRANSMIT_START;
            }
            break;

        // ------------------------------------------------------------------
        // ------------------------------------------------------------------
        // QUERY RSSI and SNR Sequence
        // ------------------------------------------------------------------
        // ------------------------------------------------------------------
        case LORA_STATE_QUERY_RSSI:
            radio_send_string("AT+RSSI?\r", 9);

            // start the response timer and then wait
            start_radio_response_timeout(RADIO_RESPONSE_TIMEOUT_STANDARD);

            lora_waiting_for_rssi = true;
            lora_rssi_received = false;
            rssi_flag = 1;
            lora_state = LORA_STATE_WAITING_FOR_RSSI;
            break;

        case LORA_STATE_WAITING_FOR_RSSI:
            // if the response doesn't come and we timeout waiting, then reset the module
            if( radio_response_timeout)
            {
                radio_response_has_timed_out = true;
                stop_radio_response_timeout();
                lora_state = LORA_STATE_RESET;
//                lora_state = LORA_STATE_GO_TO_SLEEP;
                reset_radio_governor(RADIO_GOVERNOR_TIMEOUT_LORA);
//                TRC_RADIO_RESET();
//                char_send(0x10, 1);
            }
            else if(lora_rssi_received)
            {
                lora_rssi_received = false;
                stop_radio_response_timeout();
                // after receiving OK from the last command, move to next state
//                sendrssi();
                lora_state = LORA_STATE_WAIT_FOR_OK;
                lora_next_state = LORA_STATE_QUERY_SNR;
//                lora_next_state = lora_next_next_state;
//                rssi_flag = 0;
            }
//            char_send(0x20, 1);
            break;

        case LORA_STATE_QUERY_SNR:
            radio_send_string("AT+SNR?\r", 8);

            // start the response timer and then wait
            start_radio_response_timeout(RADIO_RESPONSE_TIMEOUT_STANDARD);

            lora_waiting_for_snr = true;
            lora_snr_received = false;
            lora_state = LORA_STATE_WAITING_FOR_SNR;
            break;

        case LORA_STATE_WAITING_FOR_SNR:
            // if the response doesn't come and we timeout waiting, then reset the module
            if( radio_response_timeout)
            {
                radio_response_has_timed_out = true;
                stop_radio_response_timeout();
                lora_state = LORA_STATE_RESET;
//                TRC_RADIO_RESET();
            }
            else if(lora_snr_received)
            {
                lora_snr_received = false;
                stop_radio_response_timeout();

                if (link_quality_message_enable)
                {
                    link_quality_transmit_event();
                }
                // after receiving OK from the last command, move to next state
//                sendsnr();
                MYdevice.lora.join_status = 1;
                lora_state = LORA_STATE_WAIT_FOR_OK;
                lora_next_state = lora_next_next_state;
            }
            break;
        case LORA_STATE_PING:
            radio_send_string("AT+PING\r", 8);

            // start the response timer and then wait
            start_radio_response_timeout(RADIO_RESPONSE_TIMEOUT_STANDARD);

            lora_waiting_for_ping = true;
            lora_ping_received = false;
//            rssi_flag = 1;
            lora_state = LORA_STATE_WAITING_FOR_PING;
            break;

        case LORA_STATE_WAITING_FOR_PING:
            // if the response doesn't come and we timeout waiting, then reset the module
            if( radio_response_timeout)
            {
                radio_response_has_timed_out = true;
                stop_radio_response_timeout();
                lora_state = LORA_STATE_RESET;
                reset_radio_governor(RADIO_GOVERNOR_TIMEOUT_LORA);
                MYdevice.lora.ping_result = 1;
//                TRC_RADIO_RESET();
            }
            else if(lora_ping_received)
            {
                lora_ping_received = false;
                stop_radio_response_timeout();
                // after receiving OK from the last command, move to next state
//                sendrssi();
                lora_state = LORA_STATE_WAIT_FOR_OK;
                lora_next_state = LORA_STATE_GO_TO_SLEEP;
                MYdevice.lora.ping_result = 2;
                MYdevice.lora.join_status = 1;

            }
            break;

        // ------------------------------------------------------------------
        // ------------------------------------------------------------------
        // QUERY_TRANSMIT_DELAY and WAIT Sequence
        // ------------------------------------------------------------------
        // ------------------------------------------------------------------

        case LORA_STATE_QUERY_TRANSMIT_DELAY:
            // send query for Transmit in MilliSeconds
            radio_send_string("AT+TXN?\r", 8);

            // need to save the next state to go to after we receive the Transmit Delay
            // now set in previous state
            //lora_next_next_state = lora_next_state;
            waiting_for_tx_delay = 1;

            lora_state = LORA_STATE_WAIT_FOR_UINT_AND_OK;
            lora_next_state = LORA_STATE_RECEIVE_TRANSMIT_DELAY;
            break;

        case LORA_STATE_RECEIVE_TRANSMIT_DELAY:
            if (lora_radio_received_uint != 0)
            {
                uint32_t lora_transmit_delay_in_multiples_of_250ms =
                                    (((lora_radio_received_uint + 249) * 4) / 1000) + 1;
                uint8_t delay_in_sec;
                MYdevice.lora.network_delay_tm = lora_transmit_delay_in_multiples_of_250ms>>2;

//                TRC_RADIO_IMPORTANT_INFO("TRANSMIT DELAY (secs)", (((uint16_t)lora_transmit_delay_in_multiples_of_250ms) + 3) / 4);
                reset_radio_governor(lora_transmit_delay_in_multiples_of_250ms);
//                MYdevice.lora.join_delay_flag=1;
//                MYdevice.lora.join_delay = delay_in_sec; //try again in 5min
//                char_send(delay_in_sec, 1);
                lora_waiting_for_transmit_delay_to_elapse = true;
                MYdevice.lora.network_delay = 1;
                if(lora_next_next_state == LORA_STATE_JOIN_ATTEMPT)
                  MYdevice.lora.join_begin=1;
                // wait and then query again
//                lora_state = LORA_STATE_GO_TO_SLEEP;
                lora_state = LORA_STATE_SET_WAKE_DELAY_INTERVAL;
//                char_send('I',0);
            }
            else
            {
                // done waiting so we can continue
//                char_send('J',0);
                // restore the return state of what initiated the QUERY_TRANSMIT_DELAY: AT+JOIN or AT+SENDB
                lora_state = LORA_STATE_SET_WAKE_DELAY_INTERVAL;
                lora_waiting_for_transmit_delay_to_elapse = false;
//                lora_state = lora_next_next_state;
            }
            waiting_for_tx_delay = 0;
            break;

        // ------------------------------------------------------------------
        // ------------------------------------------------------------------

        default:
            lora_state = LORA_STATE_RESET;
//            TRC_RADIO_RESET();
            break;
    }
    if (lora_state == LORA_STATE_SLEEPING)
    {
        return false;   // returning false allows another state_machine to run
    }
    else
    {
        return true;    // returning true means the state_machine is still running and cannot be pre-empted
    }
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// process received message from the lora radio
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
void process_lora_message(uint8_t message_length)
{
    uint8_t i;

//    TRC_RADIO_RECEIVE(radio_uart_read_buffer, message_length);

    //--------------------------------------------------------------------------------------------
    // wait for string and ok
    if (lora_waiting_for_string_and_ok)
    {
        if ( ! lora_string_received)
        {
            // This flag is set in the special case when the ATI (request version info) is sent to the
            // radio.  For this command, the results are returned in multiple separate strings.  However,
            // these results need to be accumated into a single string (each sub-result separated by ',').
            // The resulting version information is then sent over the serial port to the test (verify) App
            // to display, test, and validate the version of the Radio.
            if (lora_firmware_version_capture)
            {
                // When the ATI, numerous strings of version info are returned from the Radio.  The
                // only way to know that all the strings have been sent is when it also returns an "OK".
                if (strncmp((const char *)radio_uart_read_buffer, "OK", 2) == 0)
                {
                    lora_firmware_version_capture = false;
                    lora_string_received = true;
                    lora_ok_received = true;
                }
                else
                {
                    safe_c_str_append( & safe_c_str, (const char *)radio_uart_read_buffer, message_length);
                }
            }
            else
            {
                safe_c_str_append( & safe_c_str, (const char *)radio_uart_read_buffer, message_length);
                lora_string_received = true;
            }
        }
        else if ( ! lora_ok_received)
        {
            if (strncmp((const char *)radio_uart_read_buffer, "OK", 2) == 0)
            {
                lora_ok_received = true;
            }
        }
    }
    //--------------------------------------------------------------------------------------------
    // wait for uint and ok
    else if (lora_waiting_for_uint_and_ok)
    {
        if ( ! lora_uint_received)
        {
            // test for common special case of receiving a '0'
            if ((message_length == 1) && (radio_uart_read_buffer[0] == '0'))
            {
                lora_radio_received_uint = 0;
            }
            else
            {
//                if(! lora_waiting_for_gps)
//                  {
                    if(waiting_for_tx_delay)
                      {
                        waiting_for_tx_delay = 0;
                        tx_delay_buf_len = (message_length-3);
                        for(uint8_t i = 0; i<message_length; i++)
                          {
                            tx_delay_buf[i] = radio_uart_read_buffer[i];
                          }
                      }
                lora_radio_received_uint =
                    convert_array_of_ascii_decimal_chars_to_32_bit_integer((uint8_t *) &radio_uart_read_buffer[0], message_length);
            }
            lora_uint_received = true;
        }
        else if ( ! lora_ok_received)
        {
            if (strncmp((const char *)radio_uart_read_buffer, "OK", 2) == 0)
            {
                lora_ok_received = true;
            }
        }
    }
    //--------------------------------------------------------------------------------------------
    else if (lora_waiting_for_rssi)
    {
        lora_waiting_for_rssi = false;

        lora_last_downlink_rssi = convert_small_floating_point_ascii_array_to_8_bit_twos_complement_int((uint8_t *) &radio_uart_read_buffer[0]);
        MYdevice.lora.lora_rssi_last = lora_last_downlink_rssi;

        if(MYdevice.lora.lora_rssi_last<MYdevice.lora.lora_rssi_min)
          MYdevice.lora.lora_rssi_min=MYdevice.lora.lora_rssi_last;

        if(MYdevice.lora.lora_rssi_last>MYdevice.lora.lora_rssi_max)
          MYdevice.lora.lora_rssi_max=MYdevice.lora.lora_rssi_last;

        lora_rssi_received = true;
    }
    //--------------------------------------------------------------------------------------------
    else if (lora_waiting_for_snr)
    {
        lora_waiting_for_snr = false;

        lora_last_downlink_snr = convert_small_floating_point_ascii_array_to_8_bit_twos_complement_int((uint8_t *) &radio_uart_read_buffer[0]);
        MYdevice.lora.lora_snr_last = lora_last_downlink_snr;

        if(MYdevice.lora.lora_snr_last&0x80)
          {
            if(MYdevice.lora.lora_snr_min<129)
              MYdevice.lora.lora_snr_min=MYdevice.lora.lora_snr_last;
            else
              {
                if(MYdevice.lora.lora_snr_last<MYdevice.lora.lora_snr_min)
                  MYdevice.lora.lora_snr_min=MYdevice.lora.lora_snr_last;
              }
          }
        else
          {
            if(MYdevice.lora.lora_snr_last<MYdevice.lora.lora_snr_min)
              MYdevice.lora.lora_snr_min=MYdevice.lora.lora_snr_last;
          }
        if(MYdevice.lora.lora_snr_last>MYdevice.lora.lora_snr_max)
          MYdevice.lora.lora_snr_max=MYdevice.lora.lora_snr_last;

        lora_snr_received = true;
    }
    else if (lora_waiting_for_ping)
    {
        lora_waiting_for_ping = false;
        uint8_t tmp;

        for(tmp=0; tmp<10; tmp++)
          {
            if(radio_uart_read_buffer[tmp]==',')
              break;
          }

        lora_last_ping_rssi = convert_small_floating_point_ascii_array_to_8_bit_twos_complement_int((uint8_t *) &radio_uart_read_buffer[0]);
        lora_last_ping_snr = convert_small_floating_point_ascii_array_to_8_bit_twos_complement_int((uint8_t *) &radio_uart_read_buffer[tmp+1]);
        MYdevice.lora.lora_snr_last = lora_last_ping_snr;
        MYdevice.lora.lora_rssi_last = lora_last_ping_rssi;

        if(MYdevice.lora.lora_snr_last&0x80)
          {
            if(MYdevice.lora.lora_snr_min<129)
              MYdevice.lora.lora_snr_min=MYdevice.lora.lora_snr_last;
            else
              {
                if(MYdevice.lora.lora_snr_last<MYdevice.lora.lora_snr_min)
                  MYdevice.lora.lora_snr_min=MYdevice.lora.lora_snr_last;
              }
          }
        else
          {
            if(MYdevice.lora.lora_snr_last<MYdevice.lora.lora_snr_min)
              MYdevice.lora.lora_snr_min=MYdevice.lora.lora_snr_last;
          }
        if(MYdevice.lora.lora_snr_last>MYdevice.lora.lora_snr_max)
          MYdevice.lora.lora_snr_max=MYdevice.lora.lora_snr_last;

        if(MYdevice.lora.lora_rssi_last<MYdevice.lora.lora_rssi_min)
          MYdevice.lora.lora_rssi_min=MYdevice.lora.lora_rssi_last;

        if(MYdevice.lora.lora_rssi_last>MYdevice.lora.lora_rssi_max)
          MYdevice.lora.lora_rssi_max=MYdevice.lora.lora_rssi_last;

        lora_ping_received = true;
    }
    //--------------------------------------------------------------------------------------------
    // wait for OK or ERROR, optionally preceeded by DOWNLINK
//    else if (lora_waiting_for_downlink && (lora_waiting_for_ok || lora_waiting_for_error))
    else if ((lora_waiting_for_ok || lora_waiting_for_error))
    {
        if ( ! lora_error_received__tx_buf_full_of_mac_cmds  &&
             ! lora_error_received__xmit_in_progress         &&
             ! lora_error_received__ack_not_received)
        {
            lora_error_received__tx_buf_full_of_mac_cmds = (strncmp((const char *)radio_uart_read_buffer, "Tx buffer filled by MAC Commands, send data again", 49) == 0);
            lora_error_received__xmit_in_progress        = (strncmp((const char *)radio_uart_read_buffer, "Transmit in progress", 20) == 0);
            lora_error_received__ack_not_received        = (strncmp((const char *)radio_uart_read_buffer, "ACK not received", 16) == 0);
        }

        lora_error_received                          = (strncmp((const char *)radio_uart_read_buffer, "ERROR", 5) == 0);
        lora_ok_received                             = (strncmp((const char *)radio_uart_read_buffer, "OK",    2) == 0);
        if (lora_ok_received || lora_error_received)
        {
            lora_waiting_for_ok = false;
            lora_waiting_for_error = false;
            lora_waiting_for_downlink = false;
        }
//        else if(!lora_downlink_received)
//        {
//            // we need to save the actually number of bytes received (should be 8, but might not)
//            lora_downlink_num_bytes = message_length / 2;
//
//            // if the number of downlink bytes received is 8
//            if (lora_downlink_num_bytes == DOWNLINK_MESSAGE_SIZE)
//            {
//                // will be set to true since the downlink is received
//                lora_downlink_received = true;
//
//                // then copying them so the state machine can process them
//                for (i = 0; i < DOWNLINK_MESSAGE_SIZE; i++)
//                    downlink_message_bytes[i] = ascii_byte_to_hex( (uint8_t *)(& radio_uart_read_buffer[i * 2]));
//            }
//        }
    }
    //--------------------------------------------------------------------------------------------
    // if there was a match with OK then set the OK received flag
    else if(lora_waiting_for_ok && (strncmp((const char *)radio_uart_read_buffer, "OK", 2) == 0))
    {
        lora_ok_received = true;
    }
    //--------------------------------------------------------------------------------------------
    // if we are waiting for the response from the join network command
    else if(lora_waiting_for_join)
    {
        // if JOIN SUCCESS
        if(strncmp((const char *)radio_uart_read_buffer, "OK", 2) == 0)
        {
            lora_api_internal_join__set_state(LORA_API_INTERNAL_JOIN__STATE__JOINED);
            lora_join_received = true;
//            char_send('P', 0);
        }
        // if JOIN ERROR
        else if(strncmp((const char *)radio_uart_read_buffer, "ERROR", 5) == 0)
        {
            // received noack after retries, send failed
            lora_join_received = true;
//            lora_join_received = false;
//            char_send('F', 0);
        }
    }
}

//---------------------------------------------------------------------------
// verifies that app key stored in eeprom is valid
bool validate_app_key(void)
{
    uint8_t i;
    bool key_valid = false;
    uint8_t key_byte;
    uint8_t ascii_byte[2];
    uint8_t lora_app_key_buf[16] = {0};

    ps_load(EEPROM_LORA_APP_KEY_OFFSET, lora_app_key_buf, EEPROM_LORA_APP_KEY_LENGTH);
    for(i = 0; i < EEPROM_LORA_APP_KEY_LENGTH; i++)
    {
        // read the next byte of the key from eeprom
//        key_byte = lora_app_key_buf[i];

        // if any bytes are not 0xff, then it is not default eeprom values and thus a valid key
//        if(key_byte != 0xff)
        if(lora_app_key_buf[i] != 0xFF)
            key_valid = true;

        // convert byte to ascii for key string
        hex_to_ascii_byte(lora_app_key_buf[i], ascii_byte);
        lora_app_key_string[(uint8_t)(i * 2)] = ascii_byte[0];
        lora_app_key_string[(uint8_t)((i * 2) + 1)] = ascii_byte[1];
    }
    lora_app_key_string[EEPROM_LORA_APP_KEY_LENGTH * 2] = '\0'; // turn 32 bytes of ASCII into a 33 byte C-string by appending '\0' char

    // all bytes were 0xff so not a valid key
    return key_valid;
}

//---------------------------------------------------------------------------
// verifies that app key stored in eeprom is valid
bool validate_app_eui(void)
{
    uint8_t i;
    bool key_valid = false;
    uint8_t key_byte;
    uint8_t ascii_byte[2];
    uint8_t lora_app_eui_buf[8] = {0};

    ps_load(EEPROM_LORA_APP_EUI_OFFSET, lora_app_eui_buf, EEPROM_LORA_APP_EUI_LENGTH);
    for(i = 0; i < EEPROM_LORA_APP_EUI_LENGTH; i++)
    {
        // read the next byte of the key from eeprom
//        key_byte = lora_app_eui_buf[i];

        // if any bytes are not 0xff, then it is not default eeprom values and thus a valid key
//        if(key_byte != 0xff)
        if(lora_app_eui_buf[i] != 0xFF)
            key_valid = true;

        // convert byte to ascii for key string
        hex_to_ascii_byte(lora_app_eui_buf[i], ascii_byte);
        lora_app_eui_string[(uint8_t)(i*2)] = ascii_byte[0];
        lora_app_eui_string[(uint8_t)((i*2) + 1)] = ascii_byte[1];

    }
    lora_app_eui_string[EEPROM_LORA_APP_EUI_LENGTH * 2] = '\0'; // turn 32 bytes of ASCII into a 33 byte C-string by appending '\0' char

    // all bytes were 0xff so not a valid key
    return key_valid;
}

int8_t convert_small_floating_point_ascii_array_to_8_bit_twos_complement_int(uint8_t *p)
{
    int32_t value = 0;
    bool negative = false;      // initial assumtion
    bool decimal_found = false; // initial assumtion

    if (*p == '-')
    {
        negative = true;
        p++;
    }

    for (int i = 0; i < 5; i++)
    {
        uint8_t data = *p++;

        if ((data >= '0') && (data <= '9'))
        {
            data -= '0';
            if (decimal_found)
            {
                value = value + (data + 5) / 10; // round away from 0
                break;
            }
            else
            {
                value = (value * 10) + data;
            }
        }
        else if (data == '.')
        {
            decimal_found = true;
        }
        else
        {
            break;
        }
    }
    if (negative)
    {
        value = -(SL_MIN(value, 128));
    }
    else
    {
        value = SL_MIN(value, 127);
    }
    return (int8_t)value;
}

// convert a value coded as an ascii string to raw number
uint8_t ascii_byte_to_hex(uint8_t *asciival)
{
    return (uint8_t)((ascii_nibble_to_hex(asciival[(uint8_t)0]) << 4) + ascii_nibble_to_hex(asciival[(uint8_t)1]));
}

// convert an ascii coded nibble to raw hex value
uint8_t ascii_nibble_to_hex(uint8_t asciival)
{

    if (is_ascii_coded_hex_number(asciival))
        return (uint8_t)(asciival - 0x30);
    else if (is_ascii_coded_hex_uppercase(asciival))
        return (uint8_t)(asciival - 0x37);
    else if (is_ascii_coded_hex_lowercase(asciival))
        return (uint8_t)(asciival - 0x57);
    else
        return (uint8_t)asciival;
}

// return true if it is ascii 0-9
bool is_ascii_coded_hex_number(uint8_t ascii_hex_char)
{
    // if an ascii number, lower case, or upper case
    if ((ascii_hex_char > 0x2f) && (ascii_hex_char < 0x3a))
        return true;
    else
        return false;
}

// return true if it is ascii A-Z
bool is_ascii_coded_hex_uppercase(uint8_t ascii_hex_char)
{
    // if an ascii number, lower case, or upper case
    if ((ascii_hex_char > 0x40) && (ascii_hex_char < 0x47))
        return true;
    else
        return false;
}

// return true if it is ascii a-z
bool is_ascii_coded_hex_lowercase(uint8_t ascii_hex_char)
{
    // if an ascii number, lower case, or upper case
    if ((ascii_hex_char > 0x60) && (ascii_hex_char < 0x67))
        return true;
    else
        return false;
}

//---------------------------------------------------------------------------------
// Convert an array (of array_size) of ascii characters into an int32_t.
// If any characters other than '0' through '9' are in the array, then -1 is returned.
// Thus, the '-' is an invalid character and thus -1 is returned.
// The caller MUST ensure that value from the array of characters can fit in an int32_t
int32_t convert_array_of_ascii_decimal_chars_to_32_bit_integer(uint8_t *array_ascii, int8_t array_size)
{
    int32_t sum = 0;

    for (uint8_t i = 0; i < array_size; i++)
    {
        uint8_t byteVal = array_ascii[i];
        if (!is_ascii_coded_hex_number(byteVal))
        {
            return -1;
        }
        sum += (byteVal - 0x30);
        if (i < array_size - 1)
        {
            sum *= 10;
        }
    }
    return sum;
}


void lora_payload(void)
{
 uint8_t i;
 uint16_t join_pass;
 uint16_t delay_time;
 uint32_t hb_time;
 uint16_t man_join_delay_time;
//char_send('E',0);
 man_join_delay_time = MYdevice.lora.manual_join_delay_tm*2;

 if(lora_hb_cnt<MYdevice.lora.heartbeat)
   hb_time = (MYdevice.lora.heartbeat-lora_hb_cnt)/60;
 else
   hb_time = MYdevice.lora.heartbeat/60;

 if(MYdevice.lora.join_delay_flag)
   delay_time = MYdevice.lora.join_delay-MYdevice.lora.join_delay_counter;
 else
   delay_time=0;

 join_pass=MYdevice.lora.join_attempts-MYdevice.lora.join_failed;
if(MYdevice.lora.join_status != 2)
  {
    if(lora_api_internal_join__is_network_joined())
     MYdevice.lora.join_status = 1;
    else
     MYdevice.lora.join_status = 0;
  }
 i=0;
 lora_dev_settings[i++] = 0x82; //start byte
 lora_dev_settings[i++] = 0x6B; //length
 lora_dev_settings[i++] = 0x1D; //number of IDs
 lora_dev_settings[i++] = 0x01; //DevEUI
 lora_dev_settings[i++] = 0x08;
 lora_dev_settings[i++] = MYdevice.lora.devEUI[0];
 lora_dev_settings[i++] = MYdevice.lora.devEUI[1];
 lora_dev_settings[i++] = MYdevice.lora.devEUI[2];
 lora_dev_settings[i++] = MYdevice.lora.devEUI[3];
 lora_dev_settings[i++] = MYdevice.lora.devEUI[4];
 lora_dev_settings[i++] = MYdevice.lora.devEUI[5];
 lora_dev_settings[i++] = MYdevice.lora.devEUI[6];
 lora_dev_settings[i++] = MYdevice.lora.devEUI[7];
 lora_dev_settings[i++] = 0x02; //LoRaWAN Region
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.region;
 lora_dev_settings[i++] = 0x03; //LoRaWAN Payload
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.payload;
 lora_dev_settings[i++] = 0x04; //LoRaWAN Retry
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.retry;
 lora_dev_settings[i++] = 0x05; //LoRaWAN Acknowledge
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.ack;
 lora_dev_settings[i++] = 0x06; //LoRaWAN Transmission Frequency
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.tx_interval;
 lora_dev_settings[i++] = 0x07; //LoRaWAN Enable
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.lora_en;
 lora_dev_settings[i++] = 0x08; //LoRaWAN RSSI Min
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.lora_rssi_min+128;
 lora_dev_settings[i++] = 0x09; //LoRaWAN RSSI Max
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.lora_rssi_max+128;
 lora_dev_settings[i++] = 0x0A; //LoRaWAN Data Rate
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.data_rate;
 lora_dev_settings[i++] = 0x0B; //LoRaWAN Power Level
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.tx_pwr;
 lora_dev_settings[i++] = 0x0C; //LoRaWAN Join Status
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.join_status;
 lora_dev_settings[i++] = 0x0D; //LoRaWAN Sub Channel Plan
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.sub_channel;
 lora_dev_settings[i++] = 0x0E; //LoRaWAN SNR
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.lora_snr_last+128;
// lora_dev_settings[i++] = 129;    //sending 129 results in snr = +1dbM
 lora_dev_settings[i++] = 0x0F; //LoRaWAN Join Attempts
 lora_dev_settings[i++] = 0x02;
 lora_dev_settings[i++] = (MYdevice.lora.join_attempts&0xFF00)>>8;
 lora_dev_settings[i++] = (MYdevice.lora.join_attempts&0x00FF);
 lora_dev_settings[i++] = 0x10; //LoRaWAN Total successful joins
 lora_dev_settings[i++] = 0x02;
 lora_dev_settings[i++] = (join_pass)>>8;
 lora_dev_settings[i++] = (join_pass&0x00FF);
 lora_dev_settings[i++] = 0x11; //LoRaWAN Total failed joins
 lora_dev_settings[i++] = 0x02;
 lora_dev_settings[i++] = (MYdevice.lora.join_failed&0xFF00)>>8;
 lora_dev_settings[i++] = (MYdevice.lora.join_failed&0x00FF);
 lora_dev_settings[i++] = 0x12; //LoRaWAN Total payloads sent
 lora_dev_settings[i++] = 0x02;
 lora_dev_settings[i++] = (MYdevice.lora.pkt_sent&0xFF00)>>8;
 lora_dev_settings[i++] = (MYdevice.lora.pkt_sent&0x00FF);
 lora_dev_settings[i++] = 0x13; //LoRaWAN Total payloads failed
 lora_dev_settings[i++] = 0x02;
 lora_dev_settings[i++] = (MYdevice.lora.pkt_fail&0xFF00)>>8;
 lora_dev_settings[i++] = (MYdevice.lora.pkt_fail&0x00FF);
 lora_dev_settings[i++] = 0x14; //LoRaWAN SNR Min
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.lora_snr_min+128;
 lora_dev_settings[i++] = 0x15; //LoRaWAN SNR Max
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.lora_snr_max+128;
 lora_dev_settings[i++] = 0x16; //LoRaWAN RSSI
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = (MYdevice.lora.lora_rssi_last)+128;
// lora_dev_settings[i++] = 1;    //sending will results in -127dBm
 lora_dev_settings[i++] = 0x17; //LoRaWAN PING success
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.ping_result;
 lora_dev_settings[i++] = 0x18; //Battery
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = DeviceBatt.batterylvl;
 lora_dev_settings[i++] = 0x19; //Network Imposed Delay
 lora_dev_settings[i++] = 0x02;
 lora_dev_settings[i++] = (MYdevice.lora.network_delay_tm&0xFF00)>>8;
 lora_dev_settings[i++] = (MYdevice.lora.network_delay_tm&0x00FF);
 lora_dev_settings[i++] = 0x1A; //Network Imposed Delay
 lora_dev_settings[i++] = 0x02;
 lora_dev_settings[i++] = (delay_time&0xFF00)>>8;
 lora_dev_settings[i++] = (delay_time&0x00FF);
 lora_dev_settings[i++] = 0x1B; //Manual Join Delay Time
 lora_dev_settings[i++] = 0x02;
 lora_dev_settings[i++] = (man_join_delay_time&0xFF00)>>8;
 lora_dev_settings[i++] = (man_join_delay_time&0x00FF);
 lora_dev_settings[i++] = 0x1C; //Next Heartbeat
 lora_dev_settings[i++] = 0x02;
 lora_dev_settings[i++] = (hb_time&0x0000FF00)>>8;
 lora_dev_settings[i++] = (hb_time&0x000000FF);
 lora_dev_settings[i++] = 0x1D; //LoRa Chip Alive Status
 lora_dev_settings[i++] = 0x01;
 lora_dev_settings[i++] = MYdevice.lora.lora_alive_status;
 lora_dev_settings[i] = 0xAF; //stop byte


}

void unreg_lora(void)
{
  lora_hb_cnt=0;
//  lora_api_internal_join__restart();
  lora_api_internal_join__set_state(LORA_API_INTERNAL_JOIN__STATE__NOT_JOINED);
  MYdevice.lora.lora_en = 0;
  MYdevice.lora.join_status = 0;
  MYdevice.lora.join_attempts = 0;
  MYdevice.lora.join_failed = 0;
  MYdevice.lora.join_delay_counter = 0;
  MYdevice.lora.join_delay_flag=0;
  MYdevice.lora.join_delay=0;
  MYdevice.lora.rejoin_attempt=0;
  MYdevice.lora.manual_join_delay_tm = 0;

//  MYdevice.lora.network_delay = 0;
//  MYdevice.lora.network_delay_tm = 0;

  MYdevice.lora.pkt_sent = 0;
  MYdevice.lora.pkt_fail = 0;

  MYdevice.lora.lora_snr_max=0;
  MYdevice.lora.lora_snr_min=20;
  MYdevice.lora.lora_rssi_max=128;
  MYdevice.lora.lora_rssi_min=0xFF;
  MYdevice.lora.lora_snr_last = 0;
  MYdevice.lora.lora_rssi_last = 0xF0;
//  char_send('B',0);
}


 /*
 ****************************************************************************************
 ****************************************************************************************

    **** DOCUMENT_TRANSMIT_WAIT_FOR_ACK

    ************************   TRANSMIT_WAIT_FOR_ACK Design   ***************************

    WHY IS THIS DOCUMENTATION HERE - It took 2 years to slowly and painfully evolve to this
    design, 10 minutes of reading can help us remember why.  Hopefully, only minor changes
    will be needed in the future.

    When xDot attempts to transmit a payload, it is either successful or unsuccessful.
    Unfortunately, it has been difficult to understand all different causes of errors.

    xDot does not enumerate all the different transmission ERRORs in its documentation.
    As new ERROR's are discovered, the firmware must change.  As the firmware becomes more
    complicated, its evolution sometimes results in new bugs that aren't discovered until
    much later.  This sometimes results in rapid churn of bugs introduced because of its
    complexity.

    Today, 7/24/20, in Firmware Version 2.2.2, the following is an attempt at recording a
    new "grand" design that solves all known problems.  It is complicated because it
    necessarily encompasses are requirements and considerations.  Yet, the implementation
    is ultimately simple.  The real complexity is the "why" of the design and the various
    parameters (#defines) that satisfies all requirements.

    The following describes the issues affecting the TRANSMIT_WAIT_FOR_ACK design:

    1)  Multiple LoRaWAN Zones (specifically US915 and EU868).  Each of these zones cause
        interesting side affects with important and differing timing aspects.

        For US915, it is not necessary to delay before sending an Event.

        For EU868, this firmware must query the xDot module using AT+TXN? to return the
        time required to delay (we use our LORA_STATE_GO_TO_SLEEP) before we can proceed
        with transmission.  Typical times are anywhere between 0 seconds to 140 seconds
        for Transmission of Events.  (JOIN's also do this and can get potentially last
        much longer).

        For either US915 or EU868, even after waiting the specified amount of time from
        AT+TXN?, we can still get transmission ERRORs with an "ascii reason code"
        that indicates more details that are necessary for recovery.

        This results in an internal wait loop in the firmware that is influenced by forces
        outside its control (the xDot inner workings, the LoRaWAN network, the network
        server, etc.).  To prevent a sensor from hanging (requiring the battery to
        be reinserted to recover), it is necessary to protect this retry loop by a
        maximum retry count before eventually attempting to Re-JOIN the network.

    2)  Potentially affecting the sensor firmware design, each different network server model
        has unique operating characteristics that affect event timing and/or MAC command
        sequences.  This is unfortunate because a new network server can be released that
        causes a specific sensor firmware version to no longer work the same way.

        Up until now, our sensors (v1.2, v1.3, v1.4, v2.0.1) worked for a variety of network
        servers.  However, recently the ChirpStack network server has become very popular
        and we have begun supporting it.  While it seems to work nicely with our murata
        based sensors, but is currently causing the problems with our xDot based sensors.

        While our firmware (2.2.2) and xDot sensors works with TTN, unfortunately, with
        ChirpStack the xDot returns additional ERROR's with "ascii reason codes" that not
        been seen before.  This causes a new class of ERROR that the firmware is not able
        to recover from.  Fortunately, it is possible to enhance the firmware which is the
        focus of these new changes.

        Specifically, when the firmware attempts to send a confirmed Event to a ChirpStack
        network server, the xDot returns an ERROR with either of the "ascii reason codes":

            *)  "Tx buffer filled by MAC Commands, send data again"
            *)  "Transmit in progress"
            *)  "ACK not received"

        Now we have received "ACK not received" before with other xDot/network servers
        environments, however, the first 2 "ascii reason codes" are new for the firmware
        to deal with.  Honestly, I don't really care whether xDot or ChirpStack is at
        fault.  Perhaps, this is something our firmware should naturally have been
        handling all along.  But to properly handle this new situation, it is necessary
        to enhance the firmware.

        To handle the first ERROR "Tx buffer filled by MAC Commands, send data again",
        after parsing the ascii reason code, the recovery seems reasonable.  However, to
        protect against hanging the sensor, this error should be retried a Finite
        number of times.  The following scenario illustrates:

            A) JOIN Successful          Immediately after a JOIN, (because of an imperfect
                                        design of the LoRa architecture, in my opinion),
                                        the actual JOINed channel sub-band is unknown because
                                        the gateway and network server can sometimes
                                        detect the WRONG channel sub-band.  To recover,
                                        the sensor must send between 2 to 9 Events to allow
                                        the underlying MAC command protocol to verify
                                        the sub-band.  It is these MAC command sequences
                                        that causes the xDot to report additional ERRORs.

            B) transmit Reset event.    This initiates a MAC command protocol
                                        exchange between the xDot and the network server.
                                        This is the begining of a multi-event protocol
                                        to absolutely determine the actual channel mask
                                        used to JOIN.  This Reset is the first event
                                        in this process.  It is usually successful.

            C) transmit 'second' event  This 'second' event is actually the first of many
                                        events (between 1 - 8) sent to the network server
                                        to unambiguously set the correct Channel Mask.
                                        This 'second' event is repeatly sent until the
                                        xDot does not return the "Tx buffer filled by MAC
                                        Commands, send data again".

                                        The magic occuring between the xDot and the
                                        network server is obscure, but from several
                                        sources online, this explanation makes sense.  But
                                        none-the-less, it is important to protect the sensor
                                        from hanging by retrying this event until the ERROR
                                        stops.  This is usually less than or equal to 8
                                        times.  (8 is the total number of sub-bands.)

                                        To ensure that 8 attempts is enough, the firmware
                                        will retry up to 20 times.  Why 20?  Because it is
                                        more than 8 and less than infinity.  It is very
                                        rare to take more than 8 attempts and an infinite
                                        number of retries causes the sensor to hang.  If
                                        this error happens more than 20 times, the sensor
                                        will automatically re-JOIN.

            D) transmit 'last' event    Sometimes on the last transmit event from above, we
                                        can get a "Transmit in progress" ERROR.  This is rare.
                                        But if it happens we simply retry this also.  There
                                        is no documentation of what this means, but
                                        experience has shown that unless you retry sending
                                        it, the previous 'last' event might not be sent.
                                        So we send it again.  But of course, to protect
                                        against hanging the sensor, we will send this also up
                                        to 20 times before a re-JOIN.

    3)  Also affecting the firmware design is the issue of Confirmed versus Unconfirmed
        events.  It is possible to configure either of these options in the General
        DOWNLINK configuration.

        *)  For US915, when a sensor is configured as Confirmed, then all Events
            are sent as Confirmed.  If a sensor is configured as Unconfirmed, then all
            events are sent as Unconfirmed except for the special periodic Link_Quality
            event which is always sent as Confirmed.  US915 sensors are initially configured
            as unconfirmed, but can be changed to Confirmed in the General DOWNLINK
            configuration for higher reliability.

        *)  For EU868, all sensors are configured as Unconfirmed and cannot be changed
            to Confirmed in the General DOWNLINK configuration.  However, there is a
            critical exception to this, the periodic Link_Quality event is always sent
            as Confirmed.

        By default, the periodic Link_Quality time is 24 hours with a range of between 1
        hour and 5 days (or so).  The importance of the periodic Link_Quality message
        is so that the Sensor has a way to regularly send a Confirmed Event that results
        in the sensor knowing if the network server received it.  If the sensor does
        not receive an ACK on the Link_Quality, then the sensor will retry it up to 3
        times.  If a total of 4 attempts fails, then the sensor will re-JOIN.

        This process is designed to protect a sensor from ultimately getting lost
        if it doesn't receive an ACK.  For example, this can happen for a catastrophic
        loss of sensor configuration in the network server, or merely an unintentional
        deletion of the sensor.  If this happens the sensor would effectively be
        lost until the battery was removed which is the only way to ensure a re-JOIN.
        This would be a disaster for a million sensors unexpectedly lost.

        Now, after all the above context, the way that the firmware can detect that
        the event ACK was lost is with the following sequence:

            *)  An event is sent with confirmation
            *)  If the sensor receives an "OK" from the xDot, then that is an ACK.
            *)  If the sensor receives an "ERROR" with a "ACK not received" ascii
                reason code, that means NOACK.
            *)  The firmware can retry 3 times (a total 4 attempts) to get a valid ACK.
                If this fails, then the sensor will re-JOIN.

    4)  Finally, the firmware allows up to 16 second for every Transmit Event to
        receive a response.  If no response occurs before this timeout, then the sensor
        does a Reset which causes a re-JOIN.

 ****************************************************************************************
 ****************************************************************************************
 */

//#endif // LORA_XDOT
//#endif // LORA
