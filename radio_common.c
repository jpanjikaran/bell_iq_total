/*
 * radio_common.c
 *
 *  Created on: Mar. 22, 2022
 *      Author: jasonp
 */


/*
 Common functions for radio comm

  (c) 2017 Radio Bridge Inc. and its subsidiaries.
*/
//#include "main.h"
#include "radio_common.h"
//#include "timers.h"
#include "downlink_messages.h"
#include "build_config.h"
//#include "led_flash.h"
#define LORA_XDOT
//#ifdef LORA
//    #ifdef LORA_XDOT
        #include "lora_xdot_radio.h"
#include "em_letimer.h"
#include "em_cmu.h"
#include "sl_uartdrv_instances.h"
#include "uartdrv.h"
#include "string.h"
#include "stdio.h"
#include "lora_api_internal_data.h"
#include "lora_api_internal_join.h"
#include "utility.h"
//    #endif // LORA_XDOT
//#endif


    #define LINK_QUALITY_PERIOD__IN_MINUTES_DEFAULT (24 * 60)  // (24 hrs * 60 min/hr)
    #define LINK_QUALITY_PERIOD__IN_MINUTES_MINIMUM ( 1 * 60)  // ( 1 hrs * 60 min/hr)

// buffer to hold incoming message from radio
volatile uint8_t radio_uart_read_buffer[RADIO_READ_BUFFER_SIZE];
volatile uint8_t radio_uart_read_index = 0;

volatile bool radio_response_has_timed_out = false;

// current timeout value for radio state machine
volatile uint8_t radio_state_delay_timeout;
// used for measuring delays in radio state machine
volatile uint8_t radio_state_delay_timer = 0;
// true when we are timing a delay between states
volatile bool radio_state_delay_running = false;


// timer for radio uart timeout
volatile uint16_t radio_response_timer = 0;
volatile uint16_t radio_response_timeout_val = 0;
volatile bool radio_response_timeout = false;
volatile bool radio_response_timer_is_running = false;

// timer for radio governor
volatile uint32_t radio_governor_timer = 0;
volatile static bool radio_governor_running = false;

// timer for radio downlink timeout
volatile uint16_t radio_downlink_timer = 0;
volatile bool radio_downlink_timeout = false;

uint16_t uart_tmr = 0;
uint8_t tmr_cnt1000 = 0;
uint8_t tmr_cnt300 = 0;

uint16_t link_quality_period = LINK_QUALITY_PERIOD__IN_MINUTES_DEFAULT;
uint16_t link_quality_periodic_timer = 0;
bool     link_quality_periodic_timer_expired = false;

#ifdef LORA_MURATA
    // indicates that the next 'lora_downlink_num_bytes' bytes will contain downlink data.
    volatile bool lora_downlink_bytes_next = false;

    // number of expected bytes in a downlink, generally always 8.  Anything other than 8 results in an INVALID ACK  0xFF01
    extern int8_t lora_downlink_num_bytes;

#endif // LORA_MURATA

// This compiler check is redundent and is also checked elsewhere, but is necessary to absolutely
// ensure that the size of Tx_string and next_event_buf are large enough.  If too small, than a
// very hard to debug run time error is possible where the buffer is over-written.
#if TX_BUF_SIZE_MAX < (TX_STRING_LENGTH_MAX + 1)
   #error TX_BUF_SIZE_MAX must be >= DOWNLINK_MESSAGE_LENGTH
#endif
char Tx_string[TX_STRING_LENGTH_MAX + 1];  // +1 is for '\0' character

// These two variables contain the next_event_buf_string and next_event_downlink_request flag from the call
// to radio_tx_buf_dequeue().  It is called immediately after determining that there is definately
// something to send with radio_tx_buf_has_event_to_send().  It is necessary to make a copy of these
// values because it is possible that the priority queue implementation may need to change the
// state of the queue if a new sensor event is enqueued between these two calls.  This ensures
// that nothing gets out of sync to make this copy.
char next_event_buf_string[TX_STRING_LENGTH_MAX + 1];  // +1 is for '\0' character
bool next_event_downlink_request;

extern deviceDATA MYdevice;
extern test_mode_active;
//#define PASSTHROUGH


//-----------------------------------------------------------------------------------
void radio_transmit_gen_hex(uint8_t index, uint8_t data)
{
    uint8_t nibble;
    index *= 2;
    nibble = data >> 4;
    if (nibble < 10)
    {
        Tx_string[index] = nibble + '0';
    }
    else
    {
        Tx_string[index] = nibble + 'A' - 10;
    }
    index++;
    nibble = data & 0x0f;
    if (nibble < 10)
    {
        Tx_string[index] = nibble + '0';
    }
    else
    {
        Tx_string[index] = nibble + 'A' - 10;
    }
    index++;
    Tx_string[index] = '\0'; // always keep the string null terminated.
}

////-----------------------------------------------------------------------------------
void radio_transmit_hex_data_6(uint8_t protocol_version, uint8_t radio_event, uint8_t data_2, uint8_t data_3, uint8_t data_4, bool downlink_request, bool join_wakeup)
{
    // same as: s-printf(Tx_string, "%02%02x%02x%02x%02x", protocol_version, radio_event, data_2, data_3, data_4)
    radio_transmit_gen_hex(0, protocol_version);
    radio_transmit_gen_hex(1, radio_event);
    radio_transmit_gen_hex(2, data_2);
    radio_transmit_gen_hex(3, data_3);
    radio_transmit_gen_hex(4, data_4);
    radio_transmit(radio_event, downlink_request, join_wakeup);
}

//-----------------------------------------------------------------------------------
void radio_transmit_hex_data_12(uint8_t protocol_version, uint8_t radio_event, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7, uint8_t data_8, uint8_t data_9, uint8_t data_10, bool downlink_request, bool join_wakeup)
{
    // same as: s-printf(Tx_string, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", protocol_version, radio_event, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10)
    radio_transmit_gen_hex(0, protocol_version);
    radio_transmit_gen_hex(1, radio_event);
    radio_transmit_gen_hex(2, data_2);
    radio_transmit_gen_hex(3, data_3);
    radio_transmit_gen_hex(4, data_4);
    radio_transmit_gen_hex(5, data_5);
    radio_transmit_gen_hex(6, data_6);
    radio_transmit_gen_hex(7, data_7);
    radio_transmit_gen_hex(8, data_8);
    radio_transmit_gen_hex(9, data_9);
    radio_transmit_gen_hex(10, data_10);
    radio_transmit(radio_event, downlink_request, join_wakeup);
}

//-----------------------------------------------------------------------------------
// send a tx string to the radio
void radio_transmit(uint8_t radio_event, bool downlink_request, bool join_wakeup)
{
    // enqueue in the priority transmit queue
    radio_tx_buf_enqueue(radio_event, downlink_request, join_wakeup);
}
//--------------------------------------------------------------------------------
// send a string to the radio
void radio_send_string(const char *uart_string, uint8_t len)
//void radio_send_string(uint8_t *uart_string)
{
//    TRC_RADIO_SEND(uart_string);

    uint8_t length = 10;
    length = sizeof(uart_string);
  length = len;

//    LED_FLASH__TURN_ON();  // before transmit - must be paired with LED_FLASH__TURN_OFF() below

    // UART tx to radio
    for (uint8_t i = 0; i < length; i++)
        API_EUSART_RADIO__Write(uart_string[i]);
//
//    USART_IntEnable(USART0, USART_IEN_TXBL);
//    LED_FLASH__TURN_OFF();  // after transmit - must be paired with LED_FLASH__TURN_ON() above
}
//--------------------------------------------------------------------------------
void check_radio_uart_read(void)
{
    // if there is data in the uart then add to the buffer
//  API_EUSART_RADIO__Receive_ISR();
    if (API_EUSART_RADIO__RX_Data_Available())
    {
        //-----------------------------------------------------------------
        //-----------------------------------------------------------------
        #ifdef LORA_XDOT
            uint8_t data = API_EUSART_RADIO__Read();

//#ifdef PASSTHROUGH
            if(test_mode_active==1)
              USART_Tx(USART1, data);
else
  {
            if ((data == '\r') || (data == '\n'))
            {
                if (radio_uart_read_index != 0)
                {
                    process_lora_message(radio_uart_read_index);
                    radio_uart_read_index = 0;
                }
            }
            else
            {
                radio_uart_read_buffer[radio_uart_read_index++] = data;

                // buffer overflow
                if(radio_uart_read_index >= RADIO_READ_BUFFER_SIZE)
                {
                    // reset buffer and exit
                    radio_uart_read_index = 0;
                }
            }
        #endif // LORA_XDOT
        //-----------------------------------------------------------------
        //-----------------------------------------------------------------
        #ifdef LORA_MURATA

            uint8_t data = API_EUSART_RADIO__Read();

            radio_uart_read_buffer[radio_uart_read_index++] = data;

            // buffer overflow
            if(radio_uart_read_index >= RADIO_READ_BUFFER_SIZE)
            {
                // reset buffer and exit
                radio_uart_read_index = 0;
                return;
            }

            // if we receive downlink data bytes
            if (lora_downlink_bytes_next && (radio_uart_read_index == (lora_downlink_num_bytes * 2)))
            {
                process_lora_message(radio_uart_read_index);
                radio_uart_read_index = 0;
            }
            // else if we received a complete message then process
            else if (radio_uart_read_index >= 4)
            {
                // if message terminates with \r\n\r\n then we have a full message
                if (((radio_uart_read_buffer[(uint8_t)(radio_uart_read_index-4)] == '\r') &&
                     (radio_uart_read_buffer[(uint8_t)(radio_uart_read_index-3)] == '\n') &&
                     (radio_uart_read_buffer[(uint8_t)(radio_uart_read_index-2)] == '\r') &&
                     (radio_uart_read_buffer[(uint8_t)(radio_uart_read_index-1)] == '\n')))
                {
                    process_lora_message(radio_uart_read_index);
                    radio_uart_read_index = 0;
                }
            }

        #endif // LORA_MURATA
        //-----------------------------------------------------------------
        //-----------------------------------------------------------------
        #ifdef SIGFOX
            uint8_t data = API_EUSART_RADIO__Read();

            radio_uart_read_buffer[radio_uart_read_index++] = data;

            // buffer overflow
            if(radio_uart_read_index >= RADIO_READ_BUFFER_SIZE)
            {
                // reset buffer and exit
                radio_uart_read_index = 0;
                return;
            }
            // if we received a complete message then process
            if(radio_uart_read_index > 3)
            {
                // if message terminates with \r\n then we have a full message
                if((radio_uart_read_buffer[(uint8_t)(radio_uart_read_index-2)] == '\r') &&
                (radio_uart_read_buffer[(uint8_t)(radio_uart_read_index-1)] == '\n'))
                {
                    process_sigfox_message(radio_uart_read_index);
                    radio_uart_read_index = 0;
                }
            }
        #endif // SIGFOX
        //-----------------------------------------------------------------
        //-----------------------------------------------------------------

        // UART error so restart
//        if (1 == RC1STAbits.OERR)
//        {
//            RC1STAbits.SPEN = 0;
//            RC1STAbits.SPEN = 1;
//        }
  }
//#endif
    }
    if (API_EUSART1_RADIO__RX_Data_Available())
      {
        uint8_t data = API_EUSART1_RADIO__Read();
        USART_Tx(USART0, data);
      }
}
// reset the timer for radio state delays
void reset_radio_state_delay_timer(void)
{
    radio_state_delay_timer = 0;
    radio_state_delay_running = true;
}


//-----------------------------------------------------------------------------------
// add a delay between states for the main radio state machine
// state_delay is in increments of 100ms
void set_radio_state_delay(uint8_t state_delay)
{
    radio_state_delay_timeout = state_delay;
    reset_radio_state_delay_timer();
}
//-----------------------------------------------------------------------------------
// flush the buffer so there are no remaining chars
void radio_uart_flush(void)
{
    volatile uint8_t dummy_read;

    // read out any remaining chars in buffer
    while (API_EUSART_RADIO__RX_Data_Available())
        dummy_read = API_EUSART_RADIO__Read();

    // clear software buffer
    radio_uart_read_index = 0;
}

void reset_link_quality_periodic_timer(void)
{
    link_quality_periodic_timer = 0;
    link_quality_periodic_timer_expired = false;
}



// reset the radio uart timeout
void start_radio_response_timeout(uint16_t timeout_val)
{
    radio_response_timer = 0;
    radio_response_timeout_val = timeout_val;
    radio_response_timeout = false;
    radio_response_timer_is_running = true;

//    serial_out_pause();  // must be BEFORE the following timer is started
//    API_TMR_1000_MS_StartTimer();
}

void stop_radio_response_timeout(void)
{
  //stop 1s timer for timeout need to create
//  serial_out_paused = false;
  radio_response_timer_is_running = false;

}


void reset_radio_governor(uint32_t timeout_val)
{
    if (timeout_val != 0)
    {
        radio_governor_running = true;

    }
    else
    {
        radio_governor_running = false;

    }
    radio_governor_timer = timeout_val;
    radio_ok_to_send = false;
}

bool is_timer_running(void)
{
  if(radio_governor_running||radio_response_timer_is_running)
    return true;
  else
    return false;
}
//#include "em_letimer.h"
extern bool start_Letimer0;
//extern bool my_sys_is_ready_to_sleep;

void LETIMER0_IRQHandler(void)
{
    // Acknowledge the interrupt
    uint32_t flags = LETIMER_IntGet(LETIMER0);
    LETIMER_IntClear(LETIMER0, flags);
    // Check for Under flow event
    if (flags & LETIMER_IF_UF)
    {
        // measure state delays for radio state machine
        if (radio_state_delay_timer > radio_state_delay_timeout)
            radio_state_delay_running = false;
        else
            radio_state_delay_timer++;

        uart_tmr++;
        tmr_cnt300++;
        tmr_cnt1000++;
        if((tmr_cnt300>2)&&(radio_governor_running))
          {
            tmr_cnt300=0;
            if (radio_governor_timer == 0)
            {
                // only okay to send to radio when timer expires from last tx
                radio_ok_to_send = true;
                if(!lora_api_internal_join__is_network_joined()&& MYdevice.lora.join_begin)
                  {
                    MYdevice.lora.rejoin=1;
                    MYdevice.lora.join_begin = 0;
//                     char_send('D',0);
                  }

                // since the radio timer just hit, we need to stop the timer
                radio_governor_running = false;

            }
            else // else we know that the timer has time left, so decrement it
            {
                radio_governor_timer--;
            }
          }
        if((tmr_cnt1000>9)&&(radio_response_timer_is_running))
          {
            tmr_cnt1000 = 0;
            // timer for radio uart timeout
            radio_response_timer++;
            if (radio_response_timer & radio_response_timeout_val)
            {
                // the uart read timed out
                radio_response_timeout = true;
                radio_response_timer_is_running = false;
            }

            // timer for radio downlink timeout
            radio_downlink_timer++;
            if (radio_downlink_timer & RADIO_DOWNLINK_TIMEOUT)
            {
                // the uart downlink timed out
                radio_downlink_timeout = true;
            }


          }

    }
}

void initLetimer(uint8_t en)
{

  LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;

  if(en==1)
    {
  // Reload top on underflow, set output, and run in one-shot mode
  letimerInit.enable = false;
  letimerInit.comp0Top = true;
  letimerInit.repMode = letimerRepeatFree;
  CMU_ClockEnable(cmuClock_LETIMER0, true);

  LETIMER_RepeatSet(LETIMER0, 0, 1);
  LETIMER_Init(LETIMER0, &letimerInit);

  uint32_t flags = LETIMER_IntGet(LETIMER0);
  LETIMER_IntClear(LETIMER0, flags);

  LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);
  NVIC_EnableIRQ(LETIMER0_IRQn);
    }
  else
    {
      LETIMER_Enable(LETIMER0, false);
    }
}

//------------------------------------------------------------------------------
//      Low power ms delay using LeTimer for receiving total response
//------------------------------------------------------------------------------
void lowPowerDelay(uint32_t millisec)
{
    LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;
    letimerInit.enable = true;
    letimerInit.comp0Top = true;
    letimerInit.repMode = letimerRepeatFree;

    millisec = 100;
    LETIMER_TopSet(LETIMER0, CMU_ClockFreqGet(cmuClock_LETIMER0) * millisec / 1000);  // Set the top value of the countdown timer
    LETIMER_RepeatSet(LETIMER0, 0, 1);                                                // Configure one-shot mode
    LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);
    LETIMER_Init(LETIMER0, &letimerInit);
}
