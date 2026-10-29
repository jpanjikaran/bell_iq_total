/*
 * api_eusart_radio.c
 *
 *  Created on: Mar. 29, 2022
 *      Author: jasonp
 */
#include "api_eusart_radio.h"
#include "radio_tx_buf.h"
#include "radio_common.h"
#include "lora_xdot_radio.h"
#include "utility.h"
#include "sl_app_assert.h"
#include "sl_bluetooth.h"
#include "sl_bt_api.h"
//#include "collection.h"
//#include "swmodbeacon.h"
//#include "em_iadc.h"
//#include "tempdrv.h"
#include "uartdrv.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
//#include "em_wdog.h"
//#include "mbedtls/entropy.h"
//#include "mbedtls/entropy_poll.h"
#include "sl_uartdrv_instances.h"
//#include "time.h"
//#include "app.h"

//#define PASSTHROUGH

#define EUSART_TX_BUFFER_SIZE 64

#define EUSART_RX_BUFFER_SIZE 128
#if EUSART_RX_BUFFER_SIZE < (128)
   // The currently longest response from the xDot radio is for the ATI command (returns version string) and
   // the subsequent "OK".  Together this response is probably only about 90 bytes.  So we will set
   // the RX Buffer size to 100, a little more than necessary.  The symptoms of an RX Buffer overrun are
   // that the circular buffer wraps rendering very truncated responses in the TRC.
   #error The Read Buffer MUST be big enough to handle the largest message from Radio
#endif


volatile static uint8_t eusartTxHead = 0;
volatile static uint8_t eusartTxTail = 0;
//volatile static uint8_t eusartTxBuffer[EUSART_TX_BUFFER_SIZE];
uint8_t eusartTxBuffer[EUSART_TX_BUFFER_SIZE];

volatile static uint8_t eusartRxHead = 0;
volatile static uint8_t eusartRxTail = 0;
volatile static uint8_t eusartRxBuffer[EUSART_RX_BUFFER_SIZE];

volatile static uint8_t eusartRx1Head = 0;
volatile static uint8_t eusartRx1Tail = 0;
volatile static uint8_t eusartRx1Buffer[EUSART_RX_BUFFER_SIZE];

extern uint8_t test_mode_active;

//--------------------------------------------------------------------------------------
static void uart_increment_rx_head(void)
{
    eusartRxHead++;

    if(eusartRxHead >= EUSART_RX_BUFFER_SIZE)
        eusartRxHead = 0;
}
static void uart1_increment_rx_head(void)
{
    eusartRx1Head++;

    if(eusartRx1Head >= EUSART_RX_BUFFER_SIZE)
        eusartRx1Head = 0;
}
//--------------------------------------------------------------------------------------
static void uart_increment_rx_tail(void)
{
    eusartRxTail++;

    if(eusartRxTail >= EUSART_RX_BUFFER_SIZE)
        eusartRxTail = 0;
}

static void uart1_increment_rx_tail(void)
{
    eusartRx1Tail++;

    if(eusartRx1Tail >= EUSART_RX_BUFFER_SIZE)
        eusartRx1Tail = 0;
}
//--------------------------------------------------------------------------------------
static void uart_increment_tx_head(void)
{
    eusartTxHead++;

    if(eusartTxHead >= EUSART_TX_BUFFER_SIZE)
        eusartTxHead = 0;
}
//--------------------------------------------------------------------------------------
static void uart_increment_tx_tail(void)
{
    eusartTxTail++;

    if(eusartTxTail >= EUSART_TX_BUFFER_SIZE)
        eusartTxTail = 0;
}
//--------------------------------------------------------------------------------------
//void API_EUSART_RADIO__Initialize(uint8_t hardware_gen, uint8_t radio_module)
//{
//
//  /*EUSART INIT WILL BE HANDLED AUTOMATICALLY
//   *
//   *
//   */
//    // disable interrupts before changing states
//    API_EUSART_RADIO__DISABLE_INTERRUPTS();
//
//    // Set the EUSART module to the options selected in the user interface.
//
//    // ABDOVF no_overflow; SCKP Non-Inverted; BRG16 16bit_generator; WUE enabled; ABDEN disabled;
////    BAUD1CON = 0x0A;
//
//    // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled;
////    RC1STA = 0x90;
//
//    // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave;
////    TX1STA = 0x24;
//
////    if (hardware_gen == API_HW_GEN_VALUE_1)
////    {
////        switch(radio_module)
////        {
////            case API_EUSART_RADIO_MODULE__SIGFOX_WISOL:
////                SP1BRGL = 0x19;    // 9600 baud, 4Mhz clock, 4 clock divider, 16LF18326
////                SP1BRGH = 0x00;
////                break;
////            case API_EUSART_RADIO_MODULE__LORA_MURATA:
////                SP1BRGL = 0x0C;    // 19200 baud, 4Mhz clock, 4 clock divider, 16LF18326
////                SP1BRGH = 0x00;
////                break;
////            case API_EUSART_RADIO_MODULE__LORA_XDOT:
////                // This configuration is very unlikely to happen, punt for now
////                return;
////            default:
////                return;
////        }
////    }
////    else if (hardware_gen == API_HW_GEN_VALUE_2)
////    {
////        switch(radio_module)
////        {
////            case API_EUSART_RADIO_MODULE__SIGFOX_WISOL:
////                SP1BRGL = 0x9B;    // 9600 baud, 12 Mhz clock, 2 clock divider
////                SP1BRGH = 0x00;
////                break;
////            case API_EUSART_RADIO_MODULE__LORA_MURATA:
////                SP1BRGL = 0x4D;    // 19200 baud
////                SP1BRGH = 0x00;
////                break;
////            case API_EUSART_RADIO_MODULE__LORA_XDOT:
////                SP1BRGL = 0x0C;    // 115K baud, 12 Mhz clock, 2 clock divider
////                SP1BRGH = 0x00;
////                break;
////            default:
////                return;
////        }
////    }
////    else
////    {
////        return;  // clearly an error, but just punt for now.
////    }
//
//    // initializing the driver state
//    eusartTxHead = 0;
//    eusartTxTail = 0;
//
//    eusartRxHead = 0;
//    eusartRxTail = 0;
//
//    API_EUSART_RADIO__ENABLE_INTERRUPTS();
//}

//--------------------------------------------------------------------------------------
void API_EUSART_RADIO__Receive_ISR(void)
{
    if(USART0->STATUS & USART_STATUS_RXDATAV)
    {
//        // EUSART error - restart
//
//        RC1STAbits.CREN = 0;
//        RC1STAbits.CREN = 1;
//    }

    // buffer overruns are ignored
    eusartRxBuffer[eusartRxHead] = USART_Rx(USART0);
    uart_increment_rx_head();
    }
}


//--------------------------------------------------------------------------------------
bool API_EUSART_RADIO__Tx_Buffers_Empty(void)
{
    return (eusartTxHead == eusartTxTail);
}

//--------------------------------------------------------------------------------------
bool API_EUSART_RADIO__Are_TX_RX_Buffers_Empty(void)
{
    return (eusartTxHead == eusartTxTail) && (eusartRxHead == eusartRxTail);
}
//--------------------------------------------------------------------------------------
void API_EUSART_RADIO__TX_Data_Send(void)
{
//  uint8_t tmp_buf;
    // if the uart shift register is empty, then check to see if there is new data to send
//    if(TX1STAbits.TRMT)
//    {
        // if the head and tail are not the same, then there is pending data
        if(eusartTxHead != eusartTxTail)
        {
            // TX1REG = eusartTxBuffer[eusartTxTail];
            // Replaced the above (commented out) statement with the following #define invocation.  This
            // is done to support both the product hardware and the PC model of the hardware.
//            TX1REG = eusartTxBuffer[eusartTxTail];
//            while (!(USART0->STATUS & USART_STATUS_TXBL))
//              {
//
//              }
//            tmp_buf = eusartTxBuffer[eusartTxTail];
//            for (tmp_buf=0; tmp_buf<3; tmp_buf++)
              USART_Tx(USART0,eusartTxBuffer[eusartTxTail]);
//            USART_Tx(USART0,tmp_buf);

            uart_increment_tx_tail();
        }
//    }
}
//--------------------------------------------------------------------------------------
bool API_EUSART_RADIO__RX_Data_Available(void)
{
    // there is read data pending if head and tail are different
    if(eusartRxHead != eusartRxTail)
        return true;
    else
        return false;
}

bool API_EUSART1_RADIO__RX_Data_Available(void)
{
    // there is read data pending if head and tail are different
    if(eusartRx1Head != eusartRx1Tail)
        return true;
    else
        return false;
}
//--------------------------------------------------------------------------------------
void API_EUSART_RADIO__Write(uint8_t txData)
{
    // add to uart buffer
    eusartTxBuffer[eusartTxHead] = txData;
    uart_increment_tx_head();
    // Enable transmit buffer level interrupt

    // if after incrementing head it is equal to the tail, then we have overflow
    // so push the tail forward as well so we only lose the oldest data
    if(eusartTxHead == eusartTxTail)
        uart_increment_tx_tail();
}
//--------------------------------------------------------------------------------------
uint8_t API_EUSART_RADIO__Read(void)
{
    uint8_t readValue  = 0;

    readValue = eusartRxBuffer[eusartRxTail];
    uart_increment_rx_tail();

    return readValue;
}

uint8_t API_EUSART1_RADIO__Read(void)
{
    uint8_t readValue  = 0;

    readValue = eusartRx1Buffer[eusartRx1Tail];
    uart1_increment_rx_tail();

    return readValue;
}

/**************************************************************************//**
 * @brief
 *    USART0 initialization
 *****************************************************************************/
void initUSART0(void)
{
  // Default asynchronous initializer (115.2 Kbps, 8N1, no flow control)
  USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

  // Route USART0 TX and RX to the board controller TX and RX pins
//  GPIO->USARTROUTE[0].TXROUTE = (BSP_BCC_TXPORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
//      | (BSP_BCC_TXPIN << _GPIO_USART_TXROUTE_PIN_SHIFT);
//  GPIO->USARTROUTE[0].RXROUTE = (BSP_BCC_RXPORT << _GPIO_USART_RXROUTE_PORT_SHIFT)
//      | (BSP_BCC_RXPIN << _GPIO_USART_RXROUTE_PIN_SHIFT);
  //init.baudrate = 9600; // 9600 bps, 8N1, no flow control
  // Enable RX and TX signals now that they have been routed
  GPIO->USARTROUTE[0].ROUTEEN = GPIO_USART_ROUTEEN_RXPEN | GPIO_USART_ROUTEEN_TXPEN;

  // Configure and enable USART0
  USART_InitAsync(USART0, &init);

//#ifndef PASSTHROUGH
  if(test_mode_active==0)
  // Enable transmit buffer level interrupt
    USART_IntEnable(USART0, USART_IEN_TXBL);
//#endif

  // Enable receive buffer level interrupt
 // USART_IntEnable(USART0, USART_IEN_RXDATAV);

  // Enable NVIC USART sources

  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
  NVIC_EnableIRQ(USART0_RX_IRQn);
  NVIC_ClearPendingIRQ(USART0_TX_IRQn);
  NVIC_EnableIRQ(USART0_TX_IRQn);

//#ifdef PASSTHROUGH
//  NVIC_ClearPendingIRQ(USART1_RX_IRQn);
//  NVIC_EnableIRQ(USART1_RX_IRQn);
////  NVIC_ClearPendingIRQ(USART1_TX_IRQn);
//  USART_IntEnable(USART1, USART_IEN_RXDATAV);
////  NVIC_EnableIRQ(USART1_TX_IRQn);
//#endif

  // Enable receive buffer level interrupt
  USART_IntEnable(USART0, USART_IEN_RXDATAV);
}

/**************************************************************************//**
 * @brief
 *    USART0 initialization
 *****************************************************************************/
void initUSART1(void)
{
  // Default asynchronous initializer (115.2 Kbps, 8N1, no flow control)
  USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

  // Route USART0 TX and RX to the board controller TX and RX pins
//  GPIO->USARTROUTE[0].TXROUTE = (BSP_BCC_TXPORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
//      | (BSP_BCC_TXPIN << _GPIO_USART_TXROUTE_PIN_SHIFT);
//  GPIO->USARTROUTE[0].RXROUTE = (BSP_BCC_RXPORT << _GPIO_USART_RXROUTE_PORT_SHIFT)
//      | (BSP_BCC_RXPIN << _GPIO_USART_RXROUTE_PIN_SHIFT);
  //init.baudrate = 9600; // 9600 bps, 8N1, no flow control
  // Enable RX and TX signals now that they have been routed
  GPIO->USARTROUTE[0].ROUTEEN = GPIO_USART_ROUTEEN_RXPEN | GPIO_USART_ROUTEEN_TXPEN;

  // Configure and enable USART0
  USART_InitAsync(USART1, &init);


  // Enable receive buffer level interrupt
 // USART_IntEnable(USART0, USART_IEN_RXDATAV);

  // Enable NVIC USART sources

//  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
//  NVIC_EnableIRQ(USART0_RX_IRQn);
//  NVIC_ClearPendingIRQ(USART0_TX_IRQn);
//  NVIC_EnableIRQ(USART0_TX_IRQn);

//#ifdef PASSTHROUGH
  NVIC_ClearPendingIRQ(USART1_RX_IRQn);
  NVIC_EnableIRQ(USART1_RX_IRQn);
//  NVIC_ClearPendingIRQ(USART1_TX_IRQn);
//  USART_IntEnable(USART1, USART_IEN_RXDATAV);
//  NVIC_EnableIRQ(USART1_TX_IRQn);
//#endif

  // Enable receive buffer level interrupt
  USART_IntEnable(USART1, USART_IEN_RXDATAV);
}
/**************************************************************************//**
 * @brief
 *    The USART0 transmit interrupt outputs characters.
 *****************************************************************************/
void USART0_TX_IRQHandler(void)
{
//  static uint16_t tx_index = 0;
//  uint8_t i=0;
    if (eusartTxHead != eusartTxTail)
    {
        USART0->TXDATA = eusartTxBuffer[eusartTxTail];
        uart_increment_tx_tail();
    }
    else
    {
        USART_IntDisable(USART0, USART_IEN_TXBL);
//        eusartTxTail = 0;
//        eusartTxHead = 0;
//            for(i=0; i<64; i++)
//              eusartTxBuffer[i] = 0;
//        tx_index = 0;
//
//        if ((production_mode == false) && (ASIC_conversion_in_progress == false))
//        {
//            my_sys_is_ready_to_sleep = true;
//        }
//
//        uart0_transmitter_idle = true;
    }
}


void USART0_RX_IRQHandler(void)
{
//    static uint16_t rx_index = 0;
//    uint8_t new_char;
//    production_mode_time_out = soft_timer0_counter;
    // Get the character just received
//#ifdef PASSTHROUGH
//  if(test_mode_active==1)
//    {
//      uint8_t new_char;
//      new_char = USART_Rx(USART0);
//      USART_Tx(USART1,new_char);
//    }
//else
  {
    eusartRxBuffer[eusartRxHead] = USART_Rx(USART0);
    uart_increment_rx_head();
  }
//#endif
//    if ((rx_index < BUFLEN) && (message_ready_to_be_processed == false))
//    {
//        if (new_char == '\r')
//        {
//            my_rx_buffer[rx_index] = '\0';
//            message_ready_to_be_processed = true;
//            rx_index = 0;
//        }
//        else
//        {
//            my_rx_buffer[rx_index] = new_char;
//            rx_index++;
//        }
//    }
//    else
//    {
//        rx_index = 0;
//    }
}
void USART1_RX_IRQHandler(void)
{
//    static uint16_t rx_index = 0;
    uint8_t new_char;
//    production_mode_time_out = soft_timer0_counter;
    // Get the character just received
//    new_char = USART_Rx(USART1);
//    USART_Tx(USART0,new_char);

    eusartRx1Buffer[eusartRx1Head] = USART_Rx(USART1);
    uart1_increment_rx_head();

//    uart_increment_rx_head();
//    if ((rx_index < BUFLEN) && (message_ready_to_be_processed == false))
//    {
//        if (new_char == '\r')
//        {
//            my_rx_buffer[rx_index] = '\0';
//            message_ready_to_be_processed = true;
//            rx_index = 0;
//        }
//        else
//        {
//            my_rx_buffer[rx_index] = new_char;
//            rx_index++;
//        }
//    }
//    else
//    {
//        rx_index = 0;
//    }
}

