/*
 * api_eusart_radio.h
 *
 *  Created on: Mar. 29, 2022
 *      Author: jasonp
 */
#include "sl_bt_api.h"

#ifndef API_EUSART_RADIO_H_
#define API_EUSART_RADIO_H_


#define API_EUSART_RADIO_MODULE__SIGFOX_WISOL   0x01 // deprecated
#define API_EUSART_RADIO_MODULE__LORA_MURATA    0x02 // deprecated
#define API_EUSART_RADIO_MODULE__LORA_XDOT      0x03
#define API_HW_GEN_1


#ifdef API_HW_GEN_1
    #define API_HW_GEN  API_HW_GEN_VALUE_1
#elif defined(API_HW_GEN_2)
    #define API_HW_GEN  API_HW_GEN_VALUE_2
#else
    #error must pick one
#endif

#if defined(API_HW_GEN_1)

  #define API_EUSART_RADIO__DISABLE_INTERRUPTS() //{PIE1bits.RCIE = 0; PIE1bits.TXIE = 0;}
  #define API_EUSART_RADIO__ENABLE_INTERRUPTS() //{PIE1bits.RCIE = 1;}

#elif defined(API_HW_GEN_2)

  #define API_EUSART_RADIO__DISABLE_INTERRUPTS() {PIE3bits.RC1IE = 0;}
  #define API_EUSART_RADIO__ENABLE_INTERRUPTS() {PIE3bits.RC1IE = 1;}

//#else
//  #error "Cool!  New hardware generation to implement"
#endif

//void API_EUSART_RADIO__Initialize(uint8_t hardware_gen, uint8_t radio_module);


void EUSART1_Initialize(void);

void API_EUSART_RADIO__TX_Data_Send(void);
void API_EUSART_RADIO__Write(uint8_t data);
uint8_t API_EUSART_RADIO__Read(void);
uint8_t API_EUSART1_RADIO__Read(void);
bool API_EUSART_RADIO__RX_Data_Available(void);
bool API_EUSART1_RADIO__RX_Data_Available(void);
bool API_EUSART_RADIO__Tx_Buffers_Empty(void);
bool API_EUSART_RADIO__Are_TX_RX_Buffers_Empty(void);
void API_EUSART_RADIO__Receive_ISR(void);
void initUSART0(void);
void initUSART1(void);

#endif /* API_EUSART_RADIO_H_ */
