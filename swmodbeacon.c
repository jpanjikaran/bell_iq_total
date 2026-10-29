/*
 * swmodbeacon.c
 *
 *  Created on: Jan 26, 2018
 *      Author: joet
 */

#include "swmodbeacon.h"
#include "em_system.h"
#include "sl_app_assert.h"

#include "em_adc.h"
#include "collection.h"
#include "app-config.h"
#include "utility.h"
#include "sl_status.h"

#define Day 86400/2
#define Week (Day*7)/2
#define BiWeek (Day*14)/2
#define Month (Day*30)/2
#define BiMonth (Day*60)/2
#define Quarter (Day*90)/2

#define ADC_INPUTtemp adcPosSelTEMP
#define ADC_INPUT1    adcPosSelAPORT0XCH0
#define ADC_12BIT_MAX           4096                    /* 2^12 */
#define ADC_16BIT_MAX           65536                   /* 2^16 */
#define ADC_CLOCK               1000000                 /* ADC conversion clock */


// Macros.
#define UINT16_TO_BYTES(n)            ((uint8_t) (n)), ((uint8_t)((n) >> 8))
#define UINT16_TO_BYTE0(n)            ((uint8_t) (n))
#define UINT16_TO_BYTE1(n)            ((uint8_t) ((n) >> 8))


#define ADC_SCAN_DVL       4

//extern int8_t       TXpwr;
extern uint8_t      dispTemp;
extern uint8_t      BattVolts;

//extern uint8_t      EventsCounter;
//extern uint16_t     DetectionsCounter;
//extern uint8_t      device_sku ;
//extern uint8_t      DeviceRegistration;
//extern uint8_t      factory_wake;
extern volatile uint32_t tempSample;
extern float Celsius;
extern uint8_t dispTemp;
extern uint8_t minTemp;
extern uint8_t maxTemp;
//extern const char hexasctable[];

extern uint32_t getLocalTime(void);
extern deviceDATA  MYdevice;
extern collectionType coll;
extern battData DeviceBatt;
extern MTsystem_t   MTsystem;

//static uint8_t advertising_set_handle = 0xff;

smartwavebcn myBeacon;

/**************************************************************************//**
 * @brief Initialize ADC for single and scan conversion
 *****************************************************************************/

void adcReset(void)
{
	 /* Reset ADC registers */
//	  ADC_Reset(ADC0);
}

//void setupSensorxx(void)
//{
//  /* Base the ADC configuration on the default setup. */
//  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
//  ADC_InitSingle_TypeDef sInit = ADC_INITSINGLE_DEFAULT;
//
//  /* Enable ADC clock */
//   CMU_ClockEnable(cmuClock_ADC0, true);
//
//  /* Initialize timebases */
//  init.timebase = ADC_TimebaseCalc(0);
//  init.prescale = ADC_PrescaleCalc(400000, 0);
//  ADC_Init(ADC0, &init);
//
//  /* Set input to temperature sensor. Reference must be 1.25V */
//  sInit.reference = adcRef1V25;
//  sInit.posSel = adcPosSelTEMP;
//  sInit.negSel = adcNegSelVSS;
//  ADC_InitSingle(ADC0, &sInit);
//
//}

//void TempMeasure(void)
//{
//    // temperature
//    ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
//    ADC_InitSingle_TypeDef sInit = ADC_INITSINGLE_DEFAULT;
//
//      /* Enable ADC clock */
//    CMU_ClockEnable(cmuClock_ADC0, true);
//
//      /* Initialize timebases */
//    init.timebase = ADC_TimebaseCalc(0);
//    init.prescale = ADC_PrescaleCalc(400000, 0);
//    ADC_Init(ADC0, &init);
//
//      /* Set input to temperature sensor. Reference must be 1.25V */
//    sInit.reference = adcRef1V25;
//    sInit.posSel = adcPosSelTEMP;
//    sInit.negSel = adcNegSelVSS;
//    ADC_InitSingle(ADC0, &sInit);
//    ADC_Start(ADC0, adcStartSingle);
//    //while (ADC0->STATUS & ADC_STATUS_SINGLEACT); // ADC_STATUS_SINGLEDV  was ADC_STATUS_SINGLEACT
//    //while ((ADC0->IF & ADC_STATUS_SINGLEDV) == 0);
//    while ( ( ADC0->STATUS & ADC_STATUS_SINGLEDV ) == 0 ){}
//    tempSample = ADC_DataSingleGet(ADC0);
//    Celsius =  ConvertToCelsius(tempSample);
//    //Celsius = -10;
//    dispTemp = Celsius + 128;
//    if(dispTemp > maxTemp) maxTemp = dispTemp;
//    if(dispTemp < minTemp) minTemp = dispTemp;
//}

//uint32_t BattMeasure(void)
//{
//    uint32_t rawADC = 0;
//  /* Base the ADC configuration on the default setup. */
//    ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
//    ADC_InitSingle_TypeDef sInit = ADC_INITSINGLE_DEFAULT;
//
//  /* Enable ADC clock */
//    CMU_ClockEnable(cmuClock_ADC0, true);
//
//  /* Initialize timebases */
//    init.timebase = ADC_TimebaseCalc(0);
//    init.prescale = ADC_PrescaleCalc(400000, 0);
//    ADC_Init(ADC0, &init);
//
//  /* Set input to temperature sensor. Reference must be 1.25V */
//    sInit.reference =  adcRef5V ;
//    sInit.posSel = adcPosSelAVDD; // TODO: try measuring this: adcPosSelAVDD  was adcPosSelIOVDD
//    sInit.negSel = adcNegSelVSS;
//    ADC_InitSingle(ADC0, &sInit);
//
//    ADC_Start(ADC0, adcStartSingle);
//   // while ((ADC0->IF & ADC_STATUS_SINGLEDV) == 0);
//    //while ( ADC0->STATUS & ADC_STATUS_SINGLEACT); // TODO: wait on this flag:ADC_STATUS_SINGLEDV  was ADC_STATUS_SINGLEACT    ADC0->STATUS & ADC_STATUS_SINGLEDV
//    while ( ( ADC0->STATUS & ADC_STATUS_SINGLEDV ) == 0 ){}
//    rawADC = ADC_DataSingleGet(ADC0);
//
//    BattVolts = ((rawADC - 1700) / 8);   // adjust to a percentage
//    if(rawADC < 1700) BattVolts = 5;    // if too low just say 5 percent
//    else if(rawADC > 2500) BattVolts = 100;  // if above 3 volts limit at 100 percent
//
//    return rawADC;
//
//}





/**************************************************************************//**
 * @brief Convert ADC sample values to celsius.
 * @detail See section 25.3.4.1 in the reference manual for detail on
 *   temperature measurement and conversion.
 * @param adcSample Raw value from ADC to be converted to celsius
 * @return The temperature in degrees celsius.
 *****************************************************************************/
//float ConvertToCelsius(int32_t adcSample)
//{
//  uint32_t calTemp0;
//  uint32_t calValue0;
//  int32_t readDiff;
//  float temp;
//
//  /* Factory calibration temperature from device information page. */
//  calTemp0 = ((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
//              >> _DEVINFO_CAL_TEMP_SHIFT);
//
//  calValue0 = ((DEVINFO->ADC0CAL3
//                /* _DEVINFO_ADC0CAL3_TEMPREAD1V25_MASK is not correct in
//                    current CMSIS. This is a 12-bit value, not 16-bit. */
//                & 0xFFF0)
//               >> _DEVINFO_ADC0CAL3_TEMPREAD1V25_SHIFT);
//
//  if ((calTemp0 == 0xFF) || (calValue0 == 0xFFF))
//  {
//    /* The temperature sensor is not calibrated */
//    return -100.0;
//  }
//
//  /* Vref = 1250mV
//     TGRAD_ADCTH = 1.835 mV/degC (from datasheet)
//  */
//  readDiff = calValue0 - adcSample;
//  temp     = ((float)readDiff * 1250);
//  temp    /= (4096 * -1.835);
//
//  /* Calculate offset from calibration temperature */
//  temp     = (float)calTemp0 - temp;
//  return temp;
//}



void swSetupAdvBeaconing(void)
{
  /* This function sets up a custom advertisement package according to Smartwave specifications.
   * The advertisement package is 30 bytes long.
   */
//  advertise_stop();
   adv_set_data();
//   char_send('D');
  /* Start advertising in user mode and enable connections */
  if(MYdevice.operatingMode == unregsleep || MYdevice.operatingMode == factorysleep)  // in sleep don't advertise
    {
      advertise_stop();
    }
  else
    {
      advertise_start();
    }

}

void enterStandby(void)
{
//    gecko_cmd_le_gap_set_mode(le_gap_non_discoverable, le_gap_non_connectable);  // stop advertising we are in factory mode
}

void exitStandby(void)
{
//    gecko_cmd_le_gap_set_advertise_timing(0, 2400, 2400, 0, 0);
//    gecko_cmd_le_gap_set_mode(le_gap_user_data, le_gap_undirected_connectable);
}

